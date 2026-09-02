import * as THREE from 'three';
import type { Actuator, Body, ContactHeightfield, ContactPlane, ContactSphere, Hinge, Quat, SimSettings, SpringDamper, Vec3 } from '../types';
import { buildSpec } from '../model/adapter';
import {
  buildModel,
  makeKinematicsScratch,
  updateKinematics,
  updateVelocities,
  type MultibodyModel,
} from '../dyn/model';
import { forwardDynamics, makeDynamics } from '../dyn/forward';
import { applyMotion, crossMotion, sv, transformCompose, v3 } from '../dyn/spatial';
import { transform } from '../dyn/spatial';

/**
 * Turning a state vector back into poses the scene can draw.
 *
 * Body poses are derived from `q` on demand rather than stored per frame. The kinematics
 * pass costs microseconds, so recomputing it for whichever frame is on screen is far
 * cheaper than carrying a pose per body per frame through the worker boundary and around
 * in memory — and it keeps the trajectory format to just the state.
 */

export type SolverModel = {
  model: MultibodyModel;
  /** Body id → link index. Link `i` is the body pushed `i`th, so these coincide. */
  linkOf: Map<string, number>;
};

/** Axes used to express body-motion vectors in Run. */
export type MotionFrame = 'world' | 'body';

/** Linear and angular components of a body quantity, both measured at the body origin. */
export type BodyVector = { linear: Vec3; angular: Vec3 };

/** Velocity and inertial acceleration at a body's frame origin. */
export type BodyMotion = { velocity: BodyVector; acceleration: BodyVector };

export function buildSolverModel(
  bodies: Record<string, Body>,
  hinges: Record<string, Hinge>,
  actuators: Record<string, Actuator>,
  settings: SimSettings,
  springDampers: Record<string, SpringDamper> = {},
  contactSpheres: Record<string, ContactSphere> = {},
  contactPlanes: Record<string, ContactPlane> = {},
  contactHeightfields: Record<string, ContactHeightfield> = {},
): SolverModel | { error: string } {
  const built = buildSpec(bodies, hinges, actuators, settings, contactSpheres, contactPlanes, springDampers, contactHeightfields);
  if (!built.ok) return { error: built.problems[0]?.message ?? 'The model could not be assembled.' };
  try {
    return { model: buildModel(built.spec), linkOf: built.bodyIndex };
  } catch (err) {
    return { error: err instanceof Error ? err.message : String(err) };
  }
}

export type BodyPose = { position: Vec3; quaternion: Quat };

const matrix = new THREE.Matrix4();
const quaternion = new THREE.Quaternion();

/**
 * World poses of every body frame at a configuration.
 *
 * The solver works in link frames — each body's inbound-hinge child node — so each pose is
 * carried the last fixed step out to the body frame before being handed over. Ground is not
 * a link and is always the identity.
 */
export function bodyPoses(
  solver: SolverModel,
  q: Float64Array,
  velocities?: Float64Array,
): Map<string, BodyPose> {
  const { model, linkOf } = solver;
  const scratch = makeKinematicsScratch();
  const zero = new Float64Array(model.nv);
  updateKinematics(model, q, velocities ?? zero, scratch);
  if (velocities) updateVelocities(model, scratch);

  const out = new Map<string, BodyPose>();
  const world = transform();

  for (const [bodyId, index] of linkOf) {
    const link = model.links[index];
    if (!link) continue;
    transformCompose(link.Xworld, link.linkToBody, world, scratch.v);

    // `world` maps world → body, so the drawable rotation is its transpose.
    const e = world.E;
    matrix.set(
      e[0]!, e[3]!, e[6]!, 0,
      e[1]!, e[4]!, e[7]!, 0,
      e[2]!, e[5]!, e[8]!, 0,
      0, 0, 0, 1,
    );
    quaternion.setFromRotationMatrix(matrix);
    out.set(bodyId, {
      position: [world.r[0]!, world.r[1]!, world.r[2]!],
      quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
    });
  }
  return out;
}

/** The world position of a node, given its body's pose. */
export function nodeWorldPosition(pose: BodyPose, local: Vec3): Vec3 {
  const q = new THREE.Quaternion(...pose.quaternion);
  const p = new THREE.Vector3(...local).applyQuaternion(q);
  return [p.x + pose.position[0], p.y + pose.position[1], p.z + pose.position[2]];
}

/**
 * Linear and angular velocity of each body, in world axes.
 *
 * Reported at the **body frame origin**, which is where the readout says it is measuring.
 * The solver's spatial velocity is expressed at the link origin, so the linear half picks
 * up an `ω × r` term when it is carried across — dropping that term is a classic way to get
 * plausible-looking but wrong velocities on an offset body.
 */
export function bodyVelocities(
  solver: SolverModel,
  q: Float64Array,
  v: Float64Array,
  frame: MotionFrame = 'world',
): Map<string, BodyVector> {
  const { model, linkOf } = solver;
  const scratch = makeKinematicsScratch();
  updateKinematics(model, q, v, scratch);
  updateVelocities(model, scratch);

  const out = new Map<string, BodyVector>();

  for (const [bodyId, index] of linkOf) {
    const link = model.links[index];
    if (!link) continue;
    out.set(bodyId, bodyVelocity(link, frame));
  }
  return out;
}

/** Rotate a link-coordinate vector into either world or the body's own axes. */
function expressVector(link: MultibodyModel['links'][number], x: number, y: number, z: number, frame: MotionFrame): Vec3 {
  const e = frame === 'world' ? link.Xworld.E : link.linkToBody.E;
  // Xworld maps world → link, so its transpose carries vectors out to world. linkToBody,
  // in contrast, already maps link → body and is applied directly.
  if (frame === 'world') {
    return [
      e[0]! * x + e[3]! * y + e[6]! * z,
      e[1]! * x + e[4]! * y + e[7]! * z,
      e[2]! * x + e[5]! * y + e[8]! * z,
    ];
  }
  return [
    e[0]! * x + e[1]! * y + e[2]! * z,
    e[3]! * x + e[4]! * y + e[5]! * z,
    e[6]! * x + e[7]! * y + e[8]! * z,
  ];
}

/** Velocity at the body-frame origin, expressed in the requested axes. */
function bodyVelocity(link: MultibodyModel['links'][number], frame: MotionFrame): BodyVector {
  const wx = link.v[0]!, wy = link.v[1]!, wz = link.v[2]!;
  // linkToBody.r is the link origin as seen from the body, so its negative is the fixed
  // vector from link origin to body origin, in link axes.
  const px = -link.linkToBody.r[0]!, py = -link.linkToBody.r[1]!, pz = -link.linkToBody.r[2]!;
  const linearX = link.v[3]! + wy * pz - wz * py;
  const linearY = link.v[4]! + wz * px - wx * pz;
  const linearZ = link.v[5]! + wx * py - wy * px;
  return {
    angular: expressVector(link, wx, wy, wz, frame),
    linear: expressVector(link, linearX, linearY, linearZ, frame),
  };
}

/**
 * Spatial accelerations from a generalized acceleration, with a zero inertial base.
 *
 * RNEA intentionally starts at −gravity to turn gravity into an inertial load. That is
 * exactly right for forces, but not for a Run readout: a freely falling body has a real
 * inertial acceleration of g, not zero. This small forward pass therefore uses a zero base.
 */
function inertialLinkAccelerations(model: MultibodyModel, qdd: Float64Array): Float64Array[] {
  const acceleration = model.links.map(() => sv());
  const base = sv();
  const tmp = sv();
  const scratch = v3();

  for (let i = 0; i < model.links.length; i++) {
    const link = model.links[i]!;
    const parent = link.parent < 0 ? base : acceleration[link.parent]!;
    const a = acceleration[i]!;
    applyMotion(link.X, parent, a, scratch);
    crossMotion(link.v, link.jw.vJ, tmp);
    for (let r = 0; r < 6; r++) a[r] = a[r]! + link.jw.cJ[r]! + tmp[r]!;

    const { S } = link.jw;
    for (let c = 0; c < link.joint.nv; c++) {
      const value = qdd[link.joint.vOffset + c]!;
      if (value === 0) continue;
      const baseIndex = 6 * c;
      for (let r = 0; r < 6; r++) a[r] = a[r]! + value * S[baseIndex + r]!;
    }
  }
  return acceleration;
}

/**
 * Build a reusable evaluator for physical body motion. A chart needs every sample, so the
 * dynamics work buffers live here instead of allocating a fresh solver for each frame.
 */
export function makeBodyMotionEvaluator(solver: SolverModel): {
  at: (q: Float64Array, v: Float64Array, time: number, frame?: MotionFrame) => Map<string, BodyMotion>;
} {
  const dynamics = makeDynamics(solver.model);
  const qdd = new Float64Array(solver.model.nv);

  return {
    at(q, v, time, frame = 'world') {
      qdd.fill(0);
      forwardDynamics(dynamics, q, v, time, qdd);
      // forwardDynamics intentionally returns immediately for a zero-DOF model. Refreshing
      // kinematics here also makes that harmless case report its (zero) motion correctly.
      updateKinematics(solver.model, q, v, dynamics.kin);
      updateVelocities(solver.model, dynamics.kin);
      const linkAcceleration = inertialLinkAccelerations(solver.model, qdd);
      const out = new Map<string, BodyMotion>();

      for (const [bodyId, index] of solver.linkOf) {
        const link = solver.model.links[index];
        const a = linkAcceleration[index];
        if (!link || !a) continue;

        const velocity = bodyVelocity(link, frame);
        const wx = link.v[0]!, wy = link.v[1]!, wz = link.v[2]!;
        const ax = a[0]!, ay = a[1]!, az = a[2]!;
        const px = -link.linkToBody.r[0]!, py = -link.linkToBody.r[1]!, pz = -link.linkToBody.r[2]!;
        // a(P) = a(O) + α × r + ω × (ω × r) for a point fixed to the body.
        const alphaCrossPX = ay * pz - az * py;
        const alphaCrossPY = az * px - ax * pz;
        const alphaCrossPZ = ax * py - ay * px;
        const omegaCrossPX = wy * pz - wz * py;
        const omegaCrossPY = wz * px - wx * pz;
        const omegaCrossPZ = wx * py - wy * px;
        const centripetalX = wy * omegaCrossPZ - wz * omegaCrossPY;
        const centripetalY = wz * omegaCrossPX - wx * omegaCrossPZ;
        const centripetalZ = wx * omegaCrossPY - wy * omegaCrossPX;
        out.set(bodyId, {
          velocity,
          acceleration: {
            angular: expressVector(link, ax, ay, az, frame),
            linear: expressVector(
              link,
              a[3]! + alphaCrossPX + centripetalX,
              a[4]! + alphaCrossPY + centripetalY,
              a[5]! + alphaCrossPZ + centripetalZ,
              frame,
            ),
          },
        });
      }
      return out;
    },
  };
}

/** Acceleration at every body origin, evaluated from the model's equations at this instant. */
export function bodyAccelerations(
  solver: SolverModel,
  q: Float64Array,
  v: Float64Array,
  time: number,
  frame: MotionFrame = 'world',
): Map<string, BodyVector> {
  const motion = makeBodyMotionEvaluator(solver).at(q, v, time, frame);
  return new Map([...motion].map(([id, value]) => [id, value.acceleration]));
}

/**
 * Total linear and angular momentum about the world origin.
 *
 * Worth having as a readout in its own right: with no external forces both are conserved
 * exactly, so watching them wander is the most direct statement a sanity-check tool can
 * make about whether it is still telling the truth.
 */
export function totalMomentum(
  solver: SolverModel,
  q: Float64Array,
  v: Float64Array,
): { linear: Vec3; angular: Vec3 } {
  const { model } = solver;
  const scratch = makeKinematicsScratch();
  updateKinematics(model, q, v, scratch);
  updateVelocities(model, scratch);

  const linear: Vec3 = [0, 0, 0];
  const angular: Vec3 = [0, 0, 0];

  for (const link of model.links) {
    const w = link.v;
    const I = link.I;

    // Spatial momentum in link coordinates: [I·ω + h × v ; m·v + ω × h].
    const n0 = I.I[0]! * w[0]! + I.I[1]! * w[1]! + I.I[2]! * w[2]! + (I.h[1]! * w[5]! - I.h[2]! * w[4]!);
    const n1 = I.I[3]! * w[0]! + I.I[4]! * w[1]! + I.I[5]! * w[2]! + (I.h[2]! * w[3]! - I.h[0]! * w[5]!);
    const n2 = I.I[6]! * w[0]! + I.I[7]! * w[1]! + I.I[8]! * w[2]! + (I.h[0]! * w[4]! - I.h[1]! * w[3]!);
    const f0 = I.m * w[3]! + (w[1]! * I.h[2]! - w[2]! * I.h[1]!);
    const f1 = I.m * w[4]! + (w[2]! * I.h[0]! - w[0]! * I.h[2]!);
    const f2 = I.m * w[5]! + (w[0]! * I.h[1]! - w[1]! * I.h[0]!);

    // Carry to world: rotate both halves out, then shift the moment to the world origin.
    const e = link.Xworld.E;
    const rot = (x: number, y: number, z: number): Vec3 => [
      e[0]! * x + e[3]! * y + e[6]! * z,
      e[1]! * x + e[4]! * y + e[7]! * z,
      e[2]! * x + e[5]! * y + e[8]! * z,
    ];
    const nWorld = rot(n0, n1, n2);
    const fWorld = rot(f0, f1, f2);
    const r = link.Xworld.r;

    linear[0] += fWorld[0];
    linear[1] += fWorld[1];
    linear[2] += fWorld[2];
    angular[0] += nWorld[0] + (r[1]! * fWorld[2] - r[2]! * fWorld[1]);
    angular[1] += nWorld[1] + (r[2]! * fWorld[0] - r[0]! * fWorld[2]);
    angular[2] += nWorld[2] + (r[0]! * fWorld[1] - r[1]! * fWorld[0]);
  }

  return { linear, angular };
}
