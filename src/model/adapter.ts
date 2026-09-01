import { GROUND_ID, type Actuator, type Body, type ContactPlane, type ContactSphere, type Hinge, type Profile, type SimSettings } from '../types';
import type { ActuatorSpec, BodySpec, DofParams, HingeSpec, ModelSpec } from '../dyn/model';
import { compileExpr } from '../dyn/expr';
import { orderHingeIds } from './topology';

/**
 * Translating the edited model into something the solver can run.
 *
 * This is the only place that knows both vocabularies. Everything in `src/dyn` takes plain
 * indexed data; everything in the store deals in ids and user-facing shapes. Keeping the
 * seam here is what lets the physics be tested without a store and the store be edited
 * without touching the physics.
 */

export type BuildResult =
  | { ok: true; spec: ModelSpec; bodyIndex: Map<string, number>; problems: BuildProblem[] }
  | { ok: false; problems: BuildProblem[] };

export type BuildProblem = {
  kind: 'cycle' | 'orphan' | 'expression' | 'missing';
  message: string;
  targetId?: string;
};

/**
 * Turn a profile into a scalar function of time.
 *
 * A bad expression yields a constant zero and a reported problem rather than throwing: the
 * rest of the model still builds, still renders, and still runs, which is far more useful
 * than a blank screen while you fix a typo.
 */
export function compileProfile(profile: Profile): { fn: (t: number) => number; error?: string } {
  switch (profile.kind) {
    case 'constant':
      return { fn: () => 1 };
    case 'step': {
      const { tOn, tOff } = profile;
      return { fn: (t) => (t >= tOn && t < tOff ? 1 : 0) };
    }
    case 'ramp': {
      const { t0, t1, from, to } = profile;
      const span = t1 - t0;
      return {
        fn: (t) => {
          if (t <= t0) return from;
          if (t >= t1 || span === 0) return to;
          return from + ((t - t0) / span) * (to - from);
        },
      };
    }
    case 'sine': {
      const { frequency, phase, offset } = profile;
      const w = 2 * Math.PI * frequency;
      return { fn: (t) => offset + Math.sin(w * t + phase) };
    }
    case 'impulse': {
      // Unit impulse: the height is 1/width so the total delivered impulse is independent
      // of how narrow you make it, which is what makes the knob mean something.
      const { t0, width } = profile;
      const w = Math.max(width, 1e-9);
      return { fn: (t) => (t >= t0 && t < t0 + w ? 1 / w : 0) };
    }
    case 'expr': {
      const compiled = compileExpr(profile.source);
      if (!compiled.ok) return { fn: () => 0, error: compiled.error };
      return { fn: compiled.fn };
    }
  }
}

const dofParamsOf = (hinge: Hinge): DofParams[] =>
  hinge.dof.map((d) => ({
    stiffness: d.stiffness,
    rest: d.rest,
    damping: d.damping,
    friction: d.friction,
    stiction: d.stiction,
    limitEnabled: d.limit.enabled,
    limitLo: d.limit.lo,
    limitHi: d.limit.hi,
    limitStiffness: d.limit.stiffness,
  }));

/**
 * The centre of mass in body coordinates.
 *
 * The CoM is a node like any other, which is what lets it be positioned independently of
 * the frame origin — the two are different points on most real bodies, and conflating them
 * is a quiet source of wrong answers.
 */
function comOf(body: Body): [number, number, number] {
  const node = body.nodes[body.comNodeId];
  return node ? [...node.position] : [0, 0, 0];
}

export function buildSpec(
  bodies: Record<string, Body>,
  hinges: Record<string, Hinge>,
  actuators: Record<string, Actuator>,
  settings: SimSettings,
  contactSpheres: Record<string, ContactSphere> = {},
  contactPlanes: Record<string, ContactPlane> = {},
): BuildResult {
  const problems: BuildProblem[] = [];

  const { ordered, cycle } = orderHingeIds(hinges);
  if (cycle) {
    return {
      ok: false,
      problems: [
        {
          kind: 'cycle',
          message: `These bodies form a closed loop: ${cycle
            .map((id) => bodies[id]?.name ?? id)
            .join(' → ')}. Reduced coordinates describe trees only.`,
        },
      ],
    };
  }

  // Solver body indices follow hinge order, so a parent always has a lower index than its
  // children — which is exactly the ordering the recursions need.
  const bodyIndex = new Map<string, number>();
  const bodySpecs: BodySpec[] = [];
  const hingeSpecs: HingeSpec[] = [];

  for (const hingeId of ordered) {
    const hinge = hinges[hingeId]!;
    const child = bodies[hinge.childBodyId];
    if (!child) {
      problems.push({
        kind: 'missing',
        message: `Hinge "${hinge.name}" points at a body that no longer exists.`,
        targetId: hingeId,
      });
      continue;
    }
    bodyIndex.set(child.id, bodySpecs.length);
    bodySpecs.push({
      name: child.name,
      mass: child.mass,
      com: comOf(child),
      inertia: {
        ixx: child.inertia.ixx,
        iyy: child.inertia.iyy,
        izz: child.inertia.izz,
        ixy: child.inertia.ixy,
        ixz: child.inertia.ixz,
        iyz: child.inertia.iyz,
      },
      inertiaAbout: child.inertia.about,
    });
  }

  for (const hingeId of ordered) {
    const hinge = hinges[hingeId]!;
    const child = bodies[hinge.childBodyId];
    const parent = bodies[hinge.parentBodyId];
    if (!child || !parent) continue;

    const parentNode = parent.nodes[hinge.parentNodeId] ?? parent.nodes[parent.originNodeId];
    const childNode = child.nodes[hinge.childNodeId] ?? child.nodes[child.originNodeId];
    if (!parentNode || !childNode) {
      problems.push({
        kind: 'missing',
        message: `Hinge "${hinge.name}" attaches to a node that no longer exists.`,
        targetId: hingeId,
      });
      continue;
    }

    hingeSpecs.push({
      name: hinge.name,
      parent: hinge.parentBodyId === GROUND_ID ? -1 : (bodyIndex.get(hinge.parentBodyId) ?? -1),
      child: bodyIndex.get(hinge.childBodyId)!,
      parentNodePos: parentNode.position,
      parentNodeQuat: parentNode.orientation,
      mount: hinge.mount,
      childNodePos: childNode.position,
      childNodeQuat: childNode.orientation,
      free: hinge.dof.map((d) => d.free),
      values: hinge.dof.map((d) => d.q0),
      rates: hinge.dof.map((d) => d.u0),
      params: dofParamsOf(hinge),
    });
  }

  const actuatorSpecs: ActuatorSpec[] = [];
  for (const actuator of Object.values(actuators)) {
    if (!actuator.enabled) continue;
    const body = bodies[actuator.bodyId];
    const index = bodyIndex.get(actuator.bodyId);
    if (!body || index === undefined) {
      if (actuator.bodyId === GROUND_ID) {
        problems.push({
          kind: 'orphan',
          message: `"${actuator.name}" acts on Ground, which cannot move. It will do nothing.`,
          targetId: actuator.id,
        });
      }
      continue;
    }
    const node = body.nodes[actuator.nodeId] ?? body.nodes[body.originNodeId];
    if (!node) continue;

    const { fn, error } = compileProfile(actuator.profile);
    if (error) {
      problems.push({
        kind: 'expression',
        message: `"${actuator.name}" has an invalid expression: ${error}`,
        targetId: actuator.id,
      });
    }

    // A body-fixed vector is written in the node's own axes, so the node's orientation is
    // folded in here; a world-fixed one is already in world axes and passes straight
    // through.
    const vector =
      actuator.frame === 'body'
        ? rotateByQuat(actuator.vector, node.orientation)
        : [...actuator.vector];

    actuatorSpecs.push({
      name: actuator.name,
      body: index,
      kind: actuator.kind,
      frame: actuator.frame,
      point: [...node.position],
      vector,
      profile: fn,
    });
  }

  const sphereSpecs = Object.values(contactSpheres).flatMap((sphere) => {
    if (!sphere.enabled) return [];
    const body = bodies[sphere.bodyId];
    const index = bodyIndex.get(sphere.bodyId);
    if (!body || index === undefined) return [];
    const node = body.nodes[sphere.nodeId] ?? body.nodes[body.originNodeId];
    if (!node) return [];
    return [{
      name: sphere.name,
      body: index,
      point: [...node.position],
      radius: sphere.radius,
      material: { ...sphere.material },
    }];
  });

  const planeSpecs = Object.values(contactPlanes)
    .filter((plane) => plane.enabled)
    .map((plane) => ({
      name: plane.name,
      point: [...plane.point],
      normal: [...plane.normal],
      material: { ...plane.material },
    }));

  return {
    ok: true,
    spec: {
      bodies: bodySpecs,
      hinges: hingeSpecs,
      actuators: actuatorSpecs,
      contactSpheres: sphereSpecs,
      contactPlanes: planeSpecs,
      gravity: settings.gravity,
    },
    bodyIndex,
    problems,
  };
}

/** Rotate a vector by a quaternion `[x, y, z, w]`. */
export function rotateByQuat(v: readonly number[], q: readonly number[]): [number, number, number] {
  const [x, y, z, w] = [q[0]!, q[1]!, q[2]!, q[3]!];
  const [vx, vy, vz] = [v[0]!, v[1]!, v[2]!];
  // t = 2·(q_vec × v), then v' = v + w·t + q_vec × t
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ];
}
