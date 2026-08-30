import {
  type Inertia,
  type M3,
  type Transform,
  type V3,
  applyMotion,
  inertia as makeInertia,
  inertiaTransform,
  m3,
  m3FromQuat,
  m3Identity,
  matMul,
  matVec,
  sv,
  transform,
  transformCompose,
  v3,
} from './spatial';
import { buildInertia, type InertiaInput } from './inertia';
import {
  type JointModel,
  type JointWorkspace,
  jcalc,
  makeJointModel,
  makeJointWorkspace,
  writeInitialQ,
} from './joints';

/**
 * Assembling a solver model, and the kinematics that run on it.
 *
 * The input specs here are plain data, deliberately decoupled from the store's types: the
 * whole `src/dyn` layer knows nothing about React, zustand or the UI, which is what lets
 * the physics be tested on its own.
 *
 * ## Link frames
 *
 * Each body's solver frame is **the child-node frame of its inbound hinge**, not its body
 * frame. That choice costs one fixed transform at build time and saves one at every step:
 * the joint transform already lands exactly on that frame, so the recursion never has to
 * detour through the body frame. Body-frame and node quantities are recovered for output
 * through the fixed `linkToBody` transform.
 */

export type BodySpec = {
  name: string;
  mass: number;
  /** Centre of mass, in body coordinates. */
  com: readonly number[];
  inertia: InertiaInput;
  inertiaAbout: 'com' | 'origin';
};

/**
 * What a single joint axis does beyond being free or locked.
 *
 * Damping and friction need only the rate, so they work on every axis. Stiffness and
 * limits need a scalar coordinate, which a fully-free rotation does not have — see
 * `DofBinding.qIndex`.
 */
export type DofParams = {
  stiffness: number;
  rest: number;
  damping: number;
  friction: number;
  /** Breakaway force. Above zero the axis can be held motionless by static friction. */
  stiction: number;
  limitEnabled: boolean;
  limitLo: number;
  limitHi: number;
  limitStiffness: number;
};

export const NEUTRAL_DOF_PARAMS: DofParams = {
  stiffness: 0,
  rest: 0,
  damping: 0,
  friction: 0,
  stiction: 0,
  limitEnabled: false,
  limitLo: 0,
  limitHi: 0,
  limitStiffness: 0,
};

export type HingeSpec = {
  name: string;
  /** Index into `bodies`, or −1 to attach to ground. */
  parent: number;
  child: number;
  /** Attachment node on the parent, in parent-body coordinates (world when parent is −1). */
  parentNodePos: readonly number[];
  parentNodeQuat: readonly number[];
  /** Joint-axis orientation relative to the parent node. */
  mount: readonly number[];
  /** Attachment node on the child, in child-body coordinates. */
  childNodePos: readonly number[];
  childNodeQuat: readonly number[];
  /** Six flags: [tx, ty, tz, rx, ry, rz]. */
  free: readonly boolean[];
  /** Six values. Free axes take theirs as an initial condition; locked axes hold theirs. */
  values: readonly number[];
  /** Six rates. Free axes take theirs as an initial condition; locked axes ignore them. */
  rates: readonly number[];
  /** Six entries. Omitted means neutral: no spring, damper, friction or limit. */
  params?: readonly DofParams[];
};

/**
 * An actuator as the model layer wants it: a body index, a point and vector in *body*
 * coordinates, and a time profile already reduced to a closure.
 *
 * Translating the store's node references into these coordinates happens in the adapter,
 * so `src/dyn` never has to know what a node id is.
 */
export type ActuatorSpec = {
  name: string;
  /** Index into `bodies`. */
  body: number;
  kind: 'force' | 'moment';
  frame: 'body' | 'world';
  /** Application point, in body coordinates. Ignored for a moment, which is a free vector. */
  point: readonly number[];
  /** Direction and magnitude, in body coordinates or world coordinates per `frame`. */
  vector: readonly number[];
  /** Scalar multiplier as a function of time. */
  profile: (t: number) => number;
};

export type ModelSpec = {
  bodies: BodySpec[];
  hinges: HingeSpec[];
  actuators?: ActuatorSpec[];
  gravity: readonly number[];
};

/**
 * An actuator with its geometry resolved into the link frame it acts on.
 *
 * A body-fixed direction is constant in link coordinates and is baked in here once. A
 * world-fixed direction has to be rotated into the link frame at every evaluation, which
 * is the whole difference between a thruster that tumbles with the body and one that does
 * not.
 */
export type CompiledActuator = {
  name: string;
  link: number;
  kind: 'force' | 'moment';
  frame: 'body' | 'world';
  /** Application point in link coordinates. */
  point: V3;
  /** In link coordinates when body-fixed, in world coordinates when world-fixed. */
  vector: V3;
  profile: (t: number) => number;
};

/**
 * One entry per velocity coordinate, tying it back to its axis parameters.
 *
 * `qIndex` is −1 for the rotational axes of a fully-free joint, where the configuration is
 * a quaternion and there is no scalar angle to compare against a rest position or a stop.
 * Damping and friction still apply there; stiffness and limits cannot, and the diagnostics
 * say so rather than silently ignoring the setting.
 */
export type DofBinding = {
  link: number;
  /** 0-5, indexing [tx, ty, tz, rx, ry, rz]. */
  axis: number;
  /** Index into `q`, or −1 when this axis has no scalar coordinate. */
  qIndex: number;
  params: DofParams;
};

export type Link = {
  name: string;
  /** Index of the parent link, or −1 for a link attached directly to ground. */
  parent: number;
  joint: JointModel;
  jw: JointWorkspace;
  /** Fixed: parent link frame → this joint's parent-side frame P. */
  Xtree: Transform;
  /** Parent link frame → this link frame. Rebuilt every kinematics pass. */
  X: Transform;
  /** World → this link frame. Rebuilt every kinematics pass. */
  Xworld: Transform;
  /** Fixed: this link frame → the body frame. Output only. */
  linkToBody: Transform;
  /** Spatial inertia about the link frame origin. */
  I: Inertia;
  /** Spatial velocity in link coordinates. */
  v: Float64Array;
  /** Mass and centre of mass in body coordinates, kept for energy and diagnostics. */
  mass: number;
  comBody: V3;
};

export type MultibodyModel = {
  links: Link[];
  nq: number;
  nv: number;
  gravity: V3;
  /** Initial position and velocity state, from the hinge specs. */
  q0: Float64Array;
  v0: Float64Array;
  /** Human-readable name for each velocity coordinate, for plots and CSV headers. */
  dofNames: string[];
  /** One per velocity coordinate, in the same order. */
  dofBindings: DofBinding[];
  actuators: CompiledActuator[];
};

const DOF_SUFFIX = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz'];

/**
 * A fixed transform from a body frame to a node frame mounted on it.
 *
 * `E` maps body coordinates into the node's, so it is the transpose of the node's own
 * orientation, composed with any extra mount rotation.
 */
function bodyToNode(pos: readonly number[], quat: readonly number[], mount: readonly number[] | null, out: Transform): Transform {
  const rn = m3FromQuat(quat);
  let full = rn;
  if (mount) {
    const rm = m3FromQuat(mount);
    full = matMul(rn, rm, m3());
  }
  // Transpose into body → node.
  out.E[0] = full[0]!; out.E[1] = full[3]!; out.E[2] = full[6]!;
  out.E[3] = full[1]!; out.E[4] = full[4]!; out.E[5] = full[7]!;
  out.E[6] = full[2]!; out.E[7] = full[5]!; out.E[8] = full[8]!;
  out.r[0] = pos[0] ?? 0;
  out.r[1] = pos[1] ?? 0;
  out.r[2] = pos[2] ?? 0;
  return out;
}

/** The inverse of `bodyToNode` without the mount: node frame → body frame. */
function nodeToBody(pos: readonly number[], quat: readonly number[], out: Transform): Transform {
  m3FromQuat(quat, out.E);
  const p = v3(pos[0] ?? 0, pos[1] ?? 0, pos[2] ?? 0);
  // r = −Eᵀ·p, the body origin expressed in node coordinates.
  out.r[0] = -(out.E[0]! * p[0]! + out.E[3]! * p[1]! + out.E[6]! * p[2]!);
  out.r[1] = -(out.E[1]! * p[0]! + out.E[4]! * p[1]! + out.E[7]! * p[2]!);
  out.r[2] = -(out.E[2]! * p[0]! + out.E[5]! * p[1]! + out.E[8]! * p[2]!);
  return out;
}

/**
 * Build a solver model.
 *
 * Hinges must already be ordered parent-before-child; `orderHinges` does that and reports
 * a cycle rather than looping forever. Each body must have exactly one inbound hinge —
 * that hinge *is* its parent pointer, which is what makes the topology a tree by
 * construction.
 */
export function buildModel(spec: ModelSpec): MultibodyModel {
  const links: Link[] = [];
  const dofNames: string[] = [];
  const dofBindings: DofBinding[] = [];
  let qOffset = 0;
  let vOffset = 0;

  /** Body index → link index, so a hinge can find its parent link. */
  const linkOfBody = new Map<number, number>();
  const scratchV = v3();
  const scratchA = m3();
  const scratchB = m3();

  for (const hinge of spec.hinges) {
    const body = spec.bodies[hinge.child];
    if (!body) throw new Error(`Hinge "${hinge.name}" names an unknown child body`);

    const joint = makeJointModel(hinge.free, hinge.values, qOffset, vOffset);
    const linkIndex = links.length;
    joint.freeTrans.forEach((k, i) => {
      dofNames.push(`${body.name}.${DOF_SUFFIX[k]}`);
      dofBindings.push({
        link: linkIndex,
        axis: k,
        qIndex: joint.qOffset + i,
        params: hinge.params?.[k] ?? NEUTRAL_DOF_PARAMS,
      });
    });
    joint.freeRot.forEach((k, i) => {
      dofNames.push(`${body.name}.${DOF_SUFFIX[k + 3]}`);
      dofBindings.push({
        link: linkIndex,
        axis: k + 3,
        // A fully-free rotation is stored as a quaternion, so there is no scalar angle to
        // bind a spring or a stop to.
        qIndex: joint.useQuaternion ? -1 : joint.qOffset + joint.freeTrans.length + i,
        params: hinge.params?.[k + 3] ?? NEUTRAL_DOF_PARAMS,
      });
    });
    qOffset += joint.nq;
    vOffset += joint.nv;

    // Fixed transform onto the joint's parent-side frame.
    const parentSide = bodyToNode(hinge.parentNodePos, hinge.parentNodeQuat, hinge.mount, transform());
    const Xtree = transform();
    const parentLink = hinge.parent >= 0 ? linkOfBody.get(hinge.parent) : undefined;
    if (parentLink === undefined) {
      // Attached to ground: the parent frame is the world, so the node transform is all
      // there is.
      Xtree.E.set(parentSide.E);
      Xtree.r.set(parentSide.r);
    } else {
      transformCompose(links[parentLink]!.linkToBody, parentSide, Xtree, scratchV);
    }

    // The link frame is the child node's frame, so the inertia has to move there.
    const comBody = v3(body.com[0] ?? 0, body.com[1] ?? 0, body.com[2] ?? 0);
    const bodyInertia = buildInertia(body.mass, comBody, body.inertia, body.inertiaAbout);
    const toChildNode = bodyToNode(hinge.childNodePos, hinge.childNodeQuat, null, transform());
    const linkInertia = inertiaTransform(toChildNode, bodyInertia, makeInertia(), scratchA, scratchB);

    links.push({
      name: body.name,
      parent: parentLink ?? -1,
      joint,
      jw: makeJointWorkspace(joint),
      Xtree,
      X: transform(),
      Xworld: transform(),
      linkToBody: nodeToBody(hinge.childNodePos, hinge.childNodeQuat, transform()),
      I: linkInertia,
      v: sv(),
      mass: body.mass,
      comBody,
    });
    linkOfBody.set(hinge.child, links.length - 1);
  }

  const q0 = new Float64Array(qOffset);
  const v0 = new Float64Array(vOffset);
  for (let i = 0; i < links.length; i++) {
    const hinge = spec.hinges[i]!;
    const link = links[i]!;
    writeInitialQ(link.joint, hinge.values, q0);
    let at = link.joint.vOffset;
    for (const k of link.joint.freeTrans) v0[at++] = hinge.rates[k] ?? 0;
    for (const k of link.joint.freeRot) v0[at++] = hinge.rates[k + 3] ?? 0;
  }

  return {
    links,
    nq: qOffset,
    nv: vOffset,
    gravity: v3(spec.gravity[0] ?? 0, spec.gravity[1] ?? 0, spec.gravity[2] ?? 0),
    q0,
    v0,
    dofNames,
    dofBindings,
    actuators: compileActuators(spec, linkOfBody, spec.hinges),
  };
}

/**
 * Resolve each actuator's geometry into the link frame it acts on.
 *
 * Body coordinates reach link coordinates through the child node of the body's inbound
 * hinge: `p_link = Q_cᵀ·(p_body − p_c)`. For a body-fixed actuator that is a one-time
 * conversion; for a world-fixed one only the point converts, because the direction has to
 * be re-resolved against the body's attitude at every step.
 *
 * An actuator on a body with no inbound hinge — i.e. on ground — is dropped: pushing on the
 * inertial frame does nothing, and silently doing nothing is better than pretending.
 */
function compileActuators(
  spec: ModelSpec,
  linkOfBody: Map<number, number>,
  hinges: HingeSpec[],
): CompiledActuator[] {
  const out: CompiledActuator[] = [];
  for (const act of spec.actuators ?? []) {
    const link = linkOfBody.get(act.body);
    if (link === undefined) continue;
    const hinge = hinges[link]!;
    const pc = hinge.childNodePos;
    const qc = m3FromQuat(hinge.childNodeQuat);

    const dx = (act.point[0] ?? 0) - (pc[0] ?? 0);
    const dy = (act.point[1] ?? 0) - (pc[1] ?? 0);
    const dz = (act.point[2] ?? 0) - (pc[2] ?? 0);
    const point = v3(
      qc[0]! * dx + qc[3]! * dy + qc[6]! * dz,
      qc[1]! * dx + qc[4]! * dy + qc[7]! * dz,
      qc[2]! * dx + qc[5]! * dy + qc[8]! * dz,
    );

    const vx = act.vector[0] ?? 0, vy = act.vector[1] ?? 0, vz = act.vector[2] ?? 0;
    const vector =
      act.frame === 'body'
        ? v3(
            qc[0]! * vx + qc[3]! * vy + qc[6]! * vz,
            qc[1]! * vx + qc[4]! * vy + qc[7]! * vz,
            qc[2]! * vx + qc[5]! * vy + qc[8]! * vz,
          )
        : v3(vx, vy, vz);

    out.push({ name: act.name, link, kind: act.kind, frame: act.frame, point, vector, profile: act.profile });
  }
  return out;
}

/**
 * Order hinges so every parent comes before its children, or report why that is impossible.
 *
 * A cycle here would mean a closed kinematic loop, which reduced coordinates cannot express
 * — hence the explicit error rather than a stack overflow deep in the recursion.
 */
export function orderHinges(hinges: HingeSpec[]): { ordered: HingeSpec[]; cycle: string[] | null } {
  const byChild = new Map<number, HingeSpec>();
  for (const h of hinges) byChild.set(h.child, h);

  const ordered: HingeSpec[] = [];
  const state = new Map<number, 'visiting' | 'done'>();

  const visit = (child: number, trail: number[]): string[] | null => {
    const mark = state.get(child);
    if (mark === 'done') return null;
    if (mark === 'visiting') {
      const start = trail.indexOf(child);
      const loop = trail.slice(start === -1 ? 0 : start);
      return loop.map((c) => byChild.get(c)?.name ?? `body ${c}`);
    }
    const hinge = byChild.get(child);
    if (!hinge) return null;

    state.set(child, 'visiting');
    if (hinge.parent >= 0) {
      const cycle = visit(hinge.parent, [...trail, child]);
      if (cycle) return cycle;
    }
    state.set(child, 'done');
    ordered.push(hinge);
    return null;
  };

  for (const hinge of hinges) {
    const cycle = visit(hinge.child, []);
    if (cycle) return { ordered: [], cycle };
  }
  return { ordered, cycle: null };
}

// ---------------------------------------------------------------------------
// Kinematics
// ---------------------------------------------------------------------------

export type KinematicsScratch = {
  v: V3;
  a: M3;
  b: M3;
};

export const makeKinematicsScratch = (): KinematicsScratch => ({ v: v3(), a: m3(), b: m3() });

/**
 * Refresh every joint's kinematics and every link's transforms for a configuration.
 *
 * Called once per force evaluation, with the results left in each link's workspace for the
 * inverse-dynamics and mass-matrix passes to read. Doing it here rather than inside each
 * algorithm is what keeps a forward-dynamics evaluation to a single `jcalc` per joint
 * instead of three.
 */
export function updateKinematics(
  model: MultibodyModel,
  q: Float64Array,
  v: Float64Array,
  scratch: KinematicsScratch,
): void {
  for (const link of model.links) {
    jcalc(link.joint, q, v, link.jw);
    transformCompose(link.Xtree, link.jw.XJ, link.X, scratch.v);
    if (link.parent < 0) {
      link.Xworld.E.set(link.X.E);
      link.Xworld.r.set(link.X.r);
    } else {
      transformCompose(model.links[link.parent]!.Xworld, link.X, link.Xworld, scratch.v);
    }
  }
}

/** Propagate spatial velocities down the tree, leaving each link's `v` in link coordinates. */
export function updateVelocities(model: MultibodyModel, scratch: KinematicsScratch): void {
  for (const link of model.links) {
    if (link.parent < 0) {
      link.v.set(link.jw.vJ);
    } else {
      applyMotion(link.X, model.links[link.parent]!.v, link.v, scratch.v);
      for (let r = 0; r < 6; r++) link.v[r] = link.v[r]! + link.jw.vJ[r]!;
    }
  }
}

/**
 * A link's world pose as a position and a rotation matrix mapping link → world.
 *
 * `Xworld` stores world → link, so both halves invert: the rotation transposes and the
 * offset is the negated, rotated translation.
 */
export function linkPose(link: Link, outPos: V3, outRot: M3): void {
  const e = link.Xworld.E;
  outRot[0] = e[0]!; outRot[1] = e[3]!; outRot[2] = e[6]!;
  outRot[3] = e[1]!; outRot[4] = e[4]!; outRot[5] = e[7]!;
  outRot[6] = e[2]!; outRot[7] = e[5]!; outRot[8] = e[8]!;
  outPos.set(link.Xworld.r);
}

/** The world position of a point given in a link's coordinates. */
export function pointToWorld(link: Link, local: V3, out: V3, scratch: M3): V3 {
  const pos = v3();
  linkPose(link, pos, scratch);
  matVec(scratch, local, out);
  out[0] = out[0]! + pos[0]!;
  out[1] = out[1]! + pos[1]!;
  out[2] = out[2]! + pos[2]!;
  return out;
}

/** The world pose of a body frame, given its link's pose. */
export function bodyPose(link: Link, outPos: V3, outRot: M3, scratch: KinematicsScratch): void {
  const world = transform();
  transformCompose(link.Xworld, link.linkToBody, world, scratch.v);
  const e = world.E;
  outRot[0] = e[0]!; outRot[1] = e[3]!; outRot[2] = e[6]!;
  outRot[3] = e[1]!; outRot[4] = e[4]!; outRot[5] = e[7]!;
  outRot[6] = e[2]!; outRot[7] = e[5]!; outRot[8] = e[8]!;
  outPos.set(world.r);
}

/** Identity transform, for callers that need a fixed world frame. */
export const worldTransform = (): Transform => ({ E: m3Identity(), r: v3() });
