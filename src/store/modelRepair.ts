import {
  GROUND_ID,
  type Actuator,
  type Body,
  type Conventions,
  type DofSpec,
  type Hinge,
  type Inertia,
  type Node,
  type Profile,
  type Quat,
  type SimSettings,
  type Vec3,
} from '../types';
import {
  ACTUATOR_COLORS,
  BODY_COLORS,
  DEFAULT_CONVENTIONS,
  DEFAULT_SETTINGS,
  groundBody,
  IDENTITY_QUAT,
  makeNode,
  neutralDof,
} from './defaults';
import { wouldCreateCycle } from '../model/topology';

/**
 * Repairing a persisted or shared model.
 *
 * Two untrusted sources feed the store: localStorage, which an older version of this app
 * may have written, and a shared URL, which can be truncated by a chat client or edited by
 * hand. Neither is worth trusting, and neither is worth throwing on: a model that has lost
 * a node should come back missing that node, not as a blank screen.
 *
 * So every field is checked and defaulted, and structural invariants the rest of the app
 * relies on are *restored* rather than merely validated — a body without an inbound hinge
 * gets one, a hinge pointing at a deleted node is re-pointed, and a cyclic hinge set is
 * broken by re-rooting the offenders onto ground.
 */

export type ModelPersisted = {
  bodies: Record<string, Body>;
  bodyOrder: string[];
  hinges: Record<string, Hinge>;
  hingeOrder: string[];
  actuators: Record<string, Actuator>;
  actuatorOrder: string[];
  settings: SimSettings;
  conventions: Conventions;
  selectedBodyId: string;
  selectedHingeId: string | null;
  selectedActuatorId: string | null;
};

const isObject = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null && !Array.isArray(value);

const num = (value: unknown, fallback: number): number =>
  typeof value === 'number' && Number.isFinite(value) ? value : fallback;

const str = (value: unknown, fallback: string): string =>
  typeof value === 'string' && value.length > 0 ? value : fallback;

const bool = (value: unknown, fallback: boolean): boolean =>
  typeof value === 'boolean' ? value : fallback;

function vec3(value: unknown, fallback: Vec3 = [0, 0, 0]): Vec3 {
  if (!Array.isArray(value)) return [...fallback];
  return [num(value[0], fallback[0]), num(value[1], fallback[1]), num(value[2], fallback[2])];
}

/** A quaternion, renormalized. A degenerate one falls back to identity rather than to NaN. */
function quat(value: unknown): Quat {
  if (!Array.isArray(value)) return [...IDENTITY_QUAT] as Quat;
  const q: Quat = [num(value[0], 0), num(value[1], 0), num(value[2], 0), num(value[3], 1)];
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-9) return [...IDENTITY_QUAT] as Quat;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function repairNode(value: unknown, id: string, index: number): Node {
  const raw = isObject(value) ? value : {};
  return {
    id,
    name: str(raw.name, `Node ${index + 1}`),
    position: vec3(raw.position),
    orientation: quat(raw.orientation),
  };
}

function repairInertia(value: unknown): Inertia {
  const raw = isObject(value) ? value : {};
  return {
    about: raw.about === 'origin' ? 'origin' : 'com',
    ixx: num(raw.ixx, 0.1),
    iyy: num(raw.iyy, 0.1),
    izz: num(raw.izz, 0.1),
    ixy: num(raw.ixy, 0),
    ixz: num(raw.ixz, 0),
    iyz: num(raw.iyz, 0),
  };
}

function repairDof(value: unknown): DofSpec {
  const raw = isObject(value) ? value : {};
  const limit = isObject(raw.limit) ? raw.limit : {};
  return {
    free: bool(raw.free, false),
    q0: num(raw.q0, 0),
    u0: num(raw.u0, 0),
    stiffness: num(raw.stiffness, 0),
    rest: num(raw.rest, 0),
    damping: num(raw.damping, 0),
    friction: num(raw.friction, 0),
    stiction: num(raw.stiction, 0),
    limit: {
      enabled: bool(limit.enabled, false),
      lo: num(limit.lo, -1),
      hi: num(limit.hi, 1),
      stiffness: num(limit.stiffness, 1000),
    },
  };
}

function repairProfile(value: unknown): Profile {
  if (!isObject(value)) return { kind: 'constant' };
  switch (value.kind) {
    case 'step':
      return { kind: 'step', tOn: num(value.tOn, 0), tOff: num(value.tOff, 1) };
    case 'ramp':
      return {
        kind: 'ramp',
        t0: num(value.t0, 0),
        t1: num(value.t1, 1),
        from: num(value.from, 0),
        to: num(value.to, 1),
      };
    case 'sine':
      return {
        kind: 'sine',
        frequency: num(value.frequency, 1),
        phase: num(value.phase, 0),
        offset: num(value.offset, 0),
      };
    case 'impulse':
      return { kind: 'impulse', t0: num(value.t0, 0), width: num(value.width, 0.05) };
    case 'expr':
      // Not validated here: an unparseable expression is reported by the diagnostics with a
      // caret at the offending character, which is far more useful than silently discarding
      // what the user wrote.
      return { kind: 'expr', source: str(value.source, 't') };
    default:
      return { kind: 'constant' };
  }
}

function repairBody(value: unknown, id: string, index: number): Body {
  const raw = isObject(value) ? value : {};

  const rawNodes = isObject(raw.nodes) ? raw.nodes : {};
  const nodes: Record<string, Node> = {};
  let n = 0;
  for (const [key, node] of Object.entries(rawNodes)) {
    nodes[key] = repairNode(node, key, n++);
  }
  // A body with no nodes at all cannot be attached to anything, so give it one.
  if (Object.keys(nodes).length === 0) {
    const origin = makeNode(`${id}-origin`, 'Origin');
    nodes[origin.id] = origin;
  }

  const order = Array.isArray(raw.nodeOrder)
    ? raw.nodeOrder.filter((key): key is string => typeof key === 'string' && key in nodes)
    : [];
  for (const key of Object.keys(nodes)) if (!order.includes(key)) order.push(key);

  const originNodeId = typeof raw.originNodeId === 'string' && nodes[raw.originNodeId] ? raw.originNodeId : order[0]!;
  const comNodeId = typeof raw.comNodeId === 'string' && nodes[raw.comNodeId] ? raw.comNodeId : originNodeId;

  // The origin node is at zero by construction; a payload that says otherwise is repaired
  // by shifting every node, which keeps the body's shape intact.
  const originOffset = nodes[originNodeId]!.position;
  if (originOffset[0] !== 0 || originOffset[1] !== 0 || originOffset[2] !== 0) {
    for (const key of Object.keys(nodes)) {
      const node = nodes[key]!;
      nodes[key] = {
        ...node,
        position: [
          node.position[0] - originOffset[0],
          node.position[1] - originOffset[1],
          node.position[2] - originOffset[2],
        ],
      };
    }
  }

  return {
    id,
    name: str(raw.name, `Body ${index + 1}`),
    nodes,
    nodeOrder: order,
    originNodeId,
    comNodeId,
    mass: num(raw.mass, 1),
    inertia: repairInertia(raw.inertia),
    color: str(raw.color, BODY_COLORS[index % BODY_COLORS.length]!),
    visible: bool(raw.visible, true),
    ...(raw.isGround === true ? { isGround: true as const } : {}),
  };
}

function repairSettings(value: unknown): SimSettings {
  const raw = isObject(value) ? value : {};
  const units = raw.units === 'imperial' || raw.units === 'generic' ? raw.units : 'si';
  const integrator = raw.integrator === 'euler' || raw.integrator === 'rk2' ? raw.integrator : 'rk4';
  return {
    units,
    gravity: vec3(raw.gravity, DEFAULT_SETTINGS.gravity),
    // Guarded against zero and negative: a non-positive step would loop forever.
    dt: Math.max(1e-9, num(raw.dt, DEFAULT_SETTINGS.dt)),
    duration: Math.max(1e-6, num(raw.duration, DEFAULT_SETTINGS.duration)),
    integrator,
    sampleRate: Math.min(1000, Math.max(1, num(raw.sampleRate, DEFAULT_SETTINGS.sampleRate))),
  };
}

function repairConventions(value: unknown): Conventions {
  const raw = isObject(value) ? value : {};
  const orders = ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'];
  return {
    upAxis: raw.upAxis === 'Y' ? 'Y' : 'Z',
    eulerOrder: (typeof raw.eulerOrder === 'string' && orders.includes(raw.eulerOrder)
      ? raw.eulerOrder
      : DEFAULT_CONVENTIONS.eulerOrder) as Conventions['eulerOrder'],
    rotationMode: raw.rotationMode === 'extrinsic' ? 'extrinsic' : 'intrinsic',
    angleUnit: raw.angleUnit === 'rad' ? 'rad' : 'deg',
  };
}

/**
 * Rebuild a model from an untrusted payload, or return null if there is nothing salvageable.
 *
 * Returning null rather than a half-model is deliberate: the caller keeps whatever it
 * already had, which is a working default, instead of replacing it with wreckage.
 */
export function repairModel(value: unknown): ModelPersisted | null {
  if (!isObject(value)) return null;

  const rawBodies = isObject(value.bodies) ? value.bodies : null;
  if (!rawBodies) return null;

  const bodies: Record<string, Body> = {};
  let index = 0;
  for (const [id, body] of Object.entries(rawBodies)) {
    bodies[id] = repairBody(body, id, index++);
  }
  // Ground is structural. If the payload lost it, put it back.
  if (!bodies[GROUND_ID]) bodies[GROUND_ID] = groundBody();
  else bodies[GROUND_ID] = { ...bodies[GROUND_ID], isGround: true };

  const bodyOrder = Array.isArray(value.bodyOrder)
    ? value.bodyOrder.filter((id): id is string => typeof id === 'string' && id in bodies)
    : [];
  for (const id of Object.keys(bodies)) if (!bodyOrder.includes(id)) bodyOrder.push(id);
  // Ground always leads the list; it is the root of everything.
  const groundAt = bodyOrder.indexOf(GROUND_ID);
  if (groundAt > 0) {
    bodyOrder.splice(groundAt, 1);
    bodyOrder.unshift(GROUND_ID);
  }

  const rawHinges = isObject(value.hinges) ? value.hinges : {};
  const hinges: Record<string, Hinge> = {};
  const claimed = new Set<string>();

  for (const [id, raw] of Object.entries(rawHinges)) {
    if (!isObject(raw)) continue;
    const childBodyId = typeof raw.childBodyId === 'string' ? raw.childBodyId : '';
    const child = bodies[childBodyId];
    // Ground is never a child, and a body may have only one inbound hinge — a payload
    // breaking either would give the solver something it cannot interpret.
    if (!child || child.isGround || claimed.has(childBodyId)) continue;

    let parentBodyId = typeof raw.parentBodyId === 'string' && bodies[raw.parentBodyId] ? raw.parentBodyId : GROUND_ID;
    if (parentBodyId === childBodyId) parentBodyId = GROUND_ID;
    const parent = bodies[parentBodyId]!;

    const rawDof = Array.isArray(raw.dof) ? raw.dof : [];
    const dof = Array.from({ length: 6 }, (_, i) => (rawDof[i] === undefined ? neutralDof() : repairDof(rawDof[i])));

    claimed.add(childBodyId);
    hinges[id] = {
      id,
      name: str(raw.name, `Hinge ${Object.keys(hinges).length + 1}`),
      parentBodyId,
      parentNodeId:
        typeof raw.parentNodeId === 'string' && parent.nodes[raw.parentNodeId]
          ? raw.parentNodeId
          : parent.originNodeId,
      childBodyId,
      childNodeId:
        typeof raw.childNodeId === 'string' && child.nodes[raw.childNodeId]
          ? raw.childNodeId
          : child.originNodeId,
      mount: quat(raw.mount),
      dof,
    };
  }

  // Any body left without an inbound hinge is attached to ground, welded, so it still
  // exists and can be re-parented by hand.
  for (const body of Object.values(bodies)) {
    if (body.isGround || claimed.has(body.id)) continue;
    const id = `hinge-${body.id}`;
    hinges[id] = {
      id,
      name: `${body.name} mount`,
      parentBodyId: GROUND_ID,
      parentNodeId: bodies[GROUND_ID]!.originNodeId,
      childBodyId: body.id,
      childNodeId: body.originNodeId,
      mount: [...IDENTITY_QUAT] as Quat,
      dof: Array.from({ length: 6 }, () => neutralDof()),
    };
  }

  // Break any cycle the payload smuggled in by re-rooting the offender onto ground. Done
  // after the whole set exists, because a cycle is a property of the set, not of one hinge.
  for (const hinge of Object.values(hinges)) {
    const others = { ...hinges };
    delete others[hinge.id];
    if (wouldCreateCycle(others, hinge.childBodyId, hinge.parentBodyId)) {
      hinges[hinge.id] = {
        ...hinge,
        parentBodyId: GROUND_ID,
        parentNodeId: bodies[GROUND_ID]!.originNodeId,
      };
    }
  }

  const hingeOrder = Array.isArray(value.hingeOrder)
    ? value.hingeOrder.filter((id): id is string => typeof id === 'string' && id in hinges)
    : [];
  for (const id of Object.keys(hinges)) if (!hingeOrder.includes(id)) hingeOrder.push(id);

  const rawActuators = isObject(value.actuators) ? value.actuators : {};
  const actuators: Record<string, Actuator> = {};
  let actuatorIndex = 0;
  for (const [id, raw] of Object.entries(rawActuators)) {
    if (!isObject(raw)) continue;
    const bodyId = typeof raw.bodyId === 'string' && bodies[raw.bodyId] ? raw.bodyId : null;
    if (!bodyId) continue;
    const body = bodies[bodyId]!;
    actuators[id] = {
      id,
      name: str(raw.name, `Actuator ${actuatorIndex + 1}`),
      kind: raw.kind === 'moment' ? 'moment' : 'force',
      bodyId,
      nodeId: typeof raw.nodeId === 'string' && body.nodes[raw.nodeId] ? raw.nodeId : body.originNodeId,
      frame: raw.frame === 'world' ? 'world' : 'body',
      vector: vec3(raw.vector, [0, 0, 1]),
      profile: repairProfile(raw.profile),
      enabled: bool(raw.enabled, true),
      color: str(raw.color, ACTUATOR_COLORS[actuatorIndex % ACTUATOR_COLORS.length]!),
    };
    actuatorIndex++;
  }

  const actuatorOrder = Array.isArray(value.actuatorOrder)
    ? value.actuatorOrder.filter((id): id is string => typeof id === 'string' && id in actuators)
    : [];
  for (const id of Object.keys(actuators)) if (!actuatorOrder.includes(id)) actuatorOrder.push(id);

  const selectedBodyId =
    typeof value.selectedBodyId === 'string' && bodies[value.selectedBodyId]
      ? value.selectedBodyId
      : (bodyOrder[bodyOrder.length - 1] ?? GROUND_ID);

  return {
    bodies,
    bodyOrder,
    hinges,
    hingeOrder,
    actuators,
    actuatorOrder,
    settings: repairSettings(value.settings),
    conventions: repairConventions(value.conventions),
    selectedBodyId,
    selectedHingeId:
      typeof value.selectedHingeId === 'string' && hinges[value.selectedHingeId]
        ? value.selectedHingeId
        : (hingeOrder[0] ?? null),
    selectedActuatorId:
      typeof value.selectedActuatorId === 'string' && actuators[value.selectedActuatorId]
        ? value.selectedActuatorId
        : (actuatorOrder[0] ?? null),
  };
}
