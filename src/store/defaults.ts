import {
  GROUND_ID,
  type Actuator,
  type Body,
  type Conventions,
  type DofSpec,
  type Hinge,
  type Node,
  type Quat,
  type SimSettings,
} from '../types';

/**
 * Fresh objects and the starting model.
 *
 * Kept apart from the store so the persistence repair layer can rebuild a default of any
 * shape without importing the store and creating a cycle.
 */

export const IDENTITY_QUAT: Quat = [0, 0, 0, 1];

export const BODY_COLORS = [
  '#f5a524',
  '#a855f7',
  '#06b6d4',
  '#ec4899',
  '#84cc16',
  '#f97316',
  '#14b8a6',
  '#8b5cf6',
];

export const ACTUATOR_COLORS = ['#fbbf24', '#22d3ee', '#f472b6', '#a3e635', '#c084fc'];

export const DEFAULT_CONVENTIONS: Conventions = {
  upAxis: 'Z',
  eulerOrder: 'ZYX',
  rotationMode: 'intrinsic',
  angleUnit: 'deg',
};

export const DEFAULT_SETTINGS: SimSettings = {
  units: 'si',
  gravity: [0, 0, -9.80665],
  dt: 0.001,
  duration: 10,
  integrator: 'rk4',
  sampleRate: 60,
};

export function neutralDof(free = false): DofSpec {
  return {
    free,
    q0: 0,
    u0: 0,
    stiffness: 0,
    rest: 0,
    damping: 0,
    friction: 0,
    limit: { enabled: false, lo: -1, hi: 1, stiffness: 1000 },
  };
}

export const neutralDofSet = (): DofSpec[] => Array.from({ length: 6 }, () => neutralDof());

export function makeNode(id: string, name: string, position: [number, number, number] = [0, 0, 0]): Node {
  return { id, name, position, orientation: [...IDENTITY_QUAT] as Quat };
}

/** The inertial frame: massless, fixed, and carrying nodes so hinges can attach anywhere. */
export function groundBody(): Body {
  const origin = makeNode('ground-origin', 'Origin');
  return {
    id: GROUND_ID,
    name: 'Ground',
    nodes: { [origin.id]: origin },
    nodeOrder: [origin.id],
    originNodeId: origin.id,
    comNodeId: origin.id,
    mass: 0,
    inertia: { about: 'origin', ixx: 0, iyy: 0, izz: 0, ixy: 0, ixz: 0, iyz: 0 },
    color: '#64718a',
    visible: true,
    isGround: true,
  };
}

export type ModelSlice = {
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

/**
 * The starting model: a two-link arm driven by a moment at its shoulder.
 *
 * Chosen because it exercises the things worth seeing immediately — a chain, a hinge whose
 * axis is not the body axis, a body whose centre of mass is nowhere near its origin, and an
 * actuator — while staying small enough to read at a glance. It also swings under gravity
 * on its own, so the very first frame is already moving.
 */
export function initialModel(): ModelSlice {
  const ground = groundBody();

  const upperRoot = makeNode('upper-root', 'Root');
  const upperTip = makeNode('upper-tip', 'Tip', [0, 0, -1.2]);
  const upper: Body = {
    id: 'upper',
    name: 'Upper Arm',
    nodes: { [upperRoot.id]: upperRoot, [upperTip.id]: upperTip },
    nodeOrder: [upperRoot.id, upperTip.id],
    originNodeId: upperRoot.id,
    // Not at the origin, and not at either end — the CoM is its own node for a reason.
    comNodeId: 'upper-com',
    mass: 3,
    inertia: { about: 'com', ixx: 0.36, iyy: 0.36, izz: 0.02, ixy: 0, ixz: 0, iyz: 0 },
    color: BODY_COLORS[0]!,
    visible: true,
  };
  const upperCom = makeNode('upper-com', 'CoM', [0, 0, -0.6]);
  upper.nodes[upperCom.id] = upperCom;
  upper.nodeOrder.splice(1, 0, upperCom.id);

  const lowerRoot = makeNode('lower-root', 'Root');
  const lowerCom = makeNode('lower-com', 'CoM', [0, 0, -0.45]);
  const lowerTip = makeNode('lower-tip', 'Tip', [0, 0, -0.9]);
  const lower: Body = {
    id: 'lower',
    name: 'Forearm',
    nodes: { [lowerRoot.id]: lowerRoot, [lowerCom.id]: lowerCom, [lowerTip.id]: lowerTip },
    nodeOrder: [lowerRoot.id, lowerCom.id, lowerTip.id],
    originNodeId: lowerRoot.id,
    comNodeId: lowerCom.id,
    mass: 1.6,
    inertia: { about: 'com', ixx: 0.11, iyy: 0.11, izz: 0.01, ixy: 0, ixz: 0, iyz: 0 },
    color: BODY_COLORS[1]!,
    visible: true,
  };

  const shoulderDof = neutralDofSet();
  shoulderDof[4] = { ...neutralDof(true), q0: 0.6 };
  const shoulder: Hinge = {
    id: 'shoulder',
    name: 'Shoulder',
    parentBodyId: GROUND_ID,
    parentNodeId: ground.originNodeId,
    childBodyId: upper.id,
    childNodeId: upperRoot.id,
    mount: [...IDENTITY_QUAT] as Quat,
    dof: shoulderDof,
  };

  const elbowDof = neutralDofSet();
  elbowDof[4] = { ...neutralDof(true), q0: -1.1, damping: 0.05 };
  const elbow: Hinge = {
    id: 'elbow',
    name: 'Elbow',
    parentBodyId: upper.id,
    parentNodeId: upperTip.id,
    childBodyId: lower.id,
    childNodeId: lowerRoot.id,
    mount: [...IDENTITY_QUAT] as Quat,
    dof: elbowDof,
  };

  const drive: Actuator = {
    id: 'drive',
    name: 'Shoulder drive',
    kind: 'moment',
    bodyId: upper.id,
    nodeId: upperRoot.id,
    frame: 'body',
    vector: [0, 12, 0],
    profile: { kind: 'step', tOn: 0, tOff: 1.5 },
    enabled: true,
    color: ACTUATOR_COLORS[0]!,
  };

  return {
    bodies: { [ground.id]: ground, [upper.id]: upper, [lower.id]: lower },
    bodyOrder: [ground.id, upper.id, lower.id],
    hinges: { [shoulder.id]: shoulder, [elbow.id]: elbow },
    hingeOrder: [shoulder.id, elbow.id],
    actuators: { [drive.id]: drive },
    actuatorOrder: [drive.id],
    settings: { ...DEFAULT_SETTINGS },
    conventions: { ...DEFAULT_CONVENTIONS },
    selectedBodyId: upper.id,
    selectedHingeId: shoulder.id,
    selectedActuatorId: drive.id,
  };
}
