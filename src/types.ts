/**
 * Canonical storage types.
 *
 * Two properties hold throughout, and a lot of the design follows from them:
 *
 * 1. **Convention-free.** Rotations are stored as quaternions and angles as radians. Euler
 *    order, up-axis and degrees/radians are applied only at the presentation boundary
 *    (src/math/conventions.ts), so changing a convention never rewrites the model.
 *
 * 2. **Unit-free.** Every number here is a bare magnitude. The solver does arithmetic and
 *    nothing else — no conversion, ever. The unit system (src/units.ts) picks labels,
 *    gravity presets and which plausibility checks run; it never touches a stored value.
 */

export type Vec3 = [number, number, number];

/** Quaternion stored as [x, y, z, w] to match THREE.Quaternion.toArray(). */
export type Quat = [number, number, number, number];

// ---------------------------------------------------------------------------
// Conventions (presentation only — see src/math/conventions.ts)
// ---------------------------------------------------------------------------

/** Which world axis points "up" on screen. Purely a viewing/labelling choice. */
export type UpAxis = 'Z' | 'Y';

/** The six Tait-Bryan sequences three.js supports. */
export type EulerOrder = 'XYZ' | 'XZY' | 'YXZ' | 'YZX' | 'ZXY' | 'ZYX';

/**
 * Intrinsic: each rotation is about the *new* (already-rotated) axes.
 * Extrinsic: each rotation is about the original *fixed* world axes.
 */
export type RotationMode = 'intrinsic' | 'extrinsic';

export type AngleUnit = 'deg' | 'rad';

export type Conventions = {
  upAxis: UpAxis;
  eulerOrder: EulerOrder;
  rotationMode: RotationMode;
  angleUnit: AngleUnit;
};

// ---------------------------------------------------------------------------
// Bodies and nodes
// ---------------------------------------------------------------------------

/**
 * The root of the tree: a fixed, massless pseudo-body representing the inertial frame.
 *
 * Modelled as a Body so that nodes, hinge attachment and scene rendering have exactly one
 * code path rather than a special case at every call site.
 */
export const GROUND_ID = 'ground';

/**
 * A named point on a body, in body coordinates.
 *
 * Nodes are the only attachment points in the model: hinges connect a node on one body to
 * a node on another, and actuators push or twist at a node. The orientation lets a node
 * carry an axis triad of its own, so a hinge axis need not line up with the body axes.
 */
export type Node = {
  id: string;
  name: string;
  /** Position in body coordinates — i.e. relative to the body's origin node. */
  position: Vec3;
  /** Orientation of the node's own axes relative to the body axes. */
  orientation: Quat;
};

/**
 * Which point the inertia tensor is taken about.
 *
 * Both are common in real data sheets and mixing them up is a factor-of-`m·d²` error, so
 * it is stated per body rather than assumed. Conversion is the parallel-axis theorem
 * (src/dyn/inertia.ts) and happens only when building the solver model.
 */
export type InertiaReference = 'com' | 'origin';

export type Inertia = {
  about: InertiaReference;
  /** Moments of inertia. */
  ixx: number;
  iyy: number;
  izz: number;
  /**
   * Products of inertia, in the +∫xy convention — i.e. the tensor is
   * [[ixx, -ixy, -ixz], [-ixy, iyy, -iyz], [-ixz, -iyz, izz]].
   * Named in the UI, because the opposite sign convention is equally common.
   */
  ixy: number;
  ixz: number;
  iyz: number;
};

export type Body = {
  id: string;
  name: string;
  nodes: Record<string, Node>;
  /** Display order of nodes. */
  nodeOrder: string[];
  /**
   * The node defining the body's frame origin. Its position is [0,0,0] by construction:
   * designating a different node re-expresses every other node so the body does not move.
   */
  originNodeId: string;
  /** The node at the centre of mass. Need not be the origin node. */
  comNodeId: string;
  mass: number;
  inertia: Inertia;
  color: string;
  visible: boolean;
  /** True only for the ground pseudo-body, whose mass and inertia are ignored. */
  isGround?: boolean;
};

// ---------------------------------------------------------------------------
// Hinges
// ---------------------------------------------------------------------------

/**
 * The six joint axes, in storage order.
 *
 * Translations first, then rotations — the same order as a spatial force/motion vector's
 * angular/linear split reversed, which is a deliberate readability choice for the UI: the
 * checkbox row reads "x y z | rx ry rz".
 */
export const DOF_LABELS = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz'] as const;
export type DofIndex = 0 | 1 | 2 | 3 | 4 | 5;

export type DofSpec = {
  /** Free DOFs become generalized coordinates; locked ones are held at `q0` exactly. */
  free: boolean;
  /** Initial value. For a locked DOF this is the fixed offset it is held at. */
  q0: number;
  /** Initial rate. Ignored when locked. */
  u0: number;
  /** Linear spring stiffness, restoring toward `rest`. Zero disables it. */
  stiffness: number;
  rest: number;
  /** Viscous damping coefficient, opposing the rate. Zero disables it. */
  damping: number;
  /** Sliding (kinetic) resistance opposing motion. Zero disables it. */
  friction: number;
  /**
   * Breakaway (static) force. Below this the axis is held completely still rather than
   * creeping; above it, the axis breaks free and `friction` takes over. Zero disables it.
   */
  stiction: number;
  limit: {
    enabled: boolean;
    lo: number;
    hi: number;
    /** Penalty stiffness applied past a stop. Damped critically against the local inertia. */
    stiffness: number;
  };
};

export type Hinge = {
  id: string;
  name: string;
  parentBodyId: string;
  parentNodeId: string;
  /** Unique across hinges: each body has exactly one inbound hinge, which is its parent link. */
  childBodyId: string;
  childNodeId: string;
  /**
   * Orientation of the joint axes relative to the parent node's axes.
   *
   * Editable in the UI as either a quaternion or an Euler triple, with a live two-way
   * translation between them.
   */
  mount: Quat;
  /** Exactly six entries, indexed by DofIndex. */
  dof: DofSpec[];
};

// ---------------------------------------------------------------------------
// Actuators
// ---------------------------------------------------------------------------

export type ActuatorKind = 'force' | 'moment';

/**
 * Whether the direction is fixed to the body or fixed in the world.
 *
 * Not a detail: a thruster bolted to a tumbling body sweeps its thrust around with it,
 * while one pointing a fixed way in space does not, and the two give completely different
 * motion from identical numbers.
 */
export type ActuatorFrame = 'body' | 'world';

/**
 * A scalar multiplier on the actuator vector, as a function of time.
 *
 * The GUI profiles cover the common cases without typing; `expr` is the escape hatch for
 * anything else and is parsed, not eval'd (src/dyn/expr.ts).
 */
export type Profile =
  | { kind: 'constant' }
  | { kind: 'step'; tOn: number; tOff: number }
  | { kind: 'ramp'; t0: number; t1: number; from: number; to: number }
  | { kind: 'sine'; frequency: number; phase: number; offset: number }
  | { kind: 'impulse'; t0: number; width: number }
  | { kind: 'expr'; source: string };

export type Actuator = {
  id: string;
  name: string;
  kind: ActuatorKind;
  bodyId: string;
  nodeId: string;
  frame: ActuatorFrame;
  /** Direction and magnitude, in the node's axes (body frame) or world axes. */
  vector: Vec3;
  profile: Profile;
  enabled: boolean;
  color: string;
};

// ---------------------------------------------------------------------------
// Contact geometry
// ---------------------------------------------------------------------------

export type ContactMaterial = {
  stiffness: number;
  damping: number;
};

/** A sphere centred on a body node. Radius zero is an exact point contact. */
export type ContactSphere = {
  id: string;
  name: string;
  bodyId: string;
  nodeId: string;
  radius: number;
  material: ContactMaterial;
  enabled: boolean;
};

/** A fixed, one-sided plane in world coordinates. The normal points to the allowed side. */
export type ContactPlane = {
  id: string;
  name: string;
  point: Vec3;
  normal: Vec3;
  material: ContactMaterial;
  enabled: boolean;
};

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

/**
 * Which unit system labels the fields and arms the plausibility checks.
 *
 * 'generic' enforces and assumes nothing: whatever you type is taken at face value.
 */
export type UnitSystem = 'si' | 'imperial' | 'generic';

/**
 * Integrator choice, trading accuracy against speed.
 *
 * RK4 is the default because a back-of-the-envelope answer that is quietly wrong is worse
 * than a slow one; `euler` (semi-implicit) is the "just show me the shape of the motion"
 * setting and is roughly four times faster per unit of simulated time.
 */
export type Integrator = 'euler' | 'rk2' | 'rk4';

export type SimSettings = {
  units: UnitSystem;
  /** World gravity vector. Not enforced to any magnitude — see src/units.ts. */
  gravity: Vec3;
  /** Integration step. */
  dt: number;
  /** Total simulated duration. */
  duration: number;
  integrator: Integrator;
  /** Frames emitted per unit time, independent of `dt`. */
  sampleRate: number;
};
