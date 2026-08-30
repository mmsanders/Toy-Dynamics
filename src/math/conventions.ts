import * as THREE from 'three';
import type { AngleUnit, Conventions, EulerOrder, Quat, RotationMode, UpAxis, Vec3 } from '../types';

/**
 * Convention handling.
 *
 * The scene's stored data is convention-free (see src/types.ts). Everything in this
 * module converts between that canonical storage and whatever the user has asked to
 * *see*. Changing a convention therefore never mutates a scene — it only changes how
 * the same numbers are rendered and interpreted.
 *
 * All rotation math is delegated to THREE.Quaternion / THREE.Euler / THREE.Matrix4.
 */

export const EULER_ORDERS: readonly EulerOrder[] = ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'];

export const AXIS_INDEX = { X: 0, Y: 1, Z: 2 } as const;

export type AxisName = keyof typeof AXIS_INDEX;

// ---------------------------------------------------------------------------
// Up-axis
// ---------------------------------------------------------------------------

/**
 * Rotation that maps engineering coordinates into three.js render coordinates.
 *
 * three.js is natively Y-up, and OrbitControls misbehaves if you re-point `camera.up`.
 * So instead of fighting the renderer we mount the whole scene under a single group
 * carrying this rotation, and leave the camera in native Y-up space.
 *
 * For Z-up this is -90 deg about X, which maps eng (x, y, z) -> three (x, z, -y):
 * eng Z becomes screen-up and handedness is preserved.
 *
 * A pleasant side effect: an unrotated GridHelper (which lies in the three.js XZ plane)
 * lands on the correct ground plane in both modes — eng XY when Z-up, eng XZ when Y-up.
 */
export function mountQuaternion(upAxis: UpAxis): THREE.Quaternion {
  const q = new THREE.Quaternion();
  if (upAxis === 'Z') {
    q.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
  }
  return q;
}

// ---------------------------------------------------------------------------
// Euler order / intrinsic vs extrinsic
// ---------------------------------------------------------------------------

/**
 * The three.js Euler `order` string that realizes a given user-facing sequence.
 *
 * In both modes the user's order string reads in *application* order. three.js Euler
 * orders are intrinsic, so intrinsic sequences pass through unchanged, while an
 * extrinsic (fixed-axis) sequence ABC is the intrinsic sequence CBA. Each angle stays
 * attached to its own axis in either case, because THREE.Euler always stores components
 * by axis name (.x/.y/.z) and uses `order` only to decide composition order.
 *
 * Verified against three.js for all six orders: extrinsic ABC produces exactly the same
 * matrix as three.js order CBA.
 */
export function threeOrderFor(order: EulerOrder, mode: RotationMode): EulerOrder {
  if (mode === 'intrinsic') return order;
  return ([...order].reverse().join('') as EulerOrder);
}

/**
 * The axis that gimbal-locks for a sequence: always the middle one, at +/-90 deg.
 *
 * Reversing a three-character order string leaves the middle character alone, so this
 * is the same axis in both intrinsic and extrinsic mode.
 */
export function gimbalAxisOf(order: EulerOrder): AxisName {
  return order[1] as AxisName;
}

/** Conventional aerospace names, valid only for the intrinsic Z-Y-X sequence. */
const AEROSPACE_ALIAS: Record<AxisName, string> = { X: 'roll', Y: 'pitch', Z: 'yaw' };

export type EulerSlot = {
  /** Which axis this angle rotates about. */
  axis: AxisName;
  /** Index into the per-axis [x, y, z] angle triple. */
  index: 0 | 1 | 2;
  /** 1-based position in the application sequence. */
  step: 1 | 2 | 3;
  /** e.g. "yaw" when the sequence is the aerospace one, otherwise undefined. */
  alias?: string;
};

/**
 * The three Euler slots in application order, so the UI can present them the way the
 * user reads the sequence rather than always as x/y/z.
 */
export function eulerSequence(c: Conventions): EulerSlot[] {
  const aerospace = c.eulerOrder === 'ZYX' && c.rotationMode === 'intrinsic';
  return [...c.eulerOrder].map((ch, i) => {
    const axis = ch as AxisName;
    const slot: EulerSlot = {
      axis,
      index: AXIS_INDEX[axis],
      step: (i + 1) as 1 | 2 | 3,
    };
    if (aerospace) slot.alias = AEROSPACE_ALIAS[axis];
    return slot;
  });
}

/** Human-readable summary, e.g. "intrinsic Z-Y-X (yaw-pitch-roll)". */
export function describeSequence(c: Conventions): string {
  const axes = [...c.eulerOrder].join('-');
  const base = `${c.rotationMode} ${axes}`;
  if (c.eulerOrder === 'ZYX' && c.rotationMode === 'intrinsic') return `${base} (yaw-pitch-roll)`;
  return base;
}

// ---------------------------------------------------------------------------
// Angle units
// ---------------------------------------------------------------------------

export function toDisplayAngle(radians: number, unit: AngleUnit): number {
  return unit === 'deg' ? THREE.MathUtils.radToDeg(radians) : radians;
}

export function fromDisplayAngle(value: number, unit: AngleUnit): number {
  return unit === 'deg' ? THREE.MathUtils.degToRad(value) : value;
}

/** Range of a single Euler slot in the current unit, for slider bounds. */
export function angleRange(unit: AngleUnit): { min: number; max: number; step: number } {
  return unit === 'deg'
    ? { min: -180, max: 180, step: 0.1 }
    : { min: -Math.PI, max: Math.PI, step: 0.001 };
}

// ---------------------------------------------------------------------------
// Quaternion <-> Euler
// ---------------------------------------------------------------------------

/**
 * Euler angles for a quaternion, as a per-axis [x, y, z] triple in the display unit.
 *
 * Note the triple is indexed by axis, not by sequence position — use `eulerSequence`
 * to lay it out in application order.
 */
export function eulerFromQuat(q: Quat, c: Conventions): Vec3 {
  const quat = new THREE.Quaternion(q[0], q[1], q[2], q[3]);
  const e = new THREE.Euler().setFromQuaternion(quat, threeOrderFor(c.eulerOrder, c.rotationMode));
  return [
    toDisplayAngle(e.x, c.angleUnit),
    toDisplayAngle(e.y, c.angleUnit),
    toDisplayAngle(e.z, c.angleUnit),
  ];
}

/** Inverse of `eulerFromQuat`: a per-axis [x, y, z] triple in the display unit. */
export function quatFromEuler(angles: Vec3, c: Conventions): Quat {
  const e = new THREE.Euler(
    fromDisplayAngle(angles[0], c.angleUnit),
    fromDisplayAngle(angles[1], c.angleUnit),
    fromDisplayAngle(angles[2], c.angleUnit),
    threeOrderFor(c.eulerOrder, c.rotationMode),
  );
  return new THREE.Quaternion().setFromEuler(e).toArray() as Quat;
}

/**
 * True when the sequence is within `toleranceDeg` of gimbal lock.
 *
 * At lock the Euler triple stops being unique: the first and third angles trade off
 * freely against each other, so a round-trip through the quaternion can hand back a
 * completely different-looking triple for the very same rotation. The quaternion is
 * unaffected. Worth surfacing rather than hiding.
 */
export function isNearGimbalLock(q: Quat, c: Conventions, toleranceDeg = 1): boolean {
  const angles = eulerFromQuat(q, c);
  const axis = gimbalAxisOf(c.eulerOrder);
  const raw = angles[AXIS_INDEX[axis]];
  const deg = c.angleUnit === 'deg' ? raw : THREE.MathUtils.radToDeg(raw);
  return Math.abs(Math.abs(deg) - 90) <= toleranceDeg;
}

export const DEFAULT_CONVENTIONS: Conventions = {
  upAxis: 'Z',
  eulerOrder: 'ZYX',
  rotationMode: 'intrinsic',
  angleUnit: 'deg',
};
