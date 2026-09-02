import type { UnitSystem, Vec3 } from './types';

/**
 * Unit systems.
 *
 * The solver is pure arithmetic: it never converts anything, and keeping the numbers
 * mutually consistent is the user's job. Selecting a system changes exactly three things —
 * the labels on the fields, the gravity preset, and which plausibility checks are armed.
 * It never rewrites a stored value.
 *
 * That last guarantee is the same one Rotation Wizard makes about rotation conventions,
 * and for the same reason: a control that silently edits your data while claiming to
 * change how it is displayed is a control you stop trusting.
 */

export type Quantity =
  | 'mass'
  | 'length'
  | 'force'
  | 'moment'
  | 'inertia'
  | 'time'
  | 'velocity'
  | 'angularVelocity'
  | 'acceleration'
  | 'angularAcceleration'
  | 'linearStiffness'
  | 'angularStiffness'
  | 'linearDamping'
  | 'angularDamping'
  | 'energy';

export type UnitSystemInfo = {
  id: UnitSystem;
  name: string;
  /** One line for the Setup panel, saying what the system commits you to. */
  note: string;
  labels: Record<Quantity, string>;
  /**
   * Standard gravitational acceleration in this system, or null when there is no such
   * thing — which is the case for Generic, where a length unit has not been chosen.
   */
  standardGravity: number | null;
};

const SI_LABELS: Record<Quantity, string> = {
  mass: 'kg',
  length: 'm',
  force: 'N',
  moment: 'N·m',
  inertia: 'kg·m²',
  time: 's',
  velocity: 'm/s',
  angularVelocity: 'rad/s',
  acceleration: 'm/s²',
  angularAcceleration: 'rad/s²',
  linearStiffness: 'N/m',
  angularStiffness: 'N·m/rad',
  linearDamping: 'N·s/m',
  angularDamping: 'N·m·s/rad',
  energy: 'J',
};

const IMPERIAL_LABELS: Record<Quantity, string> = {
  mass: 'slug',
  length: 'ft',
  force: 'lbf',
  moment: 'ft·lbf',
  inertia: 'slug·ft²',
  time: 's',
  velocity: 'ft/s',
  angularVelocity: 'rad/s',
  acceleration: 'ft/s²',
  angularAcceleration: 'rad/s²',
  linearStiffness: 'lbf/ft',
  angularStiffness: 'ft·lbf/rad',
  linearDamping: 'lbf·s/ft',
  angularDamping: 'ft·lbf·s/rad',
  energy: 'ft·lbf',
};

/**
 * Dimension symbols rather than units.
 *
 * More useful than blank labels: they still say which slot a number goes in, and they make
 * the coherence requirement visible — force really does have to equal M·L/T².
 */
const GENERIC_LABELS: Record<Quantity, string> = {
  mass: '[M]',
  length: '[L]',
  force: '[F]',
  moment: '[F·L]',
  inertia: '[M·L²]',
  time: '[T]',
  velocity: '[L/T]',
  angularVelocity: '[rad/T]',
  acceleration: '[L/T²]',
  angularAcceleration: '[rad/T²]',
  linearStiffness: '[F/L]',
  angularStiffness: '[F·L/rad]',
  linearDamping: '[F·T/L]',
  angularDamping: '[F·L·T/rad]',
  energy: '[F·L]',
};

export const STANDARD_GRAVITY_SI = 9.80665;
/** The same acceleration in feet: 9.80665 / 0.3048. */
export const STANDARD_GRAVITY_IMPERIAL = 32.17404855643045;

/** 1 slug = 32.174… lbm. The single most common Imperial modelling mistake. */
export const LBM_PER_SLUG = STANDARD_GRAVITY_IMPERIAL;

export const UNIT_SYSTEMS: Record<UnitSystem, UnitSystemInfo> = {
  si: {
    id: 'si',
    name: 'SI',
    note: 'kilograms, metres, newtons, seconds.',
    labels: SI_LABELS,
    standardGravity: STANDARD_GRAVITY_SI,
  },
  imperial: {
    id: 'imperial',
    name: 'Imperial',
    // Stated up front because getting this wrong is a silent factor-of-32 error, and no
    // amount of downstream checking recovers a model built on pounds-mass.
    note: 'slugs, feet, pounds-force, seconds. Mass is in slugs, not pounds — 1 slug = 32.174 lbm.',
    labels: IMPERIAL_LABELS,
    standardGravity: STANDARD_GRAVITY_IMPERIAL,
  },
  generic: {
    id: 'generic',
    name: 'Generic',
    note: 'No units assumed. Keep them consistent — force must equal mass × length / time².',
    labels: GENERIC_LABELS,
    standardGravity: null,
  },
};

export const unitLabel = (system: UnitSystem, quantity: Quantity): string =>
  UNIT_SYSTEMS[system].labels[quantity];

/** Convert a mass entered in pounds-mass into slugs, for the Imperial helper button. */
export const lbmToSlug = (lbm: number): number => lbm / LBM_PER_SLUG;

/**
 * Gravity to use after switching unit systems.
 *
 * The rule follows from "switching never rewrites a value": the preset is applied only
 * when the current gravity is still exactly the *previous* system's standard, i.e. the user
 * has not touched it. Any customised value survives the switch untouched, and the UI offers
 * the preset as a button instead.
 *
 * Returns null when the current gravity should be left alone.
 */
export function gravityOnSystemChange(
  current: Vec3,
  from: UnitSystem,
  to: UnitSystem,
): Vec3 | null {
  const fromStandard = UNIT_SYSTEMS[from].standardGravity;
  const toStandard = UNIT_SYSTEMS[to].standardGravity;

  const magnitude = Math.hypot(current[0], current[1], current[2]);
  const untouched =
    fromStandard === null
      ? magnitude === 0
      : Math.abs(magnitude - fromStandard) < 1e-6 * fromStandard;
  if (!untouched) return null;

  if (toStandard === null) return [0, 0, 0];
  if (magnitude === 0) {
    // Coming from Generic's zero gravity there is no direction to preserve, so use the
    // conventional one: down the negative third axis.
    return [0, 0, -toStandard];
  }
  const scale = toStandard / magnitude;
  return [current[0] * scale, current[1] * scale, current[2] * scale];
}

/** The standard gravity vector for a system, pointing down the negative third axis. */
export function standardGravityVector(system: UnitSystem): Vec3 {
  const g = UNIT_SYSTEMS[system].standardGravity;
  return g === null ? [0, 0, 0] : [0, 0, -g];
}
