import type { BodySpec, HingeSpec, ModelSpec } from './model';
import { NEUTRAL_DOF_PARAMS, type DofParams } from './model';

/**
 * Model builders for the tests.
 *
 * Kept beside the solver rather than inside one test file because several suites need the
 * same shapes — a pendulum, a free body, a slider — and a fixture that drifts between
 * copies is a test that quietly stops testing what it claims to.
 */

export const IDENTITY_QUAT = [0, 0, 0, 1] as const;

export const noRotation = (): number[] => [0, 0, 0, 1];

export function bodySpec(overrides: Partial<BodySpec> = {}): BodySpec {
  return {
    name: 'Body',
    mass: 1,
    com: [0, 0, 0],
    inertia: { ixx: 1, iyy: 1, izz: 1, ixy: 0, ixz: 0, iyz: 0 },
    inertiaAbout: 'com',
    ...overrides,
  };
}

export function dofParams(overrides: Partial<DofParams> = {}): DofParams {
  return { ...NEUTRAL_DOF_PARAMS, ...overrides };
}

export function hingeSpec(overrides: Partial<HingeSpec> = {}): HingeSpec {
  return {
    name: 'Hinge',
    parent: -1,
    child: 0,
    parentNodePos: [0, 0, 0],
    parentNodeQuat: noRotation(),
    mount: noRotation(),
    childNodePos: [0, 0, 0],
    childNodeQuat: noRotation(),
    free: [false, false, false, false, false, false],
    values: [0, 0, 0, 0, 0, 0],
    rates: [0, 0, 0, 0, 0, 0],
    ...overrides,
  };
}

/** Free-axis masks, as `[tx, ty, tz, rx, ry, rz]`. */
export const MASK = {
  weld: [false, false, false, false, false, false],
  slideX: [true, false, false, false, false, false],
  hingeY: [false, false, false, false, true, false],
  hingeZ: [false, false, false, false, false, true],
  universalXY: [false, false, false, true, true, false],
  universalXZ: [false, false, false, true, false, true],
  ball: [false, false, false, true, true, true],
  planar: [true, true, false, false, false, true],
  free: [true, true, true, true, true, true],
} as const;

/**
 * A point-mass pendulum: a body hanging `length` below a revolute joint about +y.
 *
 * The mass sits at −z so that zero angle is the hanging equilibrium, which makes the
 * small-angle period the textbook `2π√(L/g)` with no offset to reason about.
 */
export function pendulum(length: number, mass: number, gravity: number, theta0: number): ModelSpec {
  return {
    bodies: [
      bodySpec({
        name: 'Bob',
        mass,
        com: [0, 0, -length],
        // A true point mass: all of the inertia comes from the offset.
        inertia: { ixx: 0, iyy: 0, izz: 0, ixy: 0, ixz: 0, iyz: 0 },
        inertiaAbout: 'com',
      }),
    ],
    hinges: [
      hingeSpec({
        free: [...MASK.hingeY],
        values: [0, 0, 0, 0, theta0, 0],
      }),
    ],
    gravity: [0, 0, -gravity],
  };
}

/** Two point-mass links in series, both revolute about +y. The classic chaotic test case. */
export function doublePendulum(gravity: number): ModelSpec {
  return {
    bodies: [
      bodySpec({
        name: 'Upper',
        mass: 1.3,
        com: [0, 0, -1],
        inertia: { ixx: 0, iyy: 0, izz: 0, ixy: 0, ixz: 0, iyz: 0 },
      }),
      bodySpec({
        name: 'Lower',
        mass: 0.7,
        com: [0, 0, -0.8],
        inertia: { ixx: 0, iyy: 0, izz: 0, ixy: 0, ixz: 0, iyz: 0 },
      }),
    ],
    hinges: [
      hingeSpec({ name: 'Shoulder', child: 0, free: [...MASK.hingeY], values: [0, 0, 0, 0, 1.1, 0] }),
      hingeSpec({
        name: 'Elbow',
        parent: 0,
        child: 1,
        // Hangs off the far end of the upper link.
        parentNodePos: [0, 0, -2],
        free: [...MASK.hingeY],
        values: [0, 0, 0, 0, -0.6, 0],
      }),
    ],
    gravity: [0, 0, -gravity],
  };
}

/**
 * A single body free in all six axes, with distinct principal moments.
 *
 * Distinct moments are the point: they are what make the intermediate-axis instability
 * appear, and what makes the mass matrix a real test rather than a scaled identity.
 */
export function freeBody(
  inertia: { ixx: number; iyy: number; izz: number },
  rates: number[],
  gravity = 0,
): ModelSpec {
  return {
    bodies: [
      bodySpec({
        name: 'Free',
        mass: 2,
        com: [0, 0, 0],
        inertia: { ...inertia, ixy: 0, ixz: 0, iyz: 0 },
        inertiaAbout: 'com',
      }),
    ],
    hinges: [hingeSpec({ free: [...MASK.free], rates })],
    gravity: [0, 0, -gravity],
  };
}

/** A body on a single prismatic axis — the telescoping-pole case. */
export function slider(mass: number, params?: DofParams, x0 = 0, v0 = 0): ModelSpec {
  return {
    bodies: [bodySpec({ name: 'Slider', mass, com: [0, 0, 0] })],
    hinges: [
      hingeSpec({
        free: [...MASK.slideX],
        values: [x0, 0, 0, 0, 0, 0],
        rates: [v0, 0, 0, 0, 0, 0],
        ...(params ? { params: [params, ...Array(5).fill(NEUTRAL_DOF_PARAMS)] } : {}),
      }),
    ],
    gravity: [0, 0, 0],
  };
}
