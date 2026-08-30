import type { Inertia as SpatialInertia, M3, V3 } from './spatial';
import { inertia as makeInertia, m3 } from './spatial';

/**
 * Building a solver inertia from user-entered mass properties.
 *
 * This is where the CoM / body-origin toggle is resolved. Both references are common on
 * real data sheets and the difference is a whole `m·d²` term, so the model states which
 * one it means and the conversion happens exactly once, here, when the solver model is
 * built.
 */

/** Products of inertia in the `+∫xy` convention, as stored on the model. */
export type InertiaInput = {
  ixx: number;
  iyy: number;
  izz: number;
  ixy: number;
  ixz: number;
  iyz: number;
};

/**
 * The 3×3 tensor for a set of moments and products.
 *
 * The off-diagonals are negated because the model stores products of inertia as `∫xy dm`,
 * while the tensor that appears in `H = I·ω` carries them as `−∫xy dm`. Both sign
 * conventions are in wide use, which is exactly why the UI names the one it wants.
 */
export function tensorOf(input: InertiaInput, out: M3 = m3()): M3 {
  out[0] = input.ixx;  out[1] = -input.ixy; out[2] = -input.ixz;
  out[3] = -input.ixy; out[4] = input.iyy;  out[5] = -input.iyz;
  out[6] = -input.ixz; out[7] = -input.iyz; out[8] = input.izz;
  return out;
}

/**
 * The parallel-axis shift `m·(|c|²·1 − c·cᵀ)`, added to a CoM inertia to move it to an
 * origin offset by `−c`.
 */
export function parallelAxisShift(mass: number, c: V3, out: M3 = m3()): M3 {
  const cx = c[0]!, cy = c[1]!, cz = c[2]!;
  const cc = cx * cx + cy * cy + cz * cz;
  out[0] = mass * (cc - cx * cx);
  out[4] = mass * (cc - cy * cy);
  out[8] = mass * (cc - cz * cz);
  const xy = -mass * cx * cy;
  const xz = -mass * cx * cz;
  const yz = -mass * cy * cz;
  out[1] = xy; out[3] = xy;
  out[2] = xz; out[6] = xz;
  out[5] = yz; out[7] = yz;
  return out;
}

/**
 * Assemble the spatial inertia about the body origin.
 *
 * `com` is the centre of mass in body coordinates. When the tensor was given about the
 * CoM it is shifted out to the origin; when it was already about the origin it is used
 * as-is. Either way the result is the canonical about-origin form the solver works in.
 */
export function buildInertia(
  mass: number,
  com: V3,
  input: InertiaInput,
  about: 'com' | 'origin',
  out: SpatialInertia = makeInertia(),
): SpatialInertia {
  tensorOf(input, out.I);
  if (about === 'com') {
    const shift = parallelAxisShift(mass, com);
    for (let i = 0; i < 9; i++) out.I[i] = out.I[i]! + shift[i]!;
  }
  out.m = mass;
  out.h[0] = mass * com[0]!;
  out.h[1] = mass * com[1]!;
  out.h[2] = mass * com[2]!;
  return out;
}

/**
 * Re-express a tensor taken about one point as one taken about another.
 *
 * Both arguments are the centre of mass measured *from* the respective reference point.
 * Used when the user designates a different node as the body origin: the physical body has
 * not changed, so a tensor stated about the origin has to be moved to the new one rather
 * than silently coming to mean something else.
 */
export function moveInertiaReference(
  mass: number,
  input: InertiaInput,
  comFromOld: V3,
  comFromNew: V3,
): InertiaInput {
  const removeOld = parallelAxisShift(mass, comFromOld);
  const addNew = parallelAxisShift(mass, comFromNew);
  const d = (i: number): number => addNew[i]! - removeOld[i]!;
  return {
    ixx: input.ixx + d(0),
    iyy: input.iyy + d(4),
    izz: input.izz + d(8),
    // The shift is a tensor (−∫xy convention) while the stored products are +∫xy, so the
    // off-diagonal deltas come back with the opposite sign.
    ixy: input.ixy - d(1),
    ixz: input.ixz - d(2),
    iyz: input.iyz - d(5),
  };
}

// ---------------------------------------------------------------------------
// Physicality
// ---------------------------------------------------------------------------

/** Eigenvalues of a symmetric 3×3, ascending. Closed form — no iteration, no allocation. */
export function symmetricEigenvalues(t: M3): [number, number, number] {
  const a = t[0]!, b = t[4]!, c = t[8]!;
  const d = t[1]!, e = t[5]!, f = t[2]!;

  const p1 = d * d + e * e + f * f;
  if (p1 === 0) {
    const vals = [a, b, c].sort((x, y) => x - y);
    return [vals[0]!, vals[1]!, vals[2]!];
  }

  const q = (a + b + c) / 3;
  const p2 = (a - q) ** 2 + (b - q) ** 2 + (c - q) ** 2 + 2 * p1;
  const p = Math.sqrt(p2 / 6);

  // det((T − qI)/p) / 2, clamped because rounding can push it just outside [−1, 1].
  const m00 = (a - q) / p, m11 = (b - q) / p, m22 = (c - q) / p;
  const m01 = d / p, m12 = e / p, m02 = f / p;
  const det =
    m00 * (m11 * m22 - m12 * m12) - m01 * (m01 * m22 - m12 * m02) + m02 * (m01 * m12 - m11 * m02);
  const r = Math.min(1, Math.max(-1, det / 2));

  const phi = Math.acos(r) / 3;
  const eig1 = q + 2 * p * Math.cos(phi);
  const eig3 = q + 2 * p * Math.cos(phi + (2 * Math.PI) / 3);
  const eig2 = 3 * q - eig1 - eig3;
  const sorted = [eig1, eig2, eig3].sort((x, y) => x - y);
  return [sorted[0]!, sorted[1]!, sorted[2]!];
}

export type InertiaCheck = {
  /** Principal moments, ascending. */
  principal: [number, number, number];
  /** False when a principal moment is zero or negative — no real body has that. */
  positiveDefinite: boolean;
  /**
   * The triangle inequality `I₁ + I₂ ≥ I₃` on the principal moments.
   *
   * A real mass distribution always satisfies it. A violation means the numbers cannot
   * describe any physical body, which almost always means a typo or a wrong sign
   * convention on the products of inertia.
   */
  triangleInequality: boolean;
  /** `√(I/m)` from the mean principal moment: the body's effective size. */
  radiusOfGyration: number;
};

export function checkInertia(mass: number, tensor: M3): InertiaCheck {
  const principal = symmetricEigenvalues(tensor);
  const [i1, i2, i3] = principal;
  const scale = Math.max(Math.abs(i1), Math.abs(i2), Math.abs(i3), 1e-300);
  const mean = (i1 + i2 + i3) / 3;
  return {
    principal,
    positiveDefinite: i1 > 0,
    // Scaled tolerance: an exact rod or disc sits right on the boundary, and floating
    // point should not turn that into a warning.
    triangleInequality: i1 + i2 >= i3 - 1e-9 * scale,
    radiusOfGyration: mass > 0 && mean > 0 ? Math.sqrt(mean / mass) : 0,
  };
}
