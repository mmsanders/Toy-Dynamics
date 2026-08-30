/**
 * Spatial (6-D) vector algebra, in Featherstone's Plücker convention.
 *
 * This is the arithmetic layer under everything else: no model concepts, no units, no
 * allocation policy beyond "the caller owns the output". Every operation takes an explicit
 * output buffer so the integrator's inner loop can run without touching the allocator.
 *
 * ## Conventions
 *
 * A **motion** vector is `[ω; v]` — angular velocity, then the linear velocity of the
 * body-fixed point currently at the frame origin. A **force** vector is `[n; f]` — moment
 * about the frame origin, then linear force. Both are stored as `Float64Array(6)` with the
 * angular half first.
 *
 * A **transform** from frame A to frame B is stored compactly as `{E, r}` rather than as a
 * 6×6 matrix: `E` is the 3×3 rotation taking A-coordinates to B-coordinates, and `r` is
 * B's origin expressed in A. The 6×6 forms it stands for are
 *
 *     X  = [[E, 0], [-E·r×, E]]        (motion)
 *     X* = [[E, -E·r×], [0, E]]        (force, the inverse transpose of X)
 *
 * and both are applied below without ever materializing them.
 */

export type V3 = Float64Array;
/** Row-major 3×3. */
export type M3 = Float64Array;
/** `[ω; v]` or `[n; f]`. */
export type SV = Float64Array;

export const v3 = (x = 0, y = 0, z = 0): V3 => Float64Array.of(x, y, z);
export const sv = (): SV => new Float64Array(6);
export const m3 = (): M3 => new Float64Array(9);

export function m3Identity(out: M3 = m3()): M3 {
  out.fill(0);
  out[0] = 1;
  out[4] = 1;
  out[8] = 1;
  return out;
}

// ---------------------------------------------------------------------------
// 3-vector and 3×3 primitives
// ---------------------------------------------------------------------------

export function cross(a: V3, b: V3, out: V3): V3 {
  const ax = a[0]!, ay = a[1]!, az = a[2]!;
  const bx = b[0]!, by = b[1]!, bz = b[2]!;
  out[0] = ay * bz - az * by;
  out[1] = az * bx - ax * bz;
  out[2] = ax * by - ay * bx;
  return out;
}

export const dot3 = (a: V3, b: V3): number => a[0]! * b[0]! + a[1]! * b[1]! + a[2]! * b[2]!;

/** out = M · x */
export function matVec(mat: M3, x: V3, out: V3): V3 {
  const x0 = x[0]!, x1 = x[1]!, x2 = x[2]!;
  out[0] = mat[0]! * x0 + mat[1]! * x1 + mat[2]! * x2;
  out[1] = mat[3]! * x0 + mat[4]! * x1 + mat[5]! * x2;
  out[2] = mat[6]! * x0 + mat[7]! * x1 + mat[8]! * x2;
  return out;
}

/** out = Mᵀ · x */
export function matTVec(mat: M3, x: V3, out: V3): V3 {
  const x0 = x[0]!, x1 = x[1]!, x2 = x[2]!;
  out[0] = mat[0]! * x0 + mat[3]! * x1 + mat[6]! * x2;
  out[1] = mat[1]! * x0 + mat[4]! * x1 + mat[7]! * x2;
  out[2] = mat[2]! * x0 + mat[5]! * x1 + mat[8]! * x2;
  return out;
}

/** out = A · B */
export function matMul(a: M3, b: M3, out: M3): M3 {
  for (let i = 0; i < 3; i++) {
    const a0 = a[i * 3]!, a1 = a[i * 3 + 1]!, a2 = a[i * 3 + 2]!;
    out[i * 3] = a0 * b[0]! + a1 * b[3]! + a2 * b[6]!;
    out[i * 3 + 1] = a0 * b[1]! + a1 * b[4]! + a2 * b[7]!;
    out[i * 3 + 2] = a0 * b[2]! + a1 * b[5]! + a2 * b[8]!;
  }
  return out;
}

/** out = A · B · Aᵀ, the similarity transform an inertia tensor undergoes. */
export function matCongruence(a: M3, b: M3, out: M3, scratch: M3): M3 {
  matMul(a, b, scratch);
  // scratch · Aᵀ
  for (let i = 0; i < 3; i++) {
    const s0 = scratch[i * 3]!, s1 = scratch[i * 3 + 1]!, s2 = scratch[i * 3 + 2]!;
    out[i * 3] = s0 * a[0]! + s1 * a[1]! + s2 * a[2]!;
    out[i * 3 + 1] = s0 * a[3]! + s1 * a[4]! + s2 * a[5]!;
    out[i * 3 + 2] = s0 * a[6]! + s1 * a[7]! + s2 * a[8]!;
  }
  return out;
}

/** Rotation matrix from a quaternion `[x, y, z, w]`, mapping body coords to parent coords. */
export function m3FromQuat(q: ArrayLike<number>, out: M3 = m3()): M3 {
  const x = q[0]!, y = q[1]!, z = q[2]!, w = q[3]!;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;

  out[0] = 1 - (yy + zz); out[1] = xy - wz;       out[2] = xz + wy;
  out[3] = xy + wz;       out[4] = 1 - (xx + zz); out[5] = yz - wx;
  out[6] = xz - wy;       out[7] = yz + wx;       out[8] = 1 - (xx + yy);
  return out;
}

/** Rotation about the principal axis `axis` (0=x, 1=y, 2=z) by `angle` radians. */
export function m3FromAxisAngle(axis: 0 | 1 | 2, angle: number, out: M3 = m3()): M3 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  m3Identity(out);
  if (axis === 0) {
    out[4] = c; out[5] = -s; out[7] = s; out[8] = c;
  } else if (axis === 1) {
    out[0] = c; out[2] = s; out[6] = -s; out[8] = c;
  } else {
    out[0] = c; out[1] = -s; out[3] = s; out[4] = c;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Spatial transforms
// ---------------------------------------------------------------------------

/** A Plücker transform: `E` rotates A-coords into B-coords, `r` is B's origin in A. */
export type Transform = { E: M3; r: V3 };

export const transform = (): Transform => ({ E: m3Identity(), r: v3() });

export function transformCopy(src: Transform, out: Transform): Transform {
  out.E.set(src.E);
  out.r.set(src.r);
  return out;
}

/**
 * Compose two transforms: `out` takes A→C given A→B and B→C.
 *
 * `r` accumulates in the *first* frame's coordinates, hence the transpose: `second.r` is
 * expressed in B, and E₁ᵀ carries it back to A.
 */
export function transformCompose(first: Transform, second: Transform, out: Transform, scratch: V3): Transform {
  matTVec(first.E, second.r, scratch);
  out.r[0] = first.r[0]! + scratch[0]!;
  out.r[1] = first.r[1]! + scratch[1]!;
  out.r[2] = first.r[2]! + scratch[2]!;
  matMul(second.E, first.E, out.E);
  return out;
}

/** Apply a motion transform: `out = X · m`, taking a motion vector from A to B. */
export function applyMotion(x: Transform, m: SV, out: SV, scratch: V3): SV {
  const wx = m[0]!, wy = m[1]!, wz = m[2]!;
  // v - r × ω, then rotate the whole thing.
  const rx = x.r[0]!, ry = x.r[1]!, rz = x.r[2]!;
  scratch[0] = m[3]! - (ry * wz - rz * wy);
  scratch[1] = m[4]! - (rz * wx - rx * wz);
  scratch[2] = m[5]! - (rx * wy - ry * wx);

  const e = x.E;
  out[0] = e[0]! * wx + e[1]! * wy + e[2]! * wz;
  out[1] = e[3]! * wx + e[4]! * wy + e[5]! * wz;
  out[2] = e[6]! * wx + e[7]! * wy + e[8]! * wz;
  const s0 = scratch[0]!, s1 = scratch[1]!, s2 = scratch[2]!;
  out[3] = e[0]! * s0 + e[1]! * s1 + e[2]! * s2;
  out[4] = e[3]! * s0 + e[4]! * s1 + e[5]! * s2;
  out[5] = e[6]! * s0 + e[7]! * s1 + e[8]! * s2;
  return out;
}

/** Apply a force transform: `out = X* · f`, taking a force vector from A to B. */
export function applyForce(x: Transform, f: SV, out: SV, scratch: V3): SV {
  const fx = f[3]!, fy = f[4]!, fz = f[5]!;
  const rx = x.r[0]!, ry = x.r[1]!, rz = x.r[2]!;
  scratch[0] = f[0]! - (ry * fz - rz * fy);
  scratch[1] = f[1]! - (rz * fx - rx * fz);
  scratch[2] = f[2]! - (rx * fy - ry * fx);

  const e = x.E;
  const s0 = scratch[0]!, s1 = scratch[1]!, s2 = scratch[2]!;
  out[0] = e[0]! * s0 + e[1]! * s1 + e[2]! * s2;
  out[1] = e[3]! * s0 + e[4]! * s1 + e[5]! * s2;
  out[2] = e[6]! * s0 + e[7]! * s1 + e[8]! * s2;
  out[3] = e[0]! * fx + e[1]! * fy + e[2]! * fz;
  out[4] = e[3]! * fx + e[4]! * fy + e[5]! * fz;
  out[5] = e[6]! * fx + e[7]! * fy + e[8]! * fz;
  return out;
}

/**
 * Apply the *transposed* force transform: `out = Xᵀ · f`, carrying a force from B back
 * to A. This is the direction the inverse-dynamics sweep propagates forces in.
 */
export function applyForceTranspose(x: Transform, f: SV, out: SV, scratch: V3): SV {
  const e = x.E;
  // Eᵀ applied to both halves.
  const n0 = f[0]!, n1 = f[1]!, n2 = f[2]!;
  const f0 = f[3]!, f1 = f[4]!, f2 = f[5]!;
  scratch[0] = e[0]! * n0 + e[3]! * n1 + e[6]! * n2;
  scratch[1] = e[1]! * n0 + e[4]! * n1 + e[7]! * n2;
  scratch[2] = e[2]! * n0 + e[5]! * n1 + e[8]! * n2;
  const lx = e[0]! * f0 + e[3]! * f1 + e[6]! * f2;
  const ly = e[1]! * f0 + e[4]! * f1 + e[7]! * f2;
  const lz = e[2]! * f0 + e[5]! * f1 + e[8]! * f2;

  const rx = x.r[0]!, ry = x.r[1]!, rz = x.r[2]!;
  out[0] = scratch[0]! + (ry * lz - rz * ly);
  out[1] = scratch[1]! + (rz * lx - rx * lz);
  out[2] = scratch[2]! + (rx * ly - ry * lx);
  out[3] = lx;
  out[4] = ly;
  out[5] = lz;
  return out;
}

/** Apply the *inverse* motion transform: `out = X⁻¹ · m`, taking a motion from B to A. */
export function applyMotionInverse(x: Transform, m: SV, out: SV): SV {
  const e = x.E;
  const w0 = m[0]!, w1 = m[1]!, w2 = m[2]!;
  const v0 = m[3]!, v1 = m[4]!, v2 = m[5]!;
  const wx = e[0]! * w0 + e[3]! * w1 + e[6]! * w2;
  const wy = e[1]! * w0 + e[4]! * w1 + e[7]! * w2;
  const wz = e[2]! * w0 + e[5]! * w1 + e[8]! * w2;
  const vx = e[0]! * v0 + e[3]! * v1 + e[6]! * v2;
  const vy = e[1]! * v0 + e[4]! * v1 + e[7]! * v2;
  const vz = e[2]! * v0 + e[5]! * v1 + e[8]! * v2;
  out[0] = wx;
  out[1] = wy;
  out[2] = wz;
  const rx = x.r[0]!, ry = x.r[1]!, rz = x.r[2]!;
  out[3] = vx + (ry * wz - rz * wy);
  out[4] = vy + (rz * wx - rx * wz);
  out[5] = vz + (rx * wy - ry * wx);
  return out;
}

// ---------------------------------------------------------------------------
// Spatial cross products
// ---------------------------------------------------------------------------

/** `out = v × m`, the motion-on-motion cross product. */
export function crossMotion(v: SV, m: SV, out: SV): SV {
  const wx = v[0]!, wy = v[1]!, wz = v[2]!;
  const vx = v[3]!, vy = v[4]!, vz = v[5]!;
  const ax = m[0]!, ay = m[1]!, az = m[2]!;
  const bx = m[3]!, by = m[4]!, bz = m[5]!;
  out[0] = wy * az - wz * ay;
  out[1] = wz * ax - wx * az;
  out[2] = wx * ay - wy * ax;
  out[3] = wy * bz - wz * by + (vy * az - vz * ay);
  out[4] = wz * bx - wx * bz + (vz * ax - vx * az);
  out[5] = wx * by - wy * bx + (vx * ay - vy * ax);
  return out;
}

/** `out = v ×* f`, the motion-on-force cross product. */
export function crossForce(v: SV, f: SV, out: SV): SV {
  const wx = v[0]!, wy = v[1]!, wz = v[2]!;
  const vx = v[3]!, vy = v[4]!, vz = v[5]!;
  const nx = f[0]!, ny = f[1]!, nz = f[2]!;
  const fx = f[3]!, fy = f[4]!, fz = f[5]!;
  out[0] = wy * nz - wz * ny + (vy * fz - vz * fy);
  out[1] = wz * nx - wx * nz + (vz * fx - vx * fz);
  out[2] = wx * ny - wy * nx + (vx * fy - vy * fx);
  out[3] = wy * fz - wz * fy;
  out[4] = wz * fx - wx * fz;
  out[5] = wx * fy - wy * fx;
  return out;
}

// ---------------------------------------------------------------------------
// Spatial rigid-body inertia
// ---------------------------------------------------------------------------

/**
 * Rigid-body inertia about the frame origin: mass, first moment `h = m·c`, and the
 * rotational inertia `I` taken about that same origin.
 *
 * Stored in this compact form rather than as a 6×6 because every operation the solver
 * needs — transform, add, apply — keeps it in the form. That includes the composite
 * inertias built by CRBA: a rigid assembly of rigid bodies is itself a rigid body, so
 * nothing here ever has to widen to a general 6×6.
 *
 * Working with `h` rather than `c` is deliberate: it keeps every formula linear in the
 * mass, so a massless body (a common way to model a pure attachment point) needs no
 * special case and never divides by zero.
 */
export type Inertia = { m: number; h: V3; I: M3 };

export const inertia = (): Inertia => ({ m: 0, h: v3(), I: m3() });

export function inertiaZero(out: Inertia): Inertia {
  out.m = 0;
  out.h.fill(0);
  out.I.fill(0);
  return out;
}

export function inertiaCopy(src: Inertia, out: Inertia): Inertia {
  out.m = src.m;
  out.h.set(src.h);
  out.I.set(src.I);
  return out;
}

export function inertiaAdd(a: Inertia, b: Inertia, out: Inertia): Inertia {
  out.m = a.m + b.m;
  for (let i = 0; i < 3; i++) out.h[i] = a.h[i]! + b.h[i]!;
  for (let i = 0; i < 9; i++) out.I[i] = a.I[i]! + b.I[i]!;
  return out;
}

/** `out = I · v`: the spatial momentum (or force) of a motion vector. */
export function inertiaApply(inr: Inertia, v: SV, out: SV): SV {
  const wx = v[0]!, wy = v[1]!, wz = v[2]!;
  const vx = v[3]!, vy = v[4]!, vz = v[5]!;
  const I = inr.I;
  const hx = inr.h[0]!, hy = inr.h[1]!, hz = inr.h[2]!;

  // n = I·ω + h × v
  out[0] = I[0]! * wx + I[1]! * wy + I[2]! * wz + (hy * vz - hz * vy);
  out[1] = I[3]! * wx + I[4]! * wy + I[5]! * wz + (hz * vx - hx * vz);
  out[2] = I[6]! * wx + I[7]! * wy + I[8]! * wz + (hx * vy - hy * vx);
  // f = m·v + ω × h
  out[3] = inr.m * vx + (wy * hz - wz * hy);
  out[4] = inr.m * vy + (wz * hx - wx * hz);
  out[5] = inr.m * vz + (wx * hy - wy * hx);
  return out;
}

/**
 * Carry an inertia from frame A to frame B through `x`, i.e. `out = X*·I·X⁻¹`.
 *
 * Derived by shifting the reference point from A's origin to B's and then rotating:
 *
 *     h_B = E·(h_A − m·r)
 *     I_B = E·[ I_A + (m|r|² − 2h·r)·1 + h·rᵀ + r·hᵀ − m·r·rᵀ ]·Eᵀ
 *
 * The bracket is the parallel-axis shift written out in terms of `h` instead of `c`, which
 * is what keeps it well defined at zero mass.
 */
export function inertiaTransform(x: Transform, src: Inertia, out: Inertia, scratchA: M3, scratchB: M3): Inertia {
  const m = src.m;
  const rx = x.r[0]!, ry = x.r[1]!, rz = x.r[2]!;
  const hx = src.h[0]!, hy = src.h[1]!, hz = src.h[2]!;

  const rr = rx * rx + ry * ry + rz * rz;
  const hr = hx * rx + hy * ry + hz * rz;
  const diag = m * rr - 2 * hr;

  const t = scratchA;
  t[0] = src.I[0]! + diag + 2 * hx * rx - m * rx * rx;
  t[4] = src.I[4]! + diag + 2 * hy * ry - m * ry * ry;
  t[8] = src.I[8]! + diag + 2 * hz * rz - m * rz * rz;

  const xy = src.I[1]! + hx * ry + rx * hy - m * rx * ry;
  const xz = src.I[2]! + hx * rz + rx * hz - m * rx * rz;
  const yz = src.I[5]! + hy * rz + ry * hz - m * ry * rz;
  t[1] = xy; t[3] = xy;
  t[2] = xz; t[6] = xz;
  t[5] = yz; t[7] = yz;

  matCongruence(x.E, t, out.I, scratchB);

  const shiftedX = hx - m * rx;
  const shiftedY = hy - m * ry;
  const shiftedZ = hz - m * rz;
  const e = x.E;
  out.h[0] = e[0]! * shiftedX + e[1]! * shiftedY + e[2]! * shiftedZ;
  out.h[1] = e[3]! * shiftedX + e[4]! * shiftedY + e[5]! * shiftedZ;
  out.h[2] = e[6]! * shiftedX + e[7]! * shiftedY + e[8]! * shiftedZ;
  out.m = m;
  return out;
}

/**
 * Carry an inertia the other way, from B back to A: `out = Xᵀ·I·X`.
 *
 * This is the direction CRBA accumulates in, folding a child's composite inertia into its
 * parent. Equivalent to `inertiaTransform` through the inverse transform, whose rotation
 * is `Eᵀ` and whose offset is `−E·r`.
 */
export function inertiaTransformInverse(
  x: Transform,
  src: Inertia,
  out: Inertia,
  scratchX: Transform,
  scratchA: M3,
  scratchB: M3,
): Inertia {
  const e = x.E;
  // Eᵀ
  scratchX.E[0] = e[0]!; scratchX.E[1] = e[3]!; scratchX.E[2] = e[6]!;
  scratchX.E[3] = e[1]!; scratchX.E[4] = e[4]!; scratchX.E[5] = e[7]!;
  scratchX.E[6] = e[2]!; scratchX.E[7] = e[5]!; scratchX.E[8] = e[8]!;
  // −E·r
  matVec(e, x.r, scratchX.r);
  scratchX.r[0] = -scratchX.r[0]!;
  scratchX.r[1] = -scratchX.r[1]!;
  scratchX.r[2] = -scratchX.r[2]!;
  return inertiaTransform(scratchX, src, out, scratchA, scratchB);
}
