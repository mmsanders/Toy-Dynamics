import type { MultibodyModel } from './model';
import {
  type Inertia,
  type M3,
  type SV,
  type Transform,
  type V3,
  applyForceTranspose,
  inertia as makeInertia,
  inertiaAdd,
  inertiaApply,
  inertiaCopy,
  inertiaTransformInverse,
  m3,
  sv,
  transform,
  v3,
} from './spatial';

/**
 * The joint-space mass matrix, by the composite-rigid-body algorithm, and the linear solve
 * that turns it into accelerations.
 *
 * CRBA is O(n²) where the articulated-body algorithm is O(n), but it is chosen deliberately
 * here: it produces **H itself**, and H is what the diagnostics are built on. A failed
 * factorization is precisely the "this model is degenerate" detector, and the spread of the
 * pivots is the "nearly degenerate" warning. At the tens of DOF this tool is for, the two
 * algorithms cost about the same anyway.
 *
 * The composite inertias stay in the compact `{m, h, I}` rigid-body form throughout, which
 * is legitimate because a rigidly-connected set of rigid bodies is itself a rigid body.
 */

export type CrbaScratch = {
  Ic: Inertia[];
  /** 6 × 6 column-major workspace for `Ic·S`, sized for the widest possible joint. */
  F: Float64Array;
  Fnext: Float64Array;
  col: SV;
  tmp: SV;
  inertiaTmp: Inertia;
  transformTmp: Transform;
  a: M3;
  b: M3;
  vec: V3;
};

export function makeCrbaScratch(model: MultibodyModel): CrbaScratch {
  return {
    Ic: model.links.map(() => makeInertia()),
    F: new Float64Array(36),
    Fnext: new Float64Array(36),
    col: sv(),
    tmp: sv(),
    inertiaTmp: makeInertia(),
    transformTmp: transform(),
    a: m3(),
    b: m3(),
    vec: v3(),
  };
}

/**
 * Fill `H` (nv × nv, row-major) with the mass matrix.
 *
 * Expects `updateKinematics` to have run: it reads each joint's transform and motion
 * subspace from the workspaces.
 */
export function crba(model: MultibodyModel, H: Float64Array, s: CrbaScratch): void {
  const links = model.links;
  const nv = model.nv;
  H.fill(0);

  for (let i = 0; i < links.length; i++) inertiaCopy(links[i]!.I, s.Ic[i]!);

  for (let i = links.length - 1; i >= 0; i--) {
    const link = links[i]!;
    const Ic = s.Ic[i]!;

    if (link.parent >= 0) {
      inertiaTransformInverse(link.X, Ic, s.inertiaTmp, s.transformTmp, s.a, s.b);
      inertiaAdd(s.Ic[link.parent]!, s.inertiaTmp, s.Ic[link.parent]!);
    }

    const ni = link.joint.nv;
    if (ni === 0) continue;
    const Si = link.jw.S;
    const oi = link.joint.vOffset;

    // F = Ic · S, one spatial force per joint axis.
    for (let c = 0; c < ni; c++) {
      const base = 6 * c;
      for (let r = 0; r < 6; r++) s.col[r] = Si[base + r]!;
      inertiaApply(Ic, s.col, s.tmp);
      for (let r = 0; r < 6; r++) s.F[base + r] = s.tmp[r]!;
    }

    // Diagonal block: Sᵀ·F.
    for (let a = 0; a < ni; a++) {
      const aBase = 6 * a;
      for (let c = 0; c < ni; c++) {
        const cBase = 6 * c;
        let acc = 0;
        for (let r = 0; r < 6; r++) acc += Si[aBase + r]! * s.F[cBase + r]!;
        H[(oi + a) * nv + oi + c] = acc;
      }
    }

    // Walk up the ancestors, carrying F into each frame and pairing it with that joint's
    // subspace. This is what fills the off-diagonal coupling between a joint and every
    // joint above it.
    let j = i;
    while (links[j]!.parent >= 0) {
      const jLink = links[j]!;
      for (let c = 0; c < ni; c++) {
        const base = 6 * c;
        for (let r = 0; r < 6; r++) s.col[r] = s.F[base + r]!;
        applyForceTranspose(jLink.X, s.col, s.tmp, s.vec);
        for (let r = 0; r < 6; r++) s.Fnext[base + r] = s.tmp[r]!;
      }
      s.F.set(s.Fnext.subarray(0, 6 * ni));

      j = jLink.parent;
      const up = links[j]!;
      const nj = up.joint.nv;
      const oj = up.joint.vOffset;
      const Sj = up.jw.S;
      for (let a = 0; a < nj; a++) {
        const aBase = 6 * a;
        for (let c = 0; c < ni; c++) {
          const cBase = 6 * c;
          let acc = 0;
          for (let r = 0; r < 6; r++) acc += Sj[aBase + r]! * s.F[cBase + r]!;
          H[(oj + a) * nv + oi + c] = acc;
          H[(oi + c) * nv + oj + a] = acc;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Symmetric solve
// ---------------------------------------------------------------------------

export type Factorization = {
  /** Unit lower-triangular factor, row-major, `n × n`. */
  L: Float64Array;
  /** Diagonal of D. */
  D: Float64Array;
  n: number;
  /** Index of the first non-positive pivot, or −1 when the factorization succeeded. */
  failedAt: number;
  /**
   * `max|D| / min|D|`.
   *
   * A cheap stand-in for the condition number, not the thing itself — it ignores the
   * off-diagonal factor entirely. It is used only to decide when to *warn*, which is a job
   * an order-of-magnitude signal does perfectly well.
   */
  pivotSpread: number;
};

export function makeFactorization(n: number): Factorization {
  return { L: new Float64Array(n * n), D: new Float64Array(n), n, failedAt: -1, pivotSpread: 1 };
}

/**
 * LDLᵀ factorization of a symmetric matrix.
 *
 * LDLᵀ rather than Cholesky because it needs no square roots and, more usefully here, it
 * exposes the pivots directly: a non-positive pivot names the exact coordinate at which the
 * model stopped being solvable, which is a far better diagnostic than "Cholesky failed".
 */
export function factorize(H: Float64Array, f: Factorization): Factorization {
  const n = f.n;
  const L = f.L;
  const D = f.D;
  L.fill(0);
  f.failedAt = -1;

  // A scale to judge pivots against, so the test means the same thing for a model in
  // grams and one in tonnes.
  let scale = 0;
  for (let i = 0; i < n; i++) scale = Math.max(scale, Math.abs(H[i * n + i]!));
  const tol = Math.max(scale, 1) * 1e-12;

  for (let j = 0; j < n; j++) {
    let d = H[j * n + j]!;
    for (let k = 0; k < j; k++) d -= L[j * n + k]! * L[j * n + k]! * D[k]!;
    D[j] = d;
    if (!(d > tol)) {
      if (f.failedAt < 0) f.failedAt = j;
      // Keep going with a floor so the caller still gets a usable answer to inspect
      // rather than a cascade of NaNs.
      D[j] = tol;
    }
    L[j * n + j] = 1;
    for (let i = j + 1; i < n; i++) {
      let acc = H[i * n + j]!;
      for (let k = 0; k < j; k++) acc -= L[i * n + k]! * L[j * n + k]! * D[k]!;
      L[i * n + j] = acc / D[j]!;
    }
  }

  let lo = Infinity;
  let hi = 0;
  for (let i = 0; i < n; i++) {
    const v = Math.abs(D[i]!);
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  f.pivotSpread = n === 0 ? 1 : lo > 0 ? hi / lo : Infinity;
  return f;
}

/** Solve `H·x = b` in place on `x`, using a factorization from `factorize`. */
export function solveFactorized(f: Factorization, b: Float64Array, x: Float64Array): void {
  const n = f.n;
  const L = f.L;
  for (let i = 0; i < n; i++) {
    let acc = b[i]!;
    for (let k = 0; k < i; k++) acc -= L[i * n + k]! * x[k]!;
    x[i] = acc;
  }
  for (let i = 0; i < n; i++) x[i] = x[i]! / f.D[i]!;
  for (let i = n - 1; i >= 0; i--) {
    let acc = x[i]!;
    for (let k = i + 1; k < n; k++) acc -= L[k * n + i]! * x[k]!;
    x[i] = acc;
  }
}
