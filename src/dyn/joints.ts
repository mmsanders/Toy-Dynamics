import type { M3, SV, Transform, V3 } from './spatial';
import { cross, m3, m3FromAxisAngle, m3FromQuat, m3Identity, matMul, sv, transform, v3 } from './spatial';

/**
 * The general masked 6-DOF hinge.
 *
 * Every joint in the model is this one thing: six axes — three translational, three
 * rotational — each independently free or locked. A locked axis is not a constraint the
 * solver has to satisfy; it simply is not a coordinate. That is the whole reason for
 * working in reduced coordinates: locks are exact and free, constraint drift cannot
 * happen, and a tree can never be over-constrained.
 *
 * A 1-free-rotation joint is a revolute, one free translation is a prismatic (the
 * telescoping pole), all six free is a floating body, none free is a weld.
 *
 * ## Convention
 *
 * The joint transform from the parent-side joint frame P to the child frame C is
 * **translate, then rotate**:
 *
 *     p_P = d + R · p_C
 *
 * so `d` is the child origin expressed in P, along P's own axes, and `R` maps C-coordinates
 * into P-coordinates. Translating first is what makes a prismatic axis mean the obvious
 * thing — it stays fixed in the parent while the child rotates on the end of it.
 *
 * Rotations compose as an intrinsic X→Y→Z sequence: `R = Rx(θx)·Ry(θy)·Rz(θz)`. Locked
 * angles still contribute their fixed rotation.
 *
 * ## Why three rotational DOF are parametrized differently
 *
 * With all three rotations free the Euler triple would gimbal-lock, so that case switches
 * to a quaternion: the joint carries four position variables for three speeds, and the
 * motion subspace becomes exactly the identity in the child frame. Exact, singularity-free,
 * and it makes a fully-free hinge a proper free-flyer.
 *
 * With one or two free rotations the Euler parametrization is kept, because it is what
 * makes a hinge angle *be* the number the user typed. Only one masking can degenerate:
 * free `rx` with free `rz` and `ry` locked at ±90° puts both axes on top of each other.
 * That is a static property of the mask, so it is caught when the model is built rather
 * than being left to blow up mid-run.
 */

export type JointModel = {
  /** Free translational axis indices (0=x, 1=y, 2=z), ascending. */
  freeTrans: number[];
  /** Free rotational axis indices, ascending. */
  freeRot: number[];
  /** All three translation values; entries for free axes are overwritten from `q`. */
  lockedTrans: V3;
  /** All three rotation angles in radians; entries for free axes come from `q`. */
  lockedRot: V3;
  /** True when all three rotations are free, switching to the quaternion parametrization. */
  useQuaternion: boolean;
  /** Position variables: one per free translation, plus 4 (quaternion) or one per free rotation. */
  nq: number;
  /** Velocity variables: one per free axis, full stop. */
  nv: number;
  /** Where this joint's variables start in the global q / v vectors. */
  qOffset: number;
  vOffset: number;
};

export function makeJointModel(
  free: readonly boolean[],
  locked: readonly number[],
  qOffset: number,
  vOffset: number,
): JointModel {
  const freeTrans: number[] = [];
  const freeRot: number[] = [];
  for (let i = 0; i < 3; i++) if (free[i]) freeTrans.push(i);
  for (let i = 0; i < 3; i++) if (free[i + 3]) freeRot.push(i);

  const useQuaternion = freeRot.length === 3;
  const nv = freeTrans.length + freeRot.length;
  const nq = freeTrans.length + (useQuaternion ? 4 : freeRot.length);

  return {
    freeTrans,
    freeRot,
    lockedTrans: v3(locked[0] ?? 0, locked[1] ?? 0, locked[2] ?? 0),
    lockedRot: v3(locked[3] ?? 0, locked[4] ?? 0, locked[5] ?? 0),
    useQuaternion,
    nq,
    nv,
    qOffset,
    vOffset,
  };
}

/**
 * Per-joint scratch, allocated once when the model is built.
 *
 * The integrator calls `jcalc` several times per step — four times per step under RK4 —
 * so nothing in here may allocate.
 */
export type JointWorkspace = {
  XJ: Transform;
  /** 6 × nv, column-major: column j occupies `S[6j .. 6j+5]`. */
  S: Float64Array;
  vJ: SV;
  cJ: SV;
  /** R mapping child coordinates into parent-joint coordinates. */
  R: M3;
  d: V3;
  /** The three rotational subspace axes, in child coordinates. */
  rotCols: [V3, V3, V3];
  /** Free-axis rates, indexed by axis. Zero for locked axes. */
  rotRates: V3;
  scratchA: M3;
  scratchB: M3;
  scratchV: V3;
};

export function makeJointWorkspace(joint: JointModel): JointWorkspace {
  return {
    XJ: transform(),
    S: new Float64Array(6 * Math.max(joint.nv, 1)),
    vJ: sv(),
    cJ: sv(),
    R: m3Identity(),
    d: v3(),
    rotCols: [v3(), v3(), v3()],
    rotRates: v3(),
    scratchA: m3(),
    scratchB: m3(),
    scratchV: v3(),
  };
}

/**
 * Joint kinematics at a configuration: the transform, the motion subspace, the joint
 * velocity, and the bias acceleration.
 *
 * `cJ` is the *apparent* derivative `Ṡ·q̇` — the rate of change of S's components as seen
 * in the child frame, with the frame's own motion excluded. The recursion adds the
 * `v × vJ` term separately, so including frame motion here would double-count it.
 */
export function jcalc(
  joint: JointModel,
  q: Float64Array,
  v: Float64Array,
  w: JointWorkspace,
): void {
  const { freeTrans, freeRot, useQuaternion, qOffset, vOffset } = joint;

  // --- configuration --------------------------------------------------------------
  w.d.set(joint.lockedTrans);
  for (let i = 0; i < freeTrans.length; i++) w.d[freeTrans[i]!] = q[qOffset + i]!;

  const rotQ = qOffset + freeTrans.length;
  if (useQuaternion) {
    m3FromQuat(q.subarray(rotQ, rotQ + 4), w.R);
  } else {
    const angles = w.scratchV;
    angles.set(joint.lockedRot);
    for (let i = 0; i < freeRot.length; i++) angles[freeRot[i]!] = q[rotQ + i]!;
    // R = Rx·Ry·Rz
    m3FromAxisAngle(0, angles[0]!, w.scratchA);
    m3FromAxisAngle(1, angles[1]!, w.scratchB);
    matMul(w.scratchA, w.scratchB, w.R);
    m3FromAxisAngle(2, angles[2]!, w.scratchA);
    matMul(w.R, w.scratchA, w.scratchB);
    w.R.set(w.scratchB);
  }

  // XJ takes P → C, so its rotation is Rᵀ and its offset is the child origin in P.
  const R = w.R;
  const E = w.XJ.E;
  E[0] = R[0]!; E[1] = R[3]!; E[2] = R[6]!;
  E[3] = R[1]!; E[4] = R[4]!; E[5] = R[7]!;
  E[6] = R[2]!; E[7] = R[5]!; E[8] = R[8]!;
  w.XJ.r.set(w.d);

  // --- rotational subspace axes, in child coordinates -----------------------------
  //
  // For the intrinsic X→Y→Z sequence each axis is pushed through the rotations that come
  // *after* it, which is what leaves the last axis untouched and the first fully carried:
  //
  //     col_x = Rz(θz)ᵀ·Ry(θy)ᵀ·e_x     col_y = Rz(θz)ᵀ·e_y     col_z = e_z
  //
  // Under the quaternion parametrization the subspace is simply the identity.
  const [colX, colY, colZ] = w.rotCols;
  if (useQuaternion) {
    colX[0] = 1; colX[1] = 0; colX[2] = 0;
    colY[0] = 0; colY[1] = 1; colY[2] = 0;
    colZ[0] = 0; colZ[1] = 0; colZ[2] = 1;
  } else {
    const angles = w.scratchV;
    const cy = Math.cos(angles[1]!), sy = Math.sin(angles[1]!);
    const cz = Math.cos(angles[2]!), sz = Math.sin(angles[2]!);
    colX[0] = cz * cy; colX[1] = -sz * cy; colX[2] = sy;
    colY[0] = sz;      colY[1] = cz;       colY[2] = 0;
    colZ[0] = 0;       colZ[1] = 0;        colZ[2] = 1;
  }

  // --- motion subspace ------------------------------------------------------------
  const S = w.S;
  S.fill(0);
  // A translational axis contributes pure linear motion along that axis of P, which in
  // child coordinates is Rᵀ·e_k — the k'th row of R.
  for (let i = 0; i < freeTrans.length; i++) {
    const k = freeTrans[i]!;
    const base = 6 * i;
    S[base + 3] = R[3 * k]!;
    S[base + 4] = R[3 * k + 1]!;
    S[base + 5] = R[3 * k + 2]!;
  }
  const rotColumnStart = freeTrans.length;
  for (let i = 0; i < freeRot.length; i++) {
    const col = w.rotCols[freeRot[i]!]!;
    const base = 6 * (rotColumnStart + i);
    S[base] = col[0]!;
    S[base + 1] = col[1]!;
    S[base + 2] = col[2]!;
  }

  // --- joint velocity: vJ = S·q̇ ---------------------------------------------------
  w.vJ.fill(0);
  w.rotRates.fill(0);
  for (let i = 0; i < joint.nv; i++) {
    const rate = v[vOffset + i]!;
    if (rate === 0) continue;
    const base = 6 * i;
    for (let r = 0; r < 6; r++) w.vJ[r] = w.vJ[r]! + rate * S[base + r]!;
  }
  for (let i = 0; i < freeRot.length; i++) {
    w.rotRates[freeRot[i]!] = v[vOffset + rotColumnStart + i]!;
  }

  // --- bias acceleration: cJ = Ṡ·q̇ ------------------------------------------------
  //
  // Differentiating the column formulas above gives the compact
  //
  //     d(col_k)/dt = −Σ_{j>k} θ̇_j · (col_j × col_k)
  //
  // so the angular half is a sum over ordered pairs of free rotational axes, and vanishes
  // for fewer than two of them (and under the quaternion parametrization, where S is
  // constant). The linear half comes from the translation axes being dragged around by the
  // joint's own rotation: d(Rᵀ·e_k)/dt = −ω × (Rᵀ·e_k).
  w.cJ.fill(0);
  if (!useQuaternion && freeRot.length > 1) {
    const tmp = w.scratchV;
    for (let a = 0; a < freeRot.length; a++) {
      const k = freeRot[a]!;
      for (let b = a + 1; b < freeRot.length; b++) {
        const j = freeRot[b]!;
        const scale = w.rotRates[k]! * w.rotRates[j]!;
        if (scale === 0) continue;
        cross(w.rotCols[j]!, w.rotCols[k]!, tmp);
        w.cJ[0] = w.cJ[0]! - scale * tmp[0]!;
        w.cJ[1] = w.cJ[1]! - scale * tmp[1]!;
        w.cJ[2] = w.cJ[2]! - scale * tmp[2]!;
      }
    }
  }
  if (freeTrans.length > 0) {
    // −ω × v, with both halves read straight off vJ.
    const wx = w.vJ[0]!, wy = w.vJ[1]!, wz = w.vJ[2]!;
    const vx = w.vJ[3]!, vy = w.vJ[4]!, vz = w.vJ[5]!;
    w.cJ[3] = w.cJ[3]! - (wy * vz - wz * vy);
    w.cJ[4] = w.cJ[4]! - (wz * vx - wx * vz);
    w.cJ[5] = w.cJ[5]! - (wx * vy - wy * vx);
  }
}

// ---------------------------------------------------------------------------
// Position state
// ---------------------------------------------------------------------------

/**
 * Write a joint's initial position variables.
 *
 * `values` holds all six axis values as the user entered them; only the free ones become
 * coordinates. Under the quaternion parametrization the three initial Euler angles are
 * converted once, here, so the user still types angles even though the state is a
 * quaternion.
 */
export function writeInitialQ(joint: JointModel, values: readonly number[], q: Float64Array): void {
  let at = joint.qOffset;
  for (const k of joint.freeTrans) q[at++] = values[k] ?? 0;

  if (joint.useQuaternion) {
    const [x, y, z] = [values[3] ?? 0, values[4] ?? 0, values[5] ?? 0];
    const cx = Math.cos(x / 2), sx = Math.sin(x / 2);
    const cy = Math.cos(y / 2), sy = Math.sin(y / 2);
    const cz = Math.cos(z / 2), sz = Math.sin(z / 2);
    // Intrinsic X→Y→Z, matching the rotation built in jcalc.
    q[at] = sx * cy * cz + cx * sy * sz;
    q[at + 1] = cx * sy * cz - sx * cy * sz;
    q[at + 2] = cx * cy * sz + sx * sy * cz;
    q[at + 3] = cx * cy * cz - sx * sy * sz;
  } else {
    for (const k of joint.freeRot) q[at++] = values[k + 3] ?? 0;
  }
}

/**
 * The time derivative of the position variables, `dq/dt`, given the speeds.
 *
 * For every parametrization but the quaternion this is the identity — the coordinate is
 * the angle or the offset, and its derivative is its rate. The quaternion is the one place
 * the two vectors differ in length, and there `q̇ = ½·q ⊗ ω` with ω the child-frame
 * angular velocity.
 */
export function jointQDot(
  joint: JointModel,
  q: Float64Array,
  v: Float64Array,
  out: Float64Array,
): void {
  const nt = joint.freeTrans.length;
  for (let i = 0; i < nt; i++) out[joint.qOffset + i] = v[joint.vOffset + i]!;

  const rotQ = joint.qOffset + nt;
  const rotV = joint.vOffset + nt;
  if (joint.useQuaternion) {
    const x = q[rotQ]!, y = q[rotQ + 1]!, z = q[rotQ + 2]!, w = q[rotQ + 3]!;
    const wx = v[rotV]!, wy = v[rotV + 1]!, wz = v[rotV + 2]!;
    out[rotQ] = 0.5 * (w * wx + y * wz - z * wy);
    out[rotQ + 1] = 0.5 * (w * wy + z * wx - x * wz);
    out[rotQ + 2] = 0.5 * (w * wz + x * wy - y * wx);
    out[rotQ + 3] = 0.5 * (-x * wx - y * wy - z * wz);
  } else {
    for (let i = 0; i < joint.freeRot.length; i++) out[rotQ + i] = v[rotV + i]!;
  }
}

/** Renormalize a joint's quaternion after a step. A no-op for every other parametrization. */
export function normalizeJointQ(joint: JointModel, q: Float64Array): void {
  if (!joint.useQuaternion) return;
  const at = joint.qOffset + joint.freeTrans.length;
  const x = q[at]!, y = q[at + 1]!, z = q[at + 2]!, w = q[at + 3]!;
  const n = Math.hypot(x, y, z, w);
  if (n < 1e-12) {
    q[at] = 0; q[at + 1] = 0; q[at + 2] = 0; q[at + 3] = 1;
    return;
  }
  q[at] = x / n; q[at + 1] = y / n; q[at + 2] = z / n; q[at + 3] = w / n;
}

/**
 * How nearly the free rotational axes are parallel, as |sin| of the angle between them.
 *
 * Returns 1 (fully independent) whenever there is nothing to degenerate. The only mask that
 * can degenerate is free `rx` + free `rz` with `ry` locked, where the two axes coincide at
 * `ry = ±90°` — so this is a property of the mask and the locked angle, checkable once when
 * the model is built rather than sampled during the run.
 */
export function rotationalAxisSeparation(joint: JointModel): number {
  if (joint.useQuaternion || joint.freeRot.length < 2) return 1;
  // col_x·col_z = sin(θy) and col_x·col_y = col_y·col_z = 0, so only the x/z pair matters.
  if (joint.freeRot.includes(0) && joint.freeRot.includes(2)) {
    return Math.abs(Math.cos(joint.lockedRot[1]!));
  }
  return 1;
}
