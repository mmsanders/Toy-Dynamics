import { describe, expect, it } from 'vitest';
import { jcalc, makeJointModel, makeJointWorkspace, rotationalAxisSeparation, writeInitialQ } from './joints';
import type { JointModel } from './joints';

/**
 * Verification of the joint motion subspace against numerical differentiation.
 *
 * This is the test that matters most for multi-DOF joints. `S` and its apparent derivative
 * `Ṡ` are derived analytically in `joints.ts`, and an error in either is invisible on a
 * single-DOF joint but silently wrong the moment two axes on the same hinge move together.
 * So both are checked, for **every one of the 64 free/locked masks**, against finite
 * differences of the joint transform itself — which depends on none of the derivation.
 */

/** Deterministic pseudo-randomness, so a failure is always reproducible. */
function lcg(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/**
 * Advance the joint configuration by `h` *exactly*, for constant speeds.
 *
 * Exactness matters: a first-order step would leave an O(h) error in the central
 * difference, swamping the very quantity under test. Linear coordinates advance exactly by
 * `h·v` already, and a quaternion under constant body-frame ω advances exactly by
 * composition with the rotation of angle `|ω|h` about `ω̂`.
 */
function advanceExact(joint: JointModel, q: Float64Array, v: Float64Array, h: number): Float64Array {
  const out = Float64Array.from(q);
  const nt = joint.freeTrans.length;
  for (let i = 0; i < nt; i++) out[joint.qOffset + i] = q[joint.qOffset + i]! + h * v[joint.vOffset + i]!;

  const rotQ = joint.qOffset + nt;
  const rotV = joint.vOffset + nt;
  if (joint.useQuaternion) {
    const wx = v[rotV]!, wy = v[rotV + 1]!, wz = v[rotV + 2]!;
    const mag = Math.hypot(wx, wy, wz);
    let dx = 0, dy = 0, dz = 0, dw = 1;
    if (mag > 0) {
      const half = (mag * h) / 2;
      const s = Math.sin(half) / mag;
      dx = wx * s; dy = wy * s; dz = wz * s; dw = Math.cos(half);
    }
    // Body-frame rotation composes on the right.
    const x = q[rotQ]!, y = q[rotQ + 1]!, z = q[rotQ + 2]!, w = q[rotQ + 3]!;
    out[rotQ] = w * dx + x * dw + y * dz - z * dy;
    out[rotQ + 1] = w * dy - x * dz + y * dw + z * dx;
    out[rotQ + 2] = w * dz + x * dy - y * dx + z * dw;
    out[rotQ + 3] = w * dw - x * dx - y * dy - z * dz;
  } else {
    for (let i = 0; i < joint.freeRot.length; i++) {
      out[rotQ + i] = q[rotQ + i]! + h * v[rotV + i]!;
    }
  }
  return out;
}

/** All 64 combinations of the six free/locked flags. */
function allMasks(): boolean[][] {
  const masks: boolean[][] = [];
  for (let bits = 0; bits < 64; bits++) {
    masks.push([0, 1, 2, 3, 4, 5].map((i) => (bits & (1 << i)) !== 0));
  }
  return masks;
}

const describeMask = (mask: boolean[]): string =>
  ['tx', 'ty', 'tz', 'rx', 'ry', 'rz'].filter((_, i) => mask[i]).join('+') || 'weld';

describe('joint motion subspace', () => {
  const masks = allMasks();

  it('covers all 64 masks', () => {
    expect(masks).toHaveLength(64);
  });

  it.each(masks.map((mask) => [describeMask(mask), mask] as const))(
    'S reproduces the joint velocity for %s',
    (_label, mask) => {
      const rand = lcg(12345);
      const values = Array.from({ length: 6 }, () => rand() * 2 - 1);
      const joint = makeJointModel(mask, values, 0, 0);
      if (joint.nv === 0) return;

      const q = new Float64Array(joint.nq);
      writeInitialQ(joint, values, q);
      const v = Float64Array.from({ length: joint.nv }, () => rand() * 2 - 1);

      const w = makeJointWorkspace(joint);
      jcalc(joint, q, v, w);

      // Numerically differentiate the transform. XJ.E is Rᵀ (child → parent transposed),
      // so R is its transpose, and XJ.r is the child origin in parent coordinates.
      const h = 1e-5;
      const wPlus = makeJointWorkspace(joint);
      const wMinus = makeJointWorkspace(joint);
      jcalc(joint, advanceExact(joint, q, v, h), v, wPlus);
      jcalc(joint, advanceExact(joint, q, v, -h), v, wMinus);

      const rOf = (ws: typeof w): number[] => {
        const e = ws.XJ.E;
        return [e[0]!, e[3]!, e[6]!, e[1]!, e[4]!, e[7]!, e[2]!, e[5]!, e[8]!];
      };
      const R = rOf(w);
      const Rp = rOf(wPlus);
      const Rm = rOf(wMinus);
      const Rdot = R.map((_, i) => (Rp[i]! - Rm[i]!) / (2 * h));

      // ω̂ = Rᵀ·Ṙ, read off the antisymmetric part.
      const rtrd = (i: number, j: number): number => {
        let acc = 0;
        for (let k = 0; k < 3; k++) acc += R[k * 3 + i]! * Rdot[k * 3 + j]!;
        return acc;
      };
      const omega = [rtrd(2, 1), rtrd(0, 2), rtrd(1, 0)];

      const dDot = [0, 1, 2].map((i) => (wPlus.XJ.r[i]! - wMinus.XJ.r[i]!) / (2 * h));
      // The linear half of the spatial velocity is Rᵀ·ḋ, in child coordinates.
      const linear = [0, 1, 2].map((i) => R[0 * 3 + i]! * dDot[0]! + R[1 * 3 + i]! * dDot[1]! + R[2 * 3 + i]! * dDot[2]!);

      for (let i = 0; i < 3; i++) {
        expect(w.vJ[i]!).toBeCloseTo(omega[i]!, 6);
        expect(w.vJ[i + 3]!).toBeCloseTo(linear[i]!, 6);
      }
    },
  );

  it.each(masks.map((mask) => [describeMask(mask), mask] as const))(
    'the apparent derivative of S is right for %s',
    (_label, mask) => {
      const rand = lcg(98765);
      const values = Array.from({ length: 6 }, () => rand() * 2 - 1);
      const joint = makeJointModel(mask, values, 0, 0);
      if (joint.nv === 0) return;

      const q = new Float64Array(joint.nq);
      writeInitialQ(joint, values, q);
      const v = Float64Array.from({ length: joint.nv }, () => rand() * 2 - 1);

      const w = makeJointWorkspace(joint);
      jcalc(joint, q, v, w);

      // cJ is Ṡ·q̇ holding q̇ fixed, which is exactly the time derivative of vJ along a
      // constant-speed trajectory.
      const h = 1e-5;
      const wPlus = makeJointWorkspace(joint);
      const wMinus = makeJointWorkspace(joint);
      jcalc(joint, advanceExact(joint, q, v, h), v, wPlus);
      jcalc(joint, advanceExact(joint, q, v, -h), v, wMinus);

      for (let i = 0; i < 6; i++) {
        const numeric = (wPlus.vJ[i]! - wMinus.vJ[i]!) / (2 * h);
        expect(w.cJ[i]!).toBeCloseTo(numeric, 5);
      }
    },
  );
});

describe('joint parametrization', () => {
  it('uses a quaternion only when all three rotations are free', () => {
    expect(makeJointModel([false, false, false, true, true, true], [0, 0, 0, 0, 0, 0], 0, 0).useQuaternion).toBe(true);
    expect(makeJointModel([true, true, true, true, true, true], [0, 0, 0, 0, 0, 0], 0, 0).useQuaternion).toBe(true);
    expect(makeJointModel([false, false, false, true, true, false], [0, 0, 0, 0, 0, 0], 0, 0).useQuaternion).toBe(false);
  });

  it('sizes the state correctly for each parametrization', () => {
    const ball = makeJointModel([false, false, false, true, true, true], [0, 0, 0, 0, 0, 0], 0, 0);
    expect(ball.nv).toBe(3);
    expect(ball.nq).toBe(4); // a quaternion, not three angles

    const free = makeJointModel([true, true, true, true, true, true], [0, 0, 0, 0, 0, 0], 0, 0);
    expect(free.nv).toBe(6);
    expect(free.nq).toBe(7);

    const revolute = makeJointModel([false, false, false, false, true, false], [0, 0, 0, 0, 0, 0], 0, 0);
    expect(revolute.nv).toBe(1);
    expect(revolute.nq).toBe(1);
  });

  it('round-trips initial Euler angles through the quaternion parametrization', () => {
    const values = [0, 0, 0, 0.3, -0.7, 1.1];
    const joint = makeJointModel([false, false, false, true, true, true], values, 0, 0);
    const q = new Float64Array(joint.nq);
    writeInitialQ(joint, values, q);

    const w = makeJointWorkspace(joint);
    jcalc(joint, q, new Float64Array(joint.nv), w);

    // The same angles under the Euler parametrization must give the identical rotation —
    // otherwise a hinge would jump when its third rotation is unlocked.
    const euler = makeJointModel([false, false, false, true, true, false], values, 0, 0);
    const qe = new Float64Array(euler.nq);
    writeInitialQ(euler, values, qe);
    const we = makeJointWorkspace(euler);
    // Lock rz at the same value so the two describe the same orientation.
    euler.lockedRot[2] = values[5]!;
    jcalc(euler, qe, new Float64Array(euler.nv), we);

    for (let i = 0; i < 9; i++) expect(w.XJ.E[i]!).toBeCloseTo(we.XJ.E[i]!, 12);
  });
});

describe('degenerate axis detection', () => {
  it('flags free rx + rz when the locked ry pins them together', () => {
    const atNinety = makeJointModel(
      [false, false, false, true, false, true],
      [0, 0, 0, 0, Math.PI / 2, 0],
      0,
      0,
    );
    expect(rotationalAxisSeparation(atNinety)).toBeCloseTo(0, 12);
  });

  it('leaves the same pair alone when ry is not at a right angle', () => {
    const flat = makeJointModel([false, false, false, true, false, true], [0, 0, 0, 0, 0, 0], 0, 0);
    expect(rotationalAxisSeparation(flat)).toBeCloseTo(1, 12);
  });

  it('never flags the other two-rotation pairs, which cannot degenerate', () => {
    for (const mask of [
      [false, false, false, true, true, false],
      [false, false, false, false, true, true],
    ]) {
      for (const middle of [0, Math.PI / 2, -Math.PI / 2, 1.234]) {
        const joint = makeJointModel(mask, [0, 0, 0, middle, middle, middle], 0, 0);
        expect(rotationalAxisSeparation(joint)).toBe(1);
      }
    }
  });

  it('reports full separation for the quaternion parametrization', () => {
    const ball = makeJointModel([false, false, false, true, true, true], [0, 0, 0, 1, 1, 1], 0, 0);
    expect(rotationalAxisSeparation(ball)).toBe(1);
  });
});
