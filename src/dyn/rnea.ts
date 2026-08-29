import type { MultibodyModel } from './model';
import {
  type SV,
  type V3,
  applyForceTranspose,
  applyMotion,
  crossForce,
  crossMotion,
  inertiaApply,
  sv,
  v3,
} from './spatial';

/**
 * Inverse dynamics by the recursive Newton-Euler algorithm.
 *
 * Given a configuration, its velocities and a candidate acceleration, this returns the
 * generalized forces that would produce it. Called with zero acceleration it returns the
 * **bias forces** `C(q, q̇)` — Coriolis, centrifugal and gravity — which is exactly what
 * forward dynamics needs on the right-hand side.
 *
 * Gravity enters as a fictitious base acceleration of `−g` rather than as a per-body force.
 * That is the standard trick and it is not just tidiness: it costs one vector at the root
 * instead of a force term on every link, and it automatically lands in the right frame for
 * each body.
 */

export type RneaScratch = {
  /** Spatial acceleration per link, in link coordinates. */
  a: SV[];
  /** Spatial force per link, in link coordinates. */
  f: SV[];
  a0: SV;
  tmpA: SV;
  tmpB: SV;
  vec: V3;
};

export function makeRneaScratch(model: MultibodyModel): RneaScratch {
  return {
    a: model.links.map(() => sv()),
    f: model.links.map(() => sv()),
    a0: sv(),
    tmpA: sv(),
    tmpB: sv(),
    vec: v3(),
  };
}

/**
 * Run inverse dynamics.
 *
 * Expects `updateKinematics` and `updateVelocities` to have been run for the current state
 * — this reads the joint workspaces rather than recomputing them, so a forward-dynamics
 * evaluation costs one `jcalc` per joint rather than one per algorithm.
 *
 * `qdd` may be null, meaning zero acceleration. `fext` may be null, or an array of spatial
 * forces per link **expressed in that link's own coordinates**.
 */
export function rnea(
  model: MultibodyModel,
  qdd: Float64Array | null,
  fext: (SV | null)[] | null,
  tau: Float64Array,
  s: RneaScratch,
): void {
  const links = model.links;

  // Base acceleration of −g, so gravity appears in every link's inertial force.
  s.a0[0] = 0;
  s.a0[1] = 0;
  s.a0[2] = 0;
  s.a0[3] = -model.gravity[0]!;
  s.a0[4] = -model.gravity[1]!;
  s.a0[5] = -model.gravity[2]!;

  for (let i = 0; i < links.length; i++) {
    const link = links[i]!;
    const parentA = link.parent < 0 ? s.a0 : s.a[link.parent]!;
    const a = s.a[i]!;

    applyMotion(link.X, parentA, a, s.vec);

    // Joint bias: the apparent derivative of S, plus the term from the joint velocity
    // being expressed in a frame that is itself moving.
    crossMotion(link.v, link.jw.vJ, s.tmpA);
    for (let r = 0; r < 6; r++) a[r] = a[r]! + link.jw.cJ[r]! + s.tmpA[r]!;

    if (qdd) {
      const S = link.jw.S;
      const nv = link.joint.nv;
      for (let c = 0; c < nv; c++) {
        const acc = qdd[link.joint.vOffset + c]!;
        if (acc === 0) continue;
        const base = 6 * c;
        for (let r = 0; r < 6; r++) a[r] = a[r]! + acc * S[base + r]!;
      }
    }

    // f = I·a + v ×* (I·v) − f_ext
    const f = s.f[i]!;
    inertiaApply(link.I, a, f);
    inertiaApply(link.I, link.v, s.tmpA);
    crossForce(link.v, s.tmpA, s.tmpB);
    for (let r = 0; r < 6; r++) f[r] = f[r]! + s.tmpB[r]!;

    const ext = fext?.[i];
    if (ext) for (let r = 0; r < 6; r++) f[r] = f[r]! - ext[r]!;
  }

  for (let i = links.length - 1; i >= 0; i--) {
    const link = links[i]!;
    const f = s.f[i]!;
    const S = link.jw.S;
    const nv = link.joint.nv;

    for (let c = 0; c < nv; c++) {
      const base = 6 * c;
      let acc = 0;
      for (let r = 0; r < 6; r++) acc += S[base + r]! * f[r]!;
      tau[link.joint.vOffset + c] = acc;
    }

    if (link.parent >= 0) {
      applyForceTranspose(link.X, f, s.tmpA, s.vec);
      const pf = s.f[link.parent]!;
      for (let r = 0; r < 6; r++) pf[r] = pf[r]! + s.tmpA[r]!;
    }
  }
}

/**
 * Total kinetic energy, `½·Σ vᵀ·I·v`.
 *
 * Reads the link velocities left by `updateVelocities`. Together with the potential energy
 * this is the drift monitor that tells the user their timestep is too big.
 */
export function kineticEnergy(model: MultibodyModel, s: RneaScratch): number {
  let total = 0;
  for (const link of model.links) {
    inertiaApply(link.I, link.v, s.tmpA);
    let dot = 0;
    for (let r = 0; r < 6; r++) dot += link.v[r]! * s.tmpA[r]!;
    total += 0.5 * dot;
  }
  return total;
}
