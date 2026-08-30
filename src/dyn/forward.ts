import {
  type KinematicsScratch,
  type MultibodyModel,
  makeKinematicsScratch,
  updateKinematics,
  updateVelocities,
} from './model';
import { type CrbaScratch, type Factorization, crba, factorize, makeCrbaScratch, makeFactorization, solveFactorized } from './crba';
import { type RneaScratch, kineticEnergy, makeRneaScratch, rnea } from './rnea';
import { type SV, type V3, sv, v3 } from './spatial';

/**
 * Forward dynamics: from a state to its accelerations.
 *
 * The equation solved is the ordinary one,
 *
 *     H(q)·q̈ = τ − C(q, q̇)
 *
 * where `C` carries Coriolis, centrifugal, gravity *and* the actuator contributions
 * (folded in as external forces during the inverse-dynamics sweep), and `τ` carries the
 * joint-local effects — springs, dampers, friction and travel stops.
 *
 * In reduced coordinates on a tree this is Kane's method: the generalized speeds are the
 * joint rates, and the columns of each joint's motion subspace are the partial velocities
 * that project applied forces onto them. `Sᵀ·f` in the inverse-dynamics sweep is precisely
 * the generalized-force projection Kane's equations call for.
 */

/**
 * Velocity scale over which Coulomb friction reverses.
 *
 * True Coulomb friction is discontinuous at zero, which an explicit integrator turns into
 * chatter — the force flips sign every step and pumps energy in. Regularizing over a small
 * velocity is the standard fix. It does mean this models sliding friction, not stiction: a
 * joint under a small load creeps instead of holding. Said plainly in the UI.
 */
const FRICTION_VELOCITY_SCALE = 1e-3;

/** Damping ratio applied at a travel stop, as a fraction of critical for the local inertia. */
const LIMIT_DAMPING_RATIO = 1.0;

export type Dynamics = {
  model: MultibodyModel;
  /** Mass matrix, nv × nv row-major. Refreshed by every `forwardDynamics` call. */
  H: Float64Array;
  /** Bias forces from the last evaluation. */
  C: Float64Array;
  /** Joint-local generalized forces from the last evaluation. */
  tau: Float64Array;
  factorization: Factorization;
  fext: (SV | null)[];
  fextStorage: SV[];
  kin: KinematicsScratch;
  rneaScratch: RneaScratch;
  crbaScratch: CrbaScratch;
  scratchV: V3;
  /** True when the last solve hit a non-positive pivot. */
  singular: boolean;
  singularDof: number;
};

export function makeDynamics(model: MultibodyModel): Dynamics {
  return {
    model,
    H: new Float64Array(model.nv * model.nv),
    C: new Float64Array(model.nv),
    tau: new Float64Array(model.nv),
    factorization: makeFactorization(model.nv),
    fext: model.links.map(() => null),
    fextStorage: model.links.map(() => sv()),
    kin: makeKinematicsScratch(),
    rneaScratch: makeRneaScratch(model),
    crbaScratch: makeCrbaScratch(model),
    scratchV: v3(),
    singular: false,
    singularDof: -1,
  };
}

/**
 * Turn the actuators into per-link spatial forces, in link coordinates.
 *
 * A force applied at a point carries a moment about the link origin, hence the `r × f`.
 * A moment is a free vector and gets no such term — which is exactly the distinction
 * between the two actuator kinds.
 */
function applyActuators(d: Dynamics, t: number): void {
  const { model } = d;
  for (let i = 0; i < d.fext.length; i++) d.fext[i] = null;
  if (model.actuators.length === 0) return;

  for (const store of d.fextStorage) store.fill(0);

  for (const act of model.actuators) {
    const scale = act.profile(t);
    if (scale === 0 || !Number.isFinite(scale)) continue;

    const link = model.links[act.link]!;
    let x = act.vector[0]! * scale;
    let y = act.vector[1]! * scale;
    let z = act.vector[2]! * scale;

    if (act.frame === 'world') {
      // Xworld maps world → link, which is the rotation a world-fixed direction needs.
      const e = link.Xworld.E;
      const wx = x, wy = y, wz = z;
      x = e[0]! * wx + e[1]! * wy + e[2]! * wz;
      y = e[3]! * wx + e[4]! * wy + e[5]! * wz;
      z = e[6]! * wx + e[7]! * wy + e[8]! * wz;
    }

    const f = d.fextStorage[act.link]!;
    if (act.kind === 'moment') {
      f[0] = f[0]! + x;
      f[1] = f[1]! + y;
      f[2] = f[2]! + z;
    } else {
      const px = act.point[0]!, py = act.point[1]!, pz = act.point[2]!;
      f[0] = f[0]! + (py * z - pz * y);
      f[1] = f[1]! + (pz * x - px * z);
      f[2] = f[2]! + (px * y - py * x);
      f[3] = f[3]! + x;
      f[4] = f[4]! + y;
      f[5] = f[5]! + z;
    }
    d.fext[act.link] = f;
  }
}

/**
 * Joint-local generalized forces: springs, dampers, Coulomb friction and travel stops.
 *
 * Stops are penalty springs rather than hard constraints — a hard stop in reduced
 * coordinates would need an impact law and a event-driven integrator, which is well past
 * what a back-of-the-envelope tool should carry. The penalty is damped against the actual
 * diagonal of the mass matrix, so the stop is roughly critically damped whatever the body
 * masses are, instead of ringing or exploding depending on the model's scale.
 */
function jointForces(d: Dynamics, q: Float64Array, v: Float64Array): void {
  const { model, tau, H } = d;
  const nv = model.nv;
  tau.fill(0);

  for (let i = 0; i < nv; i++) {
    const binding = model.dofBindings[i]!;
    const p = binding.params;
    const rate = v[i]!;
    let force = 0;

    if (p.damping !== 0) force -= p.damping * rate;
    if (p.friction !== 0) force -= p.friction * Math.tanh(rate / FRICTION_VELOCITY_SCALE);

    if (binding.qIndex >= 0) {
      const value = q[binding.qIndex]!;
      if (p.stiffness !== 0) force -= p.stiffness * (value - p.rest);

      if (p.limitEnabled && p.limitStiffness > 0) {
        let penetration = 0;
        if (value < p.limitLo) penetration = value - p.limitLo;
        else if (value > p.limitHi) penetration = value - p.limitHi;
        if (penetration !== 0) {
          const effectiveMass = Math.max(H[i * nv + i]!, 1e-12);
          const damping = 2 * LIMIT_DAMPING_RATIO * Math.sqrt(p.limitStiffness * effectiveMass);
          force -= p.limitStiffness * penetration + damping * rate;
        }
      }
    }

    tau[i] = force;
  }
}

/**
 * Evaluate `q̈` for a state.
 *
 * Order matters: kinematics and velocities first (both the mass matrix and the bias forces
 * read them), then H, then the joint forces — which need H's diagonal for the stop damping
 * — then the bias forces, then the solve.
 */
export function forwardDynamics(
  d: Dynamics,
  q: Float64Array,
  v: Float64Array,
  t: number,
  qdd: Float64Array,
): void {
  const { model } = d;
  if (model.nv === 0) return;

  updateKinematics(model, q, v, d.kin);
  updateVelocities(model, d.kin);

  crba(model, d.H, d.crbaScratch);
  jointForces(d, q, v);
  applyActuators(d, t);
  rnea(model, null, d.fext, d.C, d.rneaScratch);

  const rhs = qdd;
  for (let i = 0; i < model.nv; i++) rhs[i] = d.tau[i]! - d.C[i]!;

  factorize(d.H, d.factorization);
  d.singular = d.factorization.failedAt >= 0;
  d.singularDof = d.factorization.failedAt;
  solveFactorized(d.factorization, rhs, qdd);
}

// ---------------------------------------------------------------------------
// Energy
// ---------------------------------------------------------------------------

/**
 * Total mechanical energy.
 *
 * The zero of potential energy is arbitrary — it is measured against the world origin —
 * which is fine, because only its *drift* is meaningful. A passive model whose energy
 * wanders is a model whose timestep is too big, and that is the check this feeds.
 */
export function totalEnergy(d: Dynamics, q: Float64Array, v: Float64Array): { kinetic: number; potential: number; total: number } {
  const { model } = d;
  updateKinematics(model, q, v, d.kin);
  updateVelocities(model, d.kin);

  const kinetic = kineticEnergy(model, d.rneaScratch);

  let potential = 0;
  const g = model.gravity;
  for (const link of model.links) {
    if (link.mass === 0) continue;
    // The link's centre of mass in link coordinates is h/m; push it out to world.
    const cx = link.I.h[0]! / link.I.m;
    const cy = link.I.h[1]! / link.I.m;
    const cz = link.I.h[2]! / link.I.m;
    const e = link.Xworld.E;
    // Xworld maps world → link, so link → world is its transpose.
    const wx = e[0]! * cx + e[3]! * cy + e[6]! * cz + link.Xworld.r[0]!;
    const wy = e[1]! * cx + e[4]! * cy + e[7]! * cz + link.Xworld.r[1]!;
    const wz = e[2]! * cx + e[5]! * cy + e[8]! * cz + link.Xworld.r[2]!;
    potential -= link.I.m * (g[0]! * wx + g[1]! * wy + g[2]! * wz);
  }

  return { kinetic, potential, total: kinetic + potential };
}
