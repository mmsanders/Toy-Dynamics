import type { MultibodyModel } from './model';
import { type Dynamics, forwardDynamics, zeroStuckVelocities } from './forward';
import { jointQDot, normalizeJointQ } from './joints';

/**
 * Time integration.
 *
 * Three explicit schemes, chosen so the user can trade accuracy against speed knowingly:
 *
 *  - **RK4** — the default. Four force evaluations per step, fourth-order accurate. A
 *    passive model holds its energy over a long run at a sane timestep, which is what makes
 *    the energy-drift warning a meaningful signal rather than a constant nag.
 *  - **RK2** (midpoint) — two evaluations, second order. A good middle setting.
 *  - **Semi-implicit Euler** — one evaluation, first order. Not accurate, but it is
 *    *symplectic*, so it stays bounded on oscillatory problems where explicit Euler would
 *    spiral outwards. This is the "just show me the shape of the motion" setting.
 *
 * None of them is implicit, so a genuinely stiff model — a hard travel stop, a very stiff
 * spring — still needs a small timestep. That is what the stiffness-vs-timestep diagnostic
 * exists to tell you before you waste a run finding out.
 */

export type State = { q: Float64Array; v: Float64Array };

export type Integrator = 'euler' | 'rk2' | 'rk4';

export type StepScratch = {
  qA: Float64Array;
  vA: Float64Array;
  qDotA: Float64Array;
  qDotB: Float64Array;
  qDotC: Float64Array;
  qDotD: Float64Array;
  aA: Float64Array;
  aB: Float64Array;
  aC: Float64Array;
  aD: Float64Array;
};

export function makeStepScratch(model: MultibodyModel): StepScratch {
  const q = () => new Float64Array(model.nq);
  const v = () => new Float64Array(model.nv);
  return {
    qA: q(),
    vA: v(),
    qDotA: q(),
    qDotB: q(),
    qDotC: q(),
    qDotD: q(),
    aA: v(),
    aB: v(),
    aC: v(),
    aD: v(),
  };
}

/** `dq/dt` for the whole model, joint by joint. */
function qDot(model: MultibodyModel, q: Float64Array, v: Float64Array, out: Float64Array): void {
  for (const link of model.links) jointQDot(link.joint, q, v, out);
}

/** Renormalize every quaternion-parametrized joint after a step has moved it off the unit sphere. */
export function normalizeState(model: MultibodyModel, q: Float64Array): void {
  for (const link of model.links) normalizeJointQ(link.joint, q);
}

export function stateIsFinite(state: State): boolean {
  for (let i = 0; i < state.q.length; i++) if (!Number.isFinite(state.q[i]!)) return false;
  for (let i = 0; i < state.v.length; i++) if (!Number.isFinite(state.v[i]!)) return false;
  return true;
}

/**
 * Advance one step, in place.
 *
 * Returns false if the step produced a non-finite state, which is the caller's cue to stop
 * the run and report divergence rather than filling a trajectory with NaN.
 */
export function step(
  d: Dynamics,
  state: State,
  t: number,
  dt: number,
  integrator: Integrator,
  s: StepScratch,
): boolean {
  const { model } = d;
  const { q, v } = state;
  const nq = model.nq;
  const nv = model.nv;

  if (nv === 0) return true;

  if (integrator === 'euler') {
    // Semi-implicit: the position update uses the *new* velocity, which is what makes it
    // symplectic and is the entire reason to prefer it over explicit Euler.
    forwardDynamics(d, q, v, t, s.aA, dt);
    zeroStuckVelocities(d, v);
    for (let i = 0; i < nv; i++) v[i] = v[i]! + dt * s.aA[i]!;
    qDot(model, q, v, s.qDotA);
    for (let i = 0; i < nq; i++) q[i] = q[i]! + dt * s.qDotA[i]!;
    normalizeState(model, q);
    return stateIsFinite(state);
  }

  if (integrator === 'rk2') {
    forwardDynamics(d, q, v, t, s.aA, dt);
    zeroStuckVelocities(d, v);
    qDot(model, q, v, s.qDotA);

    const half = dt / 2;
    for (let i = 0; i < nq; i++) s.qA[i] = q[i]! + half * s.qDotA[i]!;
    for (let i = 0; i < nv; i++) s.vA[i] = v[i]! + half * s.aA[i]!;
    normalizeState(model, s.qA);

    forwardDynamics(d, s.qA, s.vA, t + half, s.aB);
    qDot(model, s.qA, s.vA, s.qDotB);

    for (let i = 0; i < nq; i++) q[i] = q[i]! + dt * s.qDotB[i]!;
    for (let i = 0; i < nv; i++) v[i] = v[i]! + dt * s.aB[i]!;
    normalizeState(model, q);
    return stateIsFinite(state);
  }

  // RK4.
  const half = dt / 2;

  // Only this first evaluation is allowed to change which axes are stuck; the remaining
  // three reuse that set. A set that shifted between stages would make the derivative
  // discontinuous mid-step, which is exactly what a Runge-Kutta method assumes never happens.
  forwardDynamics(d, q, v, t, s.aA, dt);
  zeroStuckVelocities(d, v);
  qDot(model, q, v, s.qDotA);

  for (let i = 0; i < nq; i++) s.qA[i] = q[i]! + half * s.qDotA[i]!;
  for (let i = 0; i < nv; i++) s.vA[i] = v[i]! + half * s.aA[i]!;
  normalizeState(model, s.qA);
  forwardDynamics(d, s.qA, s.vA, t + half, s.aB);
  qDot(model, s.qA, s.vA, s.qDotB);

  for (let i = 0; i < nq; i++) s.qA[i] = q[i]! + half * s.qDotB[i]!;
  for (let i = 0; i < nv; i++) s.vA[i] = v[i]! + half * s.aB[i]!;
  normalizeState(model, s.qA);
  forwardDynamics(d, s.qA, s.vA, t + half, s.aC);
  qDot(model, s.qA, s.vA, s.qDotC);

  for (let i = 0; i < nq; i++) s.qA[i] = q[i]! + dt * s.qDotC[i]!;
  for (let i = 0; i < nv; i++) s.vA[i] = v[i]! + dt * s.aC[i]!;
  normalizeState(model, s.qA);
  forwardDynamics(d, s.qA, s.vA, t + dt, s.aD);
  qDot(model, s.qA, s.vA, s.qDotD);

  const sixth = dt / 6;
  for (let i = 0; i < nq; i++) {
    q[i] = q[i]! + sixth * (s.qDotA[i]! + 2 * s.qDotB[i]! + 2 * s.qDotC[i]! + s.qDotD[i]!);
  }
  for (let i = 0; i < nv; i++) {
    v[i] = v[i]! + sixth * (s.aA[i]! + 2 * s.aB[i]! + 2 * s.aC[i]! + s.aD[i]!);
  }
  normalizeState(model, q);
  return stateIsFinite(state);
}
