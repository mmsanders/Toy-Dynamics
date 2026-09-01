import type { Actuator, Body, ContactPlane, ContactSphere, Hinge, SimSettings } from '../types';
import { buildSpec } from '../model/adapter';
import { buildModel, type MultibodyModel } from '../dyn/model';
import { makeDynamics, totalEnergy, type Dynamics } from '../dyn/forward';
import { makeStepScratch, step, type State, type StepScratch } from '../dyn/integrate';

/**
 * Running a trajectory.
 *
 * Written as a resumable object rather than a loop so the same code can drive the worker —
 * which must come up for air between batches to stay cancellable — and run straight through
 * in a test. `advance` integrates for a time budget and returns; call it until it says it
 * is done.
 *
 * ## Frame layout
 *
 * Frames are packed into one flat `Float64Array`, `count × stride`, laid out as
 *
 *     [ t | q (nq) | v (nv) | kinetic | potential ]
 *
 * Only the state is stored. Body poses, node kinematics and momentum are all recoverable
 * from `q` by running kinematics, which costs microseconds — so storing them would trade a
 * large multiple of the memory for nothing. It also means a frame means the same thing
 * whatever the scene happens to be showing.
 */

export type RunInput = {
  bodies: Record<string, Body>;
  hinges: Record<string, Hinge>;
  actuators: Record<string, Actuator>;
  contactSpheres: Record<string, ContactSphere>;
  contactPlanes: Record<string, ContactPlane>;
  settings: SimSettings;
};

export type RunStatus = 'running' | 'done' | 'diverged' | 'failed';

export type TrajectoryMeta = {
  stride: number;
  nq: number;
  nv: number;
  frameCount: number;
  dofNames: string[];
  /** The step actually used, which is `settings.dt` snapped so samples land on the grid. */
  dt: number;
  sampleInterval: number;
  /**
   * True when nothing in the model can add or remove energy — no enabled actuators, no
   * damping, no friction, no travel stops. Only then is energy drift a meaningful measure
   * of integration error rather than of real physics.
   */
  passive: boolean;
};

export type RunProgress = {
  status: RunStatus;
  /** Frames written so far. */
  written: number;
  /** Largest departure of total energy from its starting value, relative. */
  energyDrift: number;
  /** Simulated time at which the state stopped being finite. */
  divergedAt?: number;
  error?: string;
};

export type Run = {
  meta: TrajectoryMeta;
  data: Float64Array;
  progress: RunProgress;
  /** Integrate for up to `budgetMs`. Returns the frames now available. */
  advance: (budgetMs: number) => RunProgress;
};

/** Whether anything in the model can change the total energy. */
function isPassive(input: RunInput): boolean {
  // Contact spring energy is not yet included in trajectory energy, so even undamped
  // contact must suppress the conservation diagnostic rather than report false drift.
  if (Object.values(input.contactSpheres).some((sphere) => sphere.enabled)
      && Object.values(input.contactPlanes).some((plane) => plane.enabled)) return false;
  if (Object.values(input.contactSpheres).filter((sphere) => sphere.enabled).length > 1) return false;
  for (const actuator of Object.values(input.actuators)) {
    if (actuator.enabled) return false;
  }
  for (const hinge of Object.values(input.hinges)) {
    for (const dof of hinge.dof) {
      if (!dof.free) continue;
      if (dof.damping !== 0 || dof.friction !== 0 || dof.stiction !== 0) return false;
      if (dof.limit.enabled && dof.limit.stiffness > 0) return false;
    }
  }
  return true;
}

export function createRun(input: RunInput): Run | { error: string } {
  const built = buildSpec(input.bodies, input.hinges, input.actuators, input.settings, input.contactSpheres, input.contactPlanes);
  if (!built.ok) {
    return { error: built.problems[0]?.message ?? 'The model could not be assembled.' };
  }

  let model: MultibodyModel;
  let dynamics: Dynamics;
  let scratch: StepScratch;
  try {
    model = buildModel(built.spec);
    dynamics = makeDynamics(model);
    scratch = makeStepScratch(model);
  } catch (err) {
    return { error: err instanceof Error ? err.message : String(err) };
  }

  const { settings } = input;
  const sampleInterval = 1 / Math.max(settings.sampleRate, 1e-6);
  // Snap the step so a whole number of them lands exactly on each sample. Keeping samples
  // on an exact grid matters more than honouring the requested dt to the last digit — a
  // drifting sample time makes plots and CSV subtly wrong to compare against another tool.
  // `dt` is a maximum stability/accuracy step, not a target that may be rounded in either
  // direction. Rounding the ratio could make the actual step almost 50% larger than the
  // value the user selected (for example, 6 ms on a 10 ms sample grid became 10 ms). That
  // silently invalidates both the user's convergence choice and the timestep diagnostic.
  // Ceiling the ratio still lands exactly on every sample while never taking a larger step.
  const stepsPerSample = Math.max(1, Math.ceil(sampleInterval / Math.max(settings.dt, 1e-12)));
  const dt = sampleInterval / stepsPerSample;
  const frameCount = Math.max(1, Math.floor(settings.duration / sampleInterval) + 1);

  const stride = 1 + model.nq + model.nv + 2;
  const data = new Float64Array(stride * frameCount);
  const state: State = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };

  const meta: TrajectoryMeta = {
    stride,
    nq: model.nq,
    nv: model.nv,
    frameCount,
    dofNames: model.dofNames,
    dt,
    sampleInterval,
    passive: isPassive(input),
  };

  const progress: RunProgress = { status: 'running', written: 0, energyDrift: 0 };

  let frame = 0;
  let simTime = 0;
  let baselineEnergy = 0;
  let energyScale = 1;

  const writeFrame = (): void => {
    const energy = totalEnergy(dynamics, state.q, state.v);
    const base = frame * stride;
    data[base] = simTime;
    data.set(state.q, base + 1);
    data.set(state.v, base + 1 + model.nq);
    data[base + 1 + model.nq + model.nv] = energy.kinetic;
    data[base + 2 + model.nq + model.nv] = energy.potential;

    if (frame === 0) {
      baselineEnergy = energy.total;
      // Scale drift against the energy actually in play, so a model that happens to sit
      // near zero total energy does not report an infinite relative drift.
      energyScale = Math.max(Math.abs(energy.total), Math.abs(energy.kinetic), 1e-12);
    } else {
      const drift = Math.abs(energy.total - baselineEnergy) / energyScale;
      if (drift > progress.energyDrift) progress.energyDrift = drift;
    }
    frame++;
    progress.written = frame;
  };

  writeFrame();

  const advance = (budgetMs: number): RunProgress => {
    if (progress.status !== 'running') return progress;
    const deadline = performance.now() + budgetMs;

    while (frame < frameCount) {
      for (let i = 0; i < stepsPerSample; i++) {
        if (!step(dynamics, state, simTime, dt, settings.integrator, scratch)) {
          progress.status = 'diverged';
          progress.divergedAt = simTime;
          return progress;
        }
        simTime += dt;
      }
      // Re-derive the sample time from the frame index rather than accumulating, so
      // rounding never lets the reported time drift away from the grid.
      simTime = frame * sampleInterval;
      writeFrame();

      if (performance.now() >= deadline) return progress;
    }

    progress.status = 'done';
    return progress;
  };

  return { meta, data, progress, advance };
}
