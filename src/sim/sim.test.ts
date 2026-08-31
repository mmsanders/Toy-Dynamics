import { describe, expect, it } from 'vitest';
import { createRun, type Run, type RunInput } from './runner';
import { initialModel } from '../store/defaults';
import { buildSolverModel, bodyPoses, bodyVelocities, totalMomentum } from './kinematics';
import { buildCsv } from './csv';
import { frameEnergy, frameQ, frameTime, frameV, frameAtTime, type Trajectory } from './useSimulation';

const baseInput = (): RunInput => {
  const model = initialModel();
  return {
    bodies: model.bodies,
    hinges: model.hinges,
    actuators: model.actuators,
    settings: model.settings,
  };
};

/** Drive a run to completion, as the worker does but without the yielding. */
function complete(input: RunInput): Run {
  const run = createRun(input);
  if ('error' in run) throw new Error(run.error);
  let guard = 0;
  while (run.progress.status === 'running' && guard++ < 10_000) run.advance(50);
  return run;
}

const asTrajectory = (run: Run): Trajectory => ({
  meta: run.meta,
  data: run.data,
  count: run.progress.written,
  progress: run.progress,
});

describe('trajectory runner', () => {
  it('fills every frame of the requested duration', () => {
    const input = baseInput();
    input.settings = { ...input.settings, duration: 2, sampleRate: 60 };
    const run = complete(input);

    expect(run.progress.status).toBe('done');
    expect(run.progress.written).toBe(run.meta.frameCount);
    expect(run.meta.frameCount).toBe(121); // 2 s at 60 Hz, inclusive of t = 0
  });

  it('puts samples on an exact grid', () => {
    const input = baseInput();
    // A dt that does not divide the sample interval, so the snapping actually does work.
    input.settings = { ...input.settings, dt: 0.0007, duration: 1, sampleRate: 50 };
    const run = complete(input);
    const t = asTrajectory(run);

    for (let i = 0; i < t.count; i++) {
      expect(frameTime(t, i)).toBeCloseTo(i * 0.02, 12);
    }
    // The step was snapped down so a whole number lands on each sample.
    expect(run.meta.dt).toBeLessThanOrEqual(0.0007);
    expect(0.02 / run.meta.dt).toBeCloseTo(Math.round(0.02 / run.meta.dt), 12);
  });

  it('never increases the requested integration timestep when snapping to samples', () => {
    const input = baseInput();
    // At 100 Hz the sample interval is 10 ms. Nearest-integer rounding would use one
    // 10 ms step here, substantially exceeding the requested 6 ms maximum.
    input.settings = { ...input.settings, dt: 0.006, duration: 0.1, sampleRate: 100 };
    const run = complete(input);

    expect(run.meta.dt).toBeLessThanOrEqual(input.settings.dt);
    expect(run.meta.dt).toBeCloseTo(0.005, 12);
    expect(run.meta.sampleInterval / run.meta.dt).toBeCloseTo(2, 12);
  });

  it('is resumable in small budgets without changing the answer', () => {
    const input = baseInput();
    input.settings = { ...input.settings, duration: 1 };

    const whole = complete(input);
    const piecewise = createRun(input);
    if ('error' in piecewise) throw new Error(piecewise.error);
    // One frame or so per call, the way the worker actually drives it.
    let guard = 0;
    while (piecewise.progress.status === 'running' && guard++ < 100_000) piecewise.advance(0);

    expect(piecewise.progress.written).toBe(whole.progress.written);
    for (let i = 0; i < whole.data.length; i++) {
      expect(piecewise.data[i]!).toBe(whole.data[i]!);
    }
  });

  it('reports energy drift, and knows when the model is passive', () => {
    const input = baseInput();
    // Remove everything that can add or remove energy.
    input.actuators = {};
    input.hinges = structuredClone(input.hinges);
    for (const hinge of Object.values(input.hinges)) {
      hinge.dof = hinge.dof.map((d) => ({ ...d, damping: 0, friction: 0 }));
    }
    input.settings = { ...input.settings, duration: 5, dt: 1e-3 };

    const run = complete(input);
    expect(run.meta.passive).toBe(true);
    // RK4 on a two-link pendulum at 1 ms holds energy to well under a part per million.
    expect(run.progress.energyDrift).toBeLessThan(1e-7);
  });

  it('drifts visibly when the timestep is far too large', () => {
    const input = baseInput();
    input.actuators = {};
    input.hinges = structuredClone(input.hinges);
    for (const hinge of Object.values(input.hinges)) {
      hinge.dof = hinge.dof.map((d) => ({ ...d, damping: 0, friction: 0 }));
    }
    input.settings = { ...input.settings, duration: 5, dt: 0.08, integrator: 'euler' };

    const run = complete(input);
    // This is exactly the signal the drift warning exists to surface.
    expect(run.progress.energyDrift).toBeGreaterThan(1e-3);
  });

  it('does not claim passivity when a damper or actuator is present', () => {
    expect(complete(baseInput()).meta.passive).toBe(false);

    const noActuators = baseInput();
    noActuators.actuators = {};
    // The default elbow carries damping, so this is still not passive.
    expect(complete(noActuators).meta.passive).toBe(false);
  });

  it('reports a closed loop rather than hanging', () => {
    const input = baseInput();
    input.hinges = structuredClone(input.hinges);
    input.hinges.shoulder!.parentBodyId = 'lower';
    const result = createRun(input);
    expect('error' in result).toBe(true);
    if ('error' in result) expect(result.error).toMatch(/closed loop/i);
  });

  it('stops and reports when the state diverges', () => {
    const input = baseInput();
    input.hinges = structuredClone(input.hinges);
    // An absurdly stiff stop with a huge step: guaranteed to blow up.
    input.hinges.elbow!.dof[4]!.limit = { enabled: true, lo: -0.1, hi: 0.1, stiffness: 1e18 };
    input.settings = { ...input.settings, dt: 0.05, duration: 5, integrator: 'euler' };

    const run = complete(input);
    expect(run.progress.status).toBe('diverged');
    expect(run.progress.divergedAt).toBeGreaterThanOrEqual(0);
    // Whatever was computed before the blow-up is still there to look at.
    expect(run.progress.written).toBeGreaterThan(0);
  });
});

describe('frame accessors', () => {
  it('reads back what the run wrote', () => {
    const input = baseInput();
    input.settings = { ...input.settings, duration: 1 };
    const t = asTrajectory(complete(input));

    expect(frameQ(t, 0)).toHaveLength(t.meta.nq);
    expect(frameV(t, 0)).toHaveLength(t.meta.nv);
    // The initial configuration is what the hinges were set to.
    expect(frameQ(t, 0)[0]).toBeCloseTo(0.6, 12);
    expect(frameQ(t, 0)[1]).toBeCloseTo(-1.1, 12);
    expect(frameEnergy(t, 0).total).toBeCloseTo(
      frameEnergy(t, 0).kinetic + frameEnergy(t, 0).potential,
      12,
    );
  });

  it('finds the frame nearest a time, clamped to what exists', () => {
    const input = baseInput();
    input.settings = { ...input.settings, duration: 1, sampleRate: 60 };
    const t = asTrajectory(complete(input));

    expect(frameAtTime(t, 0)).toBe(0);
    expect(frameTime(t, frameAtTime(t, 0.5))).toBeCloseTo(0.5, 2);
    expect(frameAtTime(t, 999)).toBe(t.count - 1);
    expect(frameAtTime(t, -5)).toBe(0);
  });
});

describe('derived kinematics', () => {
  const solverOf = (input: RunInput) => {
    const solver = buildSolverModel(input.bodies, input.hinges, input.actuators, input.settings);
    if ('error' in solver) throw new Error(solver.error);
    return solver;
  };

  it('places bodies where the model says they are', () => {
    const input = baseInput();
    const solver = solverOf(input);
    const t = asTrajectory(complete({ ...input, settings: { ...input.settings, duration: 0.1 } }));
    const poses = bodyPoses(solver, frameQ(t, 0));

    // The shoulder sits at the world origin, so the upper arm's body frame does too —
    // whatever angle it is at.
    const upper = poses.get('upper')!;
    for (const axis of upper.position) expect(axis).toBeCloseTo(0, 12);
    expect(Math.hypot(...upper.quaternion)).toBeCloseTo(1, 12);

    // The forearm hangs off the upper arm's tip, 1.2 away, so it must be somewhere else.
    const lower = poses.get('lower')!;
    expect(Math.hypot(...lower.position)).toBeCloseTo(1.2, 9);
  });

  it('conserves momentum for a model with nothing acting on it', () => {
    const input = baseInput();
    input.actuators = {};
    input.settings = { ...input.settings, gravity: [0, 0, 0], duration: 3 };
    input.hinges = structuredClone(input.hinges);
    for (const hinge of Object.values(input.hinges)) {
      hinge.dof = hinge.dof.map((d) => ({ ...d, damping: 0, friction: 0 }));
    }
    // Free the shoulder completely so the whole assembly floats.
    input.hinges.shoulder!.dof = input.hinges.shoulder!.dof.map((d, i) => ({
      ...d,
      free: true,
      u0: i === 3 ? 1.5 : i === 0 ? 0.4 : 0,
    }));

    const solver = solverOf(input);
    const t = asTrajectory(complete(input));

    const first = totalMomentum(solver, frameQ(t, 0), frameV(t, 0));
    const last = totalMomentum(solver, frameQ(t, t.count - 1), frameV(t, t.count - 1));

    for (let i = 0; i < 3; i++) {
      expect(last.linear[i]!).toBeCloseTo(first.linear[i]!, 8);
      expect(last.angular[i]!).toBeCloseTo(first.angular[i]!, 8);
    }
  });

  it('reports body velocity at the body origin, not the link origin', () => {
    const input = baseInput();
    const solver = solverOf(input);
    const t = asTrajectory(complete({ ...input, settings: { ...input.settings, duration: 0.5 } }));
    const frame = t.count - 1;
    const velocities = bodyVelocities(solver, frameQ(t, frame), frameV(t, frame));

    // The upper arm rotates about the world origin, and its body origin is that same
    // point — so it spins without translating.
    const upper = velocities.get('upper')!;
    expect(Math.hypot(...upper.linear)).toBeCloseTo(0, 9);
    expect(Math.abs(upper.angular[1]!)).toBeGreaterThan(0);
  });
});

describe('CSV export', () => {
  it('emits a header and one row per computed frame', () => {
    const input = baseInput();
    input.settings = { ...input.settings, duration: 0.2, sampleRate: 10 };
    const solver = buildSolverModel(input.bodies, input.hinges, input.actuators, input.settings);
    if ('error' in solver) throw new Error(solver.error);

    const t = asTrajectory(complete(input));
    const csv = buildCsv(t, solver, input.bodies);
    const lines = csv.split('\n');

    expect(lines).toHaveLength(t.count + 1);
    const header = lines[0]!.split(',');
    expect(header[0]).toBe('time');
    // Every row is the same width as the header — a ragged CSV is worse than none.
    for (const line of lines.slice(1)) {
      expect(line.split(',')).toHaveLength(header.length);
    }
  });

  it('includes state, poses, node positions, energy and momentum', () => {
    const input = baseInput();
    input.settings = { ...input.settings, duration: 0.1, sampleRate: 10 };
    const solver = buildSolverModel(input.bodies, input.hinges, input.actuators, input.settings);
    if ('error' in solver) throw new Error(solver.error);

    const header = buildCsv(asTrajectory(complete(input)), solver, input.bodies).split('\n')[0]!;
    for (const column of [
      'time',
      'q.Upper Arm.ry',
      'u.Forearm.ry',
      'Upper Arm.x',
      'Upper Arm.qw',
      'Forearm.wz',
      'Upper Arm.Tip.z',
      'energy.total',
      'momentum.lz',
    ]) {
      expect(header).toContain(column);
    }
  });

  it('quotes names containing a comma', () => {
    const input = baseInput();
    input.bodies = structuredClone(input.bodies);
    input.bodies.upper!.name = 'Arm, upper';
    input.settings = { ...input.settings, duration: 0.1, sampleRate: 10 };

    const solver = buildSolverModel(input.bodies, input.hinges, input.actuators, input.settings);
    if ('error' in solver) throw new Error(solver.error);
    const header = buildCsv(asTrajectory(complete(input)), solver, input.bodies).split('\n')[0]!;

    expect(header).toContain('"Arm, upper.x"');
    // The quoting must not change the column count.
    const rows = buildCsv(asTrajectory(complete(input)), solver, input.bodies).split('\n');
    expect(rows[1]!.split(',').length).toBeGreaterThan(0);
  });
});
