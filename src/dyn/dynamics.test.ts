import { describe, expect, it } from 'vitest';
import { buildModel } from './model';
import { makeDynamics, forwardDynamics, totalEnergy } from './forward';
import { makeStepScratch, step, type State } from './integrate';
import { crba } from './crba';
import { rnea } from './rnea';
import { bodySpec, doublePendulum, dofParams, freeBody, hingeSpec, MASK, pendulum, slider } from './fixtures';
import { parallelAxisShift } from './inertia';
import { updateKinematics, updateVelocities } from './model';

/**
 * The physics tests.
 *
 * The whole point of this tool is to be the thing you check another simulator against, so
 * these are not smoke tests — each one pins a result that has an independent answer:
 * a closed-form solution, a conservation law, or a second algorithm computing the same
 * quantity a different way.
 */

function run(spec: ReturnType<typeof pendulum>, dt: number, duration: number, integrator: 'euler' | 'rk2' | 'rk4' = 'rk4') {
  const model = buildModel(spec);
  const d = makeDynamics(model);
  const scratch = makeStepScratch(model);
  const state: State = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };

  const steps = Math.round(duration / dt);
  const samples: { t: number; q: Float64Array; v: Float64Array; energy: number }[] = [];
  samples.push({ t: 0, q: Float64Array.from(state.q), v: Float64Array.from(state.v), energy: totalEnergy(d, state.q, state.v).total });

  for (let i = 0; i < steps; i++) {
    const ok = step(d, state, i * dt, dt, integrator, scratch);
    expect(ok).toBe(true);
    samples.push({
      t: (i + 1) * dt,
      q: Float64Array.from(state.q),
      v: Float64Array.from(state.v),
      energy: totalEnergy(d, state.q, state.v).total,
    });
  }
  return { model, dynamics: d, samples, state };
}

/** Period from upward zero crossings of a coordinate, linearly interpolated. */
function periodFromCrossings(samples: { t: number; q: Float64Array }[], index: number): number {
  const crossings: number[] = [];
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1]!.q[index]!;
    const b = samples[i]!.q[index]!;
    if (a < 0 && b >= 0) {
      const frac = -a / (b - a);
      crossings.push(samples[i - 1]!.t + frac * (samples[i]!.t - samples[i - 1]!.t));
    }
  }
  expect(crossings.length).toBeGreaterThan(1);
  const gaps: number[] = [];
  for (let i = 1; i < crossings.length; i++) gaps.push(crossings[i]! - crossings[i - 1]!);
  return gaps.reduce((a, b) => a + b, 0) / gaps.length;
}

describe('pendulum', () => {
  it('matches the small-angle period', () => {
    const L = 1.5;
    const g = 9.80665;
    const { samples } = run(pendulum(L, 2.4, g, 0.01), 1e-3, 12);
    const measured = periodFromCrossings(samples, 0);
    const expected = 2 * Math.PI * Math.sqrt(L / g);
    // Small-angle theory is itself approximate; at 0.01 rad the true period is longer by
    // about (θ₀²/16), which is ~6e-6 relative. 0.1% is a comfortable band around that.
    expect(measured).toBeCloseTo(expected, 3);
    expect(Math.abs(measured / expected - 1)).toBeLessThan(1e-3);
  });

  it('lengthens at large amplitude, as a real pendulum does', () => {
    const L = 1;
    const g = 9.80665;
    const small = periodFromCrossings(run(pendulum(L, 1, g, 0.01), 1e-3, 10).samples, 0);
    const large = periodFromCrossings(run(pendulum(L, 1, g, 2.0), 1e-3, 10).samples, 0);
    expect(large).toBeGreaterThan(small);
    // The first-order correction is T₀·(1 + θ₀²/16); at 2 rad the exact factor is ~1.34.
    expect(large / small).toBeGreaterThan(1.2);
    expect(large / small).toBeLessThan(1.5);
  });

  it('conserves energy under RK4', () => {
    const { samples } = run(pendulum(1, 1, 9.80665, 1.0), 1e-3, 20);
    const initial = samples[0]!.energy;
    const scale = Math.max(Math.abs(initial), 1);
    for (const sample of samples) {
      expect(Math.abs(sample.energy - initial) / scale).toBeLessThan(1e-8);
    }
  });

  it('leaks energy under semi-implicit Euler but stays bounded', () => {
    const rk4 = run(pendulum(1, 1, 9.80665, 1.0), 1e-3, 20, 'rk4').samples;
    const euler = run(pendulum(1, 1, 9.80665, 1.0), 1e-3, 20, 'euler').samples;

    const drift = (s: typeof rk4) =>
      Math.max(...s.map((x) => Math.abs(x.energy - s[0]!.energy)));

    // Euler is visibly worse — that is the tradeoff the setting exists to offer — but
    // being symplectic it oscillates around the true energy rather than running away.
    expect(drift(euler)).toBeGreaterThan(drift(rk4));
    expect(drift(euler)).toBeLessThan(0.05 * Math.abs(rk4[0]!.energy || 1) + 0.05);
  });
});

describe('double pendulum', () => {
  it('conserves energy through chaotic motion', () => {
    const { samples } = run(doublePendulum(9.80665), 5e-4, 20);
    const initial = samples[0]!.energy;
    for (const sample of samples) {
      expect(Math.abs(sample.energy - initial)).toBeLessThan(1e-6);
    }
  });

  it('actually moves both joints', () => {
    const { samples } = run(doublePendulum(9.80665), 5e-4, 10);
    const last = samples[samples.length - 1]!;
    expect(Math.abs(last.q[0]! - samples[0]!.q[0]!)).toBeGreaterThan(0.1);
    expect(Math.abs(last.q[1]! - samples[0]!.q[1]!)).toBeGreaterThan(0.1);
  });

  it('agrees with a reference run at a much smaller timestep', () => {
    // Chaotic systems diverge exponentially, so this is a short window on purpose: it
    // checks the integrator's order, not long-term predictability (which no integrator has
    // here, and which the tool does not claim).
    const coarse = run(doublePendulum(9.80665), 1e-3, 2).state;
    const fine = run(doublePendulum(9.80665), 2.5e-5, 2).state;
    expect(coarse.q[0]).toBeCloseTo(fine.q[0]!, 5);
    expect(coarse.q[1]).toBeCloseTo(fine.q[1]!, 5);
  });
});

describe('torque-free rigid body', () => {
  const inertia = { ixx: 1, iyy: 2, izz: 3 };

  it('conserves angular momentum and energy', () => {
    const { model, dynamics, samples, state } = run(
      freeBody(inertia, [0, 0, 0, 0.3, 0.2, 0.9]),
      1e-3,
      15,
    );
    void state;

    // Angular momentum in the world frame is the conserved quantity; in the body frame it
    // rotates, so checking the body-frame components would prove nothing.
    const momentum = (q: Float64Array, v: Float64Array): number[] => {
      updateKinematics(model, q, v, dynamics.kin);
      updateVelocities(model, dynamics.kin);
      const link = model.links[0]!;
      const w = link.v;
      const I = link.I.I;
      const body = [
        I[0]! * w[0]! + I[1]! * w[1]! + I[2]! * w[2]!,
        I[3]! * w[0]! + I[4]! * w[1]! + I[5]! * w[2]!,
        I[6]! * w[0]! + I[7]! * w[1]! + I[8]! * w[2]!,
      ];
      // Xworld maps world → link, so its transpose carries the body vector out to world.
      const e = link.Xworld.E;
      return [
        e[0]! * body[0]! + e[3]! * body[1]! + e[6]! * body[2]!,
        e[1]! * body[0]! + e[4]! * body[1]! + e[7]! * body[2]!,
        e[2]! * body[0]! + e[5]! * body[1]! + e[8]! * body[2]!,
      ];
    };

    const first = momentum(samples[0]!.q, samples[0]!.v);
    for (const sample of samples) {
      const h = momentum(sample.q, sample.v);
      for (let i = 0; i < 3; i++) expect(h[i]!).toBeCloseTo(first[i]!, 6);
      expect(Math.abs(sample.energy - samples[0]!.energy)).toBeLessThan(1e-8);
    }
  });

  it('reproduces the intermediate-axis (Dzhanibekov) flip', () => {
    // Spun almost exactly about the intermediate axis (y, with Iyy between Ixx and Izz),
    // with a small perturbation. The classical result is that this motion is unstable and
    // the body periodically flips end over end.
    const { model, dynamics, samples } = run(
      freeBody(inertia, [0, 0, 0, 0.02, 6.0, 0.0]),
      2e-4,
      12,
    );

    // Accumulated in a loop rather than with Math.max(...array): these runs are tens of
    // thousands of samples, which is well past the argument limit of a spread call.
    let maxY = -Infinity;
    let minY = Infinity;
    for (const sample of samples) {
      updateKinematics(model, sample.q, sample.v, dynamics.kin);
      updateVelocities(model, dynamics.kin);
      const wy = model.links[0]!.v[1]!;
      if (wy > maxY) maxY = wy;
      if (wy < minY) minY = wy;
    }
    // A stable spin would keep ω_y pinned near +6 for the whole run. The flip is the sign
    // reversal, and it must be a full reversal rather than a wobble.
    expect(maxY).toBeGreaterThan(5.9);
    expect(minY).toBeLessThan(-5.9);
  });

  it('spins stably about the major and minor axes', () => {
    for (const rates of [
      [0, 0, 0, 6.0, 0.02, 0], // minor axis (smallest I)
      [0, 0, 0, 0.02, 0, 6.0], // major axis (largest I)
    ]) {
      const { model, dynamics, samples } = run(freeBody(inertia, rates), 5e-4, 20);
      const axis = rates[3] === 6.0 ? 0 : 2;
      for (const sample of samples) {
        updateKinematics(model, sample.q, sample.v, dynamics.kin);
        updateVelocities(model, dynamics.kin);
        // No flip: the dominant component never even approaches zero.
        expect(model.links[0]!.v[axis]!).toBeGreaterThan(5.5);
      }
    }
  });
});

describe('prismatic joint', () => {
  it('matches the closed-form solution under a constant force', () => {
    const mass = 3.2;
    const force = 7.5;
    const spec = slider(mass);
    spec.actuators = [
      {
        name: 'Push',
        body: 0,
        kind: 'force',
        frame: 'world',
        point: [0, 0, 0],
        vector: [force, 0, 0],
        profile: () => 1,
      },
    ];
    const { samples } = run(spec, 1e-3, 4);
    const a = force / mass;
    for (const sample of samples) {
      expect(sample.q[0]!).toBeCloseTo(0.5 * a * sample.t * sample.t, 9);
      expect(sample.v[0]!).toBeCloseTo(a * sample.t, 9);
    }
  });

  it('oscillates at √(k/m) on a spring', () => {
    const mass = 2.5;
    const stiffness = 40;
    const spec = slider(mass, dofParams({ stiffness }), 0.2, 0);
    const { samples } = run(spec, 1e-4, 12);
    const measured = periodFromCrossings(samples, 0);
    expect(measured).toBeCloseTo(2 * Math.PI * Math.sqrt(mass / stiffness), 4);
  });

  it('decays toward the rest position with damping', () => {
    const spec = slider(1, dofParams({ stiffness: 30, damping: 3, rest: 0.4 }), 0, 0);
    const { samples } = run(spec, 1e-4, 20);
    expect(samples[samples.length - 1]!.q[0]!).toBeCloseTo(0.4, 4);
    expect(samples[samples.length - 1]!.v[0]!).toBeCloseTo(0, 4);
  });

  it('holds behind a travel stop', () => {
    const spec = slider(
      1,
      dofParams({ limitEnabled: true, limitLo: -1, limitHi: 0.5, limitStiffness: 5000 }),
      0,
      3,
    );
    const { samples } = run(spec, 1e-4, 6);
    // A penalty stop allows a little overshoot by construction; what matters is that it
    // bounds the travel rather than letting the body sail through.
    const maxX = Math.max(...samples.map((s) => s.q[0]!));
    expect(maxX).toBeGreaterThan(0.5);
    expect(maxX).toBeLessThan(0.62);
  });
});

describe('inertia reference toggle', () => {
  it('gives identical motion whether the tensor is about the CoM or the origin', () => {
    const com = [0.3, -0.2, 0.45];
    const mass = 4.1;
    const aboutCom = { ixx: 0.8, iyy: 1.1, izz: 1.4, ixy: 0.05, ixz: -0.03, iyz: 0.02 };

    // The same physical body, stated the other way: shift the tensor out to the origin by
    // hand and declare it as such.
    const shift = parallelAxisShift(mass, Float64Array.from(com));
    const aboutOrigin = {
      ixx: aboutCom.ixx + shift[0]!,
      iyy: aboutCom.iyy + shift[4]!,
      izz: aboutCom.izz + shift[8]!,
      // Stored products are +∫xy while the tensor carries −∫xy, hence the sign flip on
      // the shift's off-diagonals.
      ixy: aboutCom.ixy - shift[1]!,
      ixz: aboutCom.ixz - shift[2]!,
      iyz: aboutCom.iyz - shift[5]!,
    };

    const make = (inertia: typeof aboutCom, about: 'com' | 'origin') => ({
      bodies: [bodySpec({ name: 'B', mass, com, inertia, inertiaAbout: about })],
      hinges: [hingeSpec({ free: [...MASK.free], rates: [0.4, -0.2, 0.1, 0.7, 1.3, -0.5] })],
      gravity: [0, 0, -9.80665],
    });

    const a = run(make(aboutCom, 'com'), 1e-3, 3).state;
    const b = run(make(aboutOrigin, 'origin'), 1e-3, 3).state;

    for (let i = 0; i < a.q.length; i++) expect(a.q[i]!).toBeCloseTo(b.q[i]!, 12);
    for (let i = 0; i < a.v.length; i++) expect(a.v[i]!).toBeCloseTo(b.v[i]!, 12);
  });
});

describe('mass matrix', () => {
  it('agrees with inverse dynamics column by column', () => {
    const spec = doublePendulum(0); // gravity off, so C is purely velocity-dependent
    const model = buildModel(spec);
    const d = makeDynamics(model);

    const q = Float64Array.from([0.4, -0.9]);
    const v = new Float64Array(model.nv); // zero velocity isolates H from Coriolis terms

    updateKinematics(model, q, v, d.kin);
    updateVelocities(model, d.kin);
    crba(model, d.H, d.crbaScratch);

    const bias = new Float64Array(model.nv);
    rnea(model, null, null, bias, d.rneaScratch);

    const column = new Float64Array(model.nv);
    const tau = new Float64Array(model.nv);
    for (let j = 0; j < model.nv; j++) {
      column.fill(0);
      column[j] = 1;
      rnea(model, column, null, tau, d.rneaScratch);
      for (let i = 0; i < model.nv; i++) {
        expect(tau[i]! - bias[i]!).toBeCloseTo(d.H[i * model.nv + j]!, 9);
      }
    }
  });

  it('is symmetric and positive definite', () => {
    const model = buildModel(doublePendulum(9.80665));
    const d = makeDynamics(model);
    const q = Float64Array.from([0.3, 1.2]);
    const v = Float64Array.from([0.5, -0.4]);

    updateKinematics(model, q, v, d.kin);
    updateVelocities(model, d.kin);
    crba(model, d.H, d.crbaScratch);

    const n = model.nv;
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        expect(d.H[i * n + j]!).toBeCloseTo(d.H[j * n + i]!, 12);
      }
    }

    const qdd = new Float64Array(n);
    forwardDynamics(d, q, v, 0, qdd);
    expect(d.singular).toBe(false);
    for (let i = 0; i < n; i++) expect(d.factorization.D[i]!).toBeGreaterThan(0);
  });
});

describe('actuators', () => {
  it('a force at the centre of mass gives F = ma', () => {
    const spec = freeBody({ ixx: 1, iyy: 2, izz: 3 }, [0, 0, 0, 0, 0, 0]);
    spec.actuators = [
      { name: 'F', body: 0, kind: 'force', frame: 'world', point: [0, 0, 0], vector: [5, -3, 2], profile: () => 1 },
    ];
    const model = buildModel(spec);
    const d = makeDynamics(model);
    const qdd = new Float64Array(model.nv);
    forwardDynamics(d, Float64Array.from(model.q0), new Float64Array(model.nv), 0, qdd);

    // Free-flyer coordinates are [tx, ty, tz, ωx, ωy, ωz]; mass is 2 from the fixture.
    expect(qdd[0]!).toBeCloseTo(5 / 2, 10);
    expect(qdd[1]!).toBeCloseTo(-3 / 2, 10);
    expect(qdd[2]!).toBeCloseTo(2 / 2, 10);
    expect(qdd[3]!).toBeCloseTo(0, 10);
  });

  it('a moment gives τ = Iα about each principal axis', () => {
    const inertia = { ixx: 1.5, iyy: 2.5, izz: 4 };
    const spec = freeBody(inertia, [0, 0, 0, 0, 0, 0]);
    spec.actuators = [
      { name: 'M', body: 0, kind: 'moment', frame: 'body', point: [0, 0, 0], vector: [3, 5, 8], profile: () => 1 },
    ];
    const model = buildModel(spec);
    const d = makeDynamics(model);
    const qdd = new Float64Array(model.nv);
    forwardDynamics(d, Float64Array.from(model.q0), new Float64Array(model.nv), 0, qdd);

    expect(qdd[3]!).toBeCloseTo(3 / inertia.ixx, 10);
    expect(qdd[4]!).toBeCloseTo(5 / inertia.iyy, 10);
    expect(qdd[5]!).toBeCloseTo(8 / inertia.izz, 10);
    // A pure moment produces no linear acceleration.
    for (let i = 0; i < 3; i++) expect(qdd[i]!).toBeCloseTo(0, 10);
  });

  it('an off-centre force produces both acceleration and spin', () => {
    const spec = freeBody({ ixx: 1, iyy: 1, izz: 1 }, [0, 0, 0, 0, 0, 0]);
    const lever = 0.75;
    spec.actuators = [
      { name: 'F', body: 0, kind: 'force', frame: 'body', point: [lever, 0, 0], vector: [0, 0, 4], profile: () => 1 },
    ];
    const model = buildModel(spec);
    const d = makeDynamics(model);
    const qdd = new Float64Array(model.nv);
    forwardDynamics(d, Float64Array.from(model.q0), new Float64Array(model.nv), 0, qdd);

    expect(qdd[2]!).toBeCloseTo(4 / 2, 10);
    // r × F about +x with force along +z is a moment about −y.
    expect(qdd[4]!).toBeCloseTo(-lever * 4 / 1, 10);
  });

  it('distinguishes a body-fixed direction from a world-fixed one', () => {
    const build = (frame: 'body' | 'world') => {
      const spec = freeBody({ ixx: 1, iyy: 1, izz: 1 }, [0, 0, 0, 0, 0, 3]);
      spec.actuators = [
        { name: 'T', body: 0, kind: 'force', frame, point: [0, 0, 0], vector: [1, 0, 0], profile: () => 1 },
      ];
      return run(spec, 1e-3, 2).state;
    };

    const body = build('body');
    const world = build('world');
    const displacement = (s: typeof body) => Math.hypot(s.q[0]!, s.q[1]!, s.q[2]!);

    const accel = 1 / 2; // 1 N on the fixture's 2 kg
    const omega = 3;
    const t = 2;

    // Spinning at 3 rad/s about z, a world-fixed thruster accelerates in a straight line:
    // ½·a·t².
    expect(displacement(world)).toBeCloseTo(0.5 * accel * t * t, 6);

    // A body-fixed thruster sweeps its direction around with the body, so integrating
    // a·(cos ωt, sin ωt) twice gives a cycloid — most of the impulse cancels, but a
    // secular drift survives along +y:
    //     x = (a/ω²)·(1 − cos ωt),  y = (a/ω²)·(ωt − sin ωt)
    // Identical numbers, an eightfold difference in where the body ends up. That is the
    // whole reason the frame is a per-actuator choice.
    const k = accel / (omega * omega);
    expect(body.q[0]!).toBeCloseTo(k * (1 - Math.cos(omega * t)), 6);
    expect(body.q[1]!).toBeCloseTo(k * (omega * t - Math.sin(omega * t)), 6);
    expect(displacement(body)).toBeLessThan(0.4 * displacement(world));
  });

  it('follows a time profile', () => {
    const spec = slider(1);
    spec.actuators = [
      {
        name: 'Burn',
        body: 0,
        kind: 'force',
        frame: 'world',
        point: [0, 0, 0],
        // A finite burn: on for the first second only.
        profile: (t) => (t < 1 ? 1 : 0),
        vector: [2, 0, 0],
      },
    ];
    const { samples } = run(spec, 1e-4, 3);
    const atBurnEnd = samples.find((s) => s.t >= 1)!;
    const final = samples[samples.length - 1]!;

    // Accelerates to 2 m/s over the burn, then coasts.
    expect(atBurnEnd.v[0]!).toBeCloseTo(2, 3);
    expect(final.v[0]!).toBeCloseTo(2, 3);
    expect(final.q[0]!).toBeCloseTo(1 + 2 * (final.t - 1), 3);
  });
});

describe('locked degrees of freedom', () => {
  it('holds a locked axis exactly, forever', () => {
    const spec = slider(1);
    spec.hinges[0]!.values = [0.3, 0.7, -0.2, 0, 0, 0];
    spec.gravity = [0, 0, -9.80665];
    const { model, dynamics, samples, state } = run(spec, 1e-3, 5);
    void samples;

    // Only tx is free; ty and tz are locked at their entered offsets and gravity must not
    // move them by so much as a rounding error, because they are not coordinates at all.
    expect(model.nv).toBe(1);
    expect(state.q.length).toBe(1);
    updateKinematics(model, state.q, state.v, dynamics.kin);
    const pos = model.links[0]!.Xworld.r;
    expect(pos[1]!).toBeCloseTo(0.7, 15);
    expect(pos[2]!).toBeCloseTo(-0.2, 15);
  });

  it('a fully locked model has no coordinates and does not move', () => {
    const spec = slider(1);
    spec.hinges[0]!.free = [false, false, false, false, false, false];
    spec.gravity = [0, 0, -9.80665];
    const model = buildModel(spec);
    expect(model.nv).toBe(0);
    expect(model.nq).toBe(0);

    const d = makeDynamics(model);
    const scratch = makeStepScratch(model);
    const state: State = { q: new Float64Array(0), v: new Float64Array(0) };
    expect(step(d, state, 0, 1e-3, 'rk4', scratch)).toBe(true);
  });
});
