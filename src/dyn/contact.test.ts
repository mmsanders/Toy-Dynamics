import { describe, expect, it } from 'vitest';
import { forwardDynamics, makeDynamics, totalEnergy } from './forward';
import { makeStepScratch, step } from './integrate';
import { bodySpec, hingeSpec, MASK } from './fixtures';
import { buildModel, type ContactMaterialSpec, type ModelSpec } from './model';

const material: ContactMaterialSpec = { stiffness: 100, damping: 10 };

function acceleration(spec: ModelSpec, settleDt = 1e-3): Float64Array {
  const model = buildModel(spec);
  const dynamics = makeDynamics(model);
  const out = new Float64Array(model.nv);
  forwardDynamics(dynamics, model.q0, model.v0, 0, out, settleDt);
  return out;
}

describe('analytical compliant contact', () => {
  it('includes compliant spring energy in total mechanical energy', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec()],
      hinges: [hingeSpec({ free: [...MASK.slideX], values: [-0.1, 0, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Point', body: 0, point: [0, 0, 0], radius: 0, material }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material }],
      gravity: [0, 0, 0],
    };
    const model = buildModel(spec);
    const dynamics = makeDynamics(model);
    const energy = totalEnergy(dynamics, model.q0, model.v0);
    expect(energy.kinetic).toBe(0);
    expect(energy.potential).toBeCloseTo(0.5, 12);
    expect(energy.total).toBeCloseTo(0.5, 12);
  });
  it('applies the sphere-plane penalty force and closing-speed damping', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec({ mass: 2 })],
      hinges: [hingeSpec({ free: [...MASK.slideX], values: [-0.1, 0, 0, 0, 0, 0], rates: [-0.5, 0, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Point', body: 0, point: [0, 0, 0], radius: 0, material }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material }],
      gravity: [0, 0, 0],
    };

    // (k * penetration + c * closing speed) / mass = (10 + 5) / 2.
    expect(acceleration(spec)[0]).toBeCloseTo(7.5, 12);
  });

  it('supports static gravity equilibrium at the expected mg/k penetration', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec({ mass: 2 })],
      // For m=2, |g|=10, and k=100, equilibrium penetration is mg/k=0.2.
      hinges: [hingeSpec({ free: [...MASK.slideX], values: [-0.2, 0, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Point', body: 0, point: [0, 0, 0], radius: 0, material }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material }],
      gravity: [-10, 0, 0],
    };

    expect(acceleration(spec)[0]).toBeCloseTo(0, 12);
  });

  it('never lets contact damping attract a separating sphere', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec()],
      hinges: [hingeSpec({ free: [...MASK.slideX], values: [-0.01, 0, 0, 0, 0, 0], rates: [10, 0, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Point', body: 0, point: [0, 0, 0], radius: 0, material }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material }],
      gravity: [0, 0, 0],
    };

    expect(acceleration(spec)[0]).toBeCloseTo(1, 12);
  });

  it('opposes tangential slip with bounded regularized friction', () => {
    const frictionMaterial: ContactMaterialSpec = {
      stiffness: 100, damping: 0, friction: 0.5, frictionVelocity: 0.01,
    };
    const spec: ModelSpec = {
      bodies: [bodySpec()],
      hinges: [hingeSpec({
        free: [...MASK.planar], values: [-0.1, 0, 0, 0, 0, 0], rates: [0, 2, 0, 0, 0, 0],
      })],
      contactSpheres: [{ name: 'Slider', body: 0, point: [0, 0, 0], radius: 0, material: frictionMaterial }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material: frictionMaterial }],
      gravity: [0, 0, 0],
    };

    const qdd = acceleration(spec);
    expect(qdd[0]).toBeCloseTo(10, 12);
    expect(qdd[1]).toBeCloseTo(-5, 12);
  });

  it('slows sustained sliding monotonically without injecting energy', () => {
    const frictionMaterial: ContactMaterialSpec = {
      stiffness: 100, damping: 0, friction: 0.5, frictionVelocity: 0.01,
    };
    const model = buildModel({
      bodies: [bodySpec()],
      hinges: [hingeSpec({
        free: [...MASK.planar], values: [-0.1, 0, 0, 0, 0, 0], rates: [0, 1, 0, 0, 0, 0],
      })],
      contactSpheres: [{ name: 'Slider', body: 0, point: [0, 0, 0], radius: 0, material: frictionMaterial }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material: frictionMaterial }],
      gravity: [-10, 0, 0],
    });
    const dynamics = makeDynamics(model);
    const state = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };
    const scratch = makeStepScratch(model);
    let previousSpeed = Math.abs(state.v[1]!);
    let previousEnergy = totalEnergy(dynamics, state.q, state.v).total;

    for (let i = 0; i < 150; i++) {
      expect(step(dynamics, state, i * 0.001, 0.001, 'rk4', scratch)).toBe(true);
      const speed = Math.abs(state.v[1]!);
      const energy = totalEnergy(dynamics, state.q, state.v).total;
      expect(speed).toBeLessThanOrEqual(previousSpeed + 1e-12);
      expect(energy).toBeLessThanOrEqual(previousEnergy + 1e-10);
      previousSpeed = speed;
      previousEnergy = energy;
    }
    expect(previousSpeed).toBeLessThan(0.3);
  });

  it('accumulates forces from multiple planes', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec()],
      hinges: [hingeSpec({ free: [...MASK.planar], values: [-0.1, -0.2, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Ball', body: 0, point: [0, 0, 0], radius: 0, material }],
      contactPlanes: [
        { name: 'X wall', point: [0, 0, 0], normal: [1, 0, 0], material },
        { name: 'Y wall', point: [0, 0, 0], normal: [0, 1, 0], material },
      ],
      gravity: [0, 0, 0],
    };

    const qdd = acceleration(spec);
    expect(qdd[0]).toBeCloseTo(10, 12);
    expect(qdd[1]).toBeCloseTo(20, 12);
  });

  it('turns an off-centre normal force into the expected moment', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec({ inertia: { ixx: 1, iyy: 1, izz: 1, ixy: 0, ixz: 0, iyz: 0 } })],
      hinges: [hingeSpec({ free: [...MASK.free], values: [-0.1, 0, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Offset point', body: 0, point: [0, 1, 0], radius: 0, material }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material }],
      gravity: [0, 0, 0],
    };

    const qdd = acceleration(spec);
    expect(qdd[0]).toBeCloseTo(10, 12);
    expect(qdd[5]).toBeCloseTo(-10, 12);
  });

  it('applies equal and opposite sphere-sphere forces', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec(), bodySpec()],
      hinges: [
        hingeSpec({ name: 'A', child: 0, free: [...MASK.slideX], values: [-0.4, 0, 0, 0, 0, 0] }),
        hingeSpec({ name: 'B', child: 1, free: [...MASK.slideX], values: [0.4, 0, 0, 0, 0, 0] }),
      ],
      contactSpheres: [
        { name: 'A sphere', body: 0, point: [0, 0, 0], radius: 0.5, material },
        { name: 'B sphere', body: 1, point: [0, 0, 0], radius: 0.5, material },
      ],
      gravity: [0, 0, 0],
    };

    const qdd = acceleration(spec);
    expect(qdd[0]).toBeCloseTo(-20, 12);
    expect(qdd[1]).toBeCloseTo(20, 12);
    expect(qdd[0]! + qdd[1]!).toBeCloseTo(0, 12);
  });

  // A finite-radius sphere is where friction stops being a block on a plane: the force acts
  // at the surface, a radius from the centre, and that lever arm is what makes a ball roll.
  // Both checks below have textbook answers that depend on the moment being there.
  const RADIUS = 0.1, MASS = 1, STIFFNESS = 1e4, GRAVITY = 9.81;
  const SOLID_SPHERE = 0.4 * MASS * RADIUS * RADIUS;
  const rollingMaterial: ContactMaterialSpec = { stiffness: STIFFNESS, damping: 50, friction: 0.5, frictionVelocity: 0.01 };
  const ballOnFloor = (rates: number[]): ModelSpec => ({
    bodies: [bodySpec({ mass: MASS, inertia: { ixx: SOLID_SPHERE, iyy: SOLID_SPHERE, izz: SOLID_SPHERE, ixy: 0, ixz: 0, iyz: 0 } })],
    hinges: [hingeSpec({
      free: [...MASK.free],
      // Resting at the static equilibrium penetration, so nothing bounces.
      values: [0, 0, RADIUS - MASS * GRAVITY / STIFFNESS, 0, 0, 0],
      rates,
    })],
    contactSpheres: [{ name: 'Ball', body: 0, point: [0, 0, 0], radius: RADIUS, material: rollingMaterial }],
    contactPlanes: [{ name: 'Floor', point: [0, 0, 0], normal: [0, 0, 1], material: rollingMaterial }],
    gravity: [0, 0, -GRAVITY],
  });
  const settle = (spec: ModelSpec, seconds: number) => {
    const model = buildModel(spec);
    const dynamics = makeDynamics(model);
    const state = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };
    const scratch = makeStepScratch(model);
    const dt = 1e-4;
    for (let i = 0; i < seconds / dt; i++) expect(step(dynamics, state, i * dt, dt, 'rk4', scratch)).toBe(true);
    return state.v;
  };

  it('lets a sliding solid sphere spin up and roll at 5/7 of its launch speed', () => {
    // Momentum about the contact point is conserved while it slips: m·v₀·r = (m r² + I)·ω,
    // so a solid sphere rolls away at v = v₀ / (1 + 2/5) = 5/7 v₀ with ω = v / r.
    const v = settle(ballOnFloor([1, 0, 0, 0, 0, 0]), 0.5);
    expect(v[0]).toBeCloseTo(5 / 7, 3);
    expect(v[4]! * RADIUS).toBeCloseTo(v[0]!, 3);
  });

  it('lets a spinning sphere dropped on a floor pull itself into motion', () => {
    // The mirror image: spin-only launches translation at v = ω₀ r / (1 + 5/2) = 2/7 ω₀ r.
    const omega = 10;
    const v = settle(ballOnFloor([0, 0, 0, 0, omega, 0]), 0.5);
    expect(v[0]).toBeCloseTo((2 / 7) * omega * RADIUS, 3);
    expect(v[4]! * RADIUS).toBeCloseTo(v[0]!, 3);
  });

  it('applies sphere-sphere friction at the surface, with the spin it implies', () => {
    const frictionMaterial: ContactMaterialSpec = { stiffness: 100, damping: 0, friction: 0.5, frictionVelocity: 0.01 };
    const spec: ModelSpec = {
      bodies: [bodySpec(), bodySpec()],
      hinges: [
        hingeSpec({ name: 'A', child: 0, free: [...MASK.free], values: [-0.4, 0, 0, 0, 0, 0], rates: [0, 0, 0, 0, 0, 1] }),
        hingeSpec({ name: 'B', child: 1, free: [...MASK.free], values: [0.4, 0, 0, 0, 0, 0] }),
      ],
      contactSpheres: [
        { name: 'A', body: 0, point: [0, 0, 0], radius: 0.5, material: frictionMaterial },
        { name: 'B', body: 1, point: [0, 0, 0], radius: 0.5, material: frictionMaterial },
      ],
      gravity: [0, 0, 0],
    };
    // A spins about z, so its surface point at the contact moves +y at ω r = 0.5. The
    // normal load is k·δ = 100·0.2 = 20 and the slip is far past the regularization scale,
    // so friction is the full μN = 10: B is dragged along +y, A pushed back, and each feels
    // the moment r × f = −5 about z.
    const qdd = acceleration(spec);
    expect(qdd[1]).toBeCloseTo(-10, 9);
    expect(qdd[7]).toBeCloseTo(10, 9);
    expect(qdd[5]).toBeCloseTo(-5, 9);
    expect(qdd[11]).toBeCloseTo(-5, 9);
    expect(qdd[0]).toBeCloseTo(-20, 9);
    expect(qdd[6]).toBeCloseTo(20, 9);
  });

  // A bounded plate is a square patch centred on its point. For a +Z normal the shared
  // basis is the ordinary X/Y pair, so a plate of size 2 covers x, y in [-1, 1].
  const plate = (bounded: boolean, centre: number[], radius = 0.5): ModelSpec => ({
    bodies: [bodySpec()],
    hinges: [hingeSpec({ free: [...MASK.free], values: [centre[0]!, centre[1]!, centre[2]!, 0, 0, 0] })],
    contactSpheres: [{ name: 'Ball', body: 0, point: [0, 0, 0], radius, material }],
    contactPlanes: [{ name: 'Plate', point: [0, 0, 0], normal: [0, 0, 1], size: 2, bounded, material }],
    gravity: [0, 0, 0],
  });

  it('is unchanged by bounding a plane while the sphere is over it', () => {
    const over = [0.4, -0.2, 0.3];
    const bounded = acceleration(plate(true, over));
    const unbounded = acceleration(plate(false, over));
    expect(bounded[2]).toBeCloseTo(20, 12);
    expect([...bounded]).toEqual([...unbounded]);
  });

  it('lets a sphere past the edge of a plate fall through where it has ended', () => {
    // Far enough past the rim that no part of the sphere is over the plate any more.
    const past = [1.8, 0, 0.3];
    expect(acceleration(plate(true, past))[2]).toBe(0);
    // The same sphere against an unbounded plane is still firmly in contact, which is the
    // whole difference between the two.
    expect(acceleration(plate(false, past))[2]).toBeCloseTo(20, 12);
  });

  it('pushes a sphere overhanging an edge outwards as well as up', () => {
    // Centre 0.3 past the rim and 0.3 above the face, so the nearest point of the plate is
    // the rim itself and the normal is the diagonal between the two.
    const qdd = acceleration(plate(true, [1.3, 0, 0.3]));
    const reach = Math.hypot(0.3, 0.3);
    const push = material.stiffness * (0.5 - reach) / Math.SQRT2;
    expect(qdd[0]).toBeCloseTo(push, 9);
    expect(qdd[2]).toBeCloseTo(push, 9);
    // The force still runs through the centre, so an overhang alone imparts no spin.
    expect(qdd[4]).toBeCloseTo(0, 12);
  });

  it('rolls a ball off the end of a plate and drops it', () => {
    const plateMaterial: ContactMaterialSpec = { stiffness: STIFFNESS, damping: 50, friction: 0.5, frictionVelocity: 0.01 };
    const model = buildModel({
      bodies: [bodySpec({ mass: MASS, inertia: { ixx: SOLID_SPHERE, iyy: SOLID_SPHERE, izz: SOLID_SPHERE, ixy: 0, ixz: 0, iyz: 0 } })],
      hinges: [hingeSpec({
        free: [...MASK.free],
        values: [0.2, 0, RADIUS - MASS * GRAVITY / STIFFNESS, 0, 0, 0],
        rates: [1, 0, 0, 0, 0, 0],
      })],
      contactSpheres: [{ name: 'Ball', body: 0, point: [0, 0, 0], radius: RADIUS, material: plateMaterial }],
      contactPlanes: [{ name: 'Plate', point: [0, 0, 0], normal: [0, 0, 1], size: 2, bounded: true, material: plateMaterial }],
      gravity: [0, 0, -GRAVITY],
    });
    const dynamics = makeDynamics(model);
    const state = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };
    const scratch = makeStepScratch(model);
    const dt = 1e-4;

    let lowestWhileOn = Infinity;
    for (let i = 0; i < 1.6 / dt; i++) {
      expect(step(dynamics, state, i * dt, dt, 'rk4', scratch)).toBe(true);
      // While it is still on the plate it stays on top of it, to within the penetration a
      // compliant contact allows.
      if (state.q[0]! < 1 - RADIUS) lowestWhileOn = Math.min(lowestWhileOn, state.q[2]!);
    }
    expect(lowestWhileOn).toBeGreaterThan(RADIUS - 10 * MASS * GRAVITY / STIFFNESS);

    // Past the end there is nothing left to hold it up.
    expect(state.q[0]).toBeGreaterThan(1.2);
    expect(state.q[2]).toBeLessThan(-0.5);
    expect(state.v[2]).toBeLessThan(-1);
  });

  it('freezes contact activation across Runge-Kutta stages', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec()],
      hinges: [hingeSpec({ free: [...MASK.slideX], values: [1, 0, 0, 0, 0, 0] })],
      contactSpheres: [{ name: 'Point', body: 0, point: [0, 0, 0], radius: 0, material }],
      contactPlanes: [{ name: 'Wall', point: [0, 0, 0], normal: [1, 0, 0], material }],
      gravity: [0, 0, 0],
    };
    const model = buildModel(spec);
    const dynamics = makeDynamics(model);
    const qdd = new Float64Array(model.nv);
    forwardDynamics(dynamics, model.q0, model.v0, 0, qdd, 1e-3);
    expect(qdd[0]).toBe(0);

    model.q0[0] = -0.1;
    forwardDynamics(dynamics, model.q0, model.v0, 0, qdd, 0);
    expect(qdd[0]).toBe(0);

    forwardDynamics(dynamics, model.q0, model.v0, 0, qdd, 1e-3);
    expect(qdd[0]).toBeCloseTo(10, 12);
  });
});
