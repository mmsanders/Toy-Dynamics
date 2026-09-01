import { describe, expect, it } from 'vitest';
import { makeDynamics, forwardDynamics, totalEnergy } from './forward';
import { buildModel, type ModelSpec } from './model';
import { bodySpec, hingeSpec, MASK, slider } from './fixtures';

function acceleration(spec: ModelSpec): Float64Array {
  const model = buildModel(spec);
  const dynamics = makeDynamics(model);
  const qdd = new Float64Array(model.nv);
  forwardDynamics(dynamics, model.q0, model.v0, 0, qdd);
  return qdd;
}

describe('two-node spring-dampers', () => {
  it('restores a body toward a fixed ground-node anchor', () => {
    const spec = slider(2, undefined, 3, 0);
    spec.springDampers = [{
      name: 'Ground spring', bodyA: -1, pointA: [0, 0, 0], bodyB: 0, pointB: [0, 0, 0],
      stiffness: 8, damping: 0, restLength: 1,
    }];

    // Extension is 2, so the spring pulls with 16. A mass of 2 accelerates at −8.
    expect(acceleration(spec)[0]).toBeCloseTo(-8, 12);
  });

  it('uses relative endpoint speed for damping', () => {
    const spec = slider(2, undefined, 1, 3);
    spec.springDampers = [{
      name: 'Ground damper', bodyA: -1, pointA: [0, 0, 0], bodyB: 0, pointB: [0, 0, 0],
      stiffness: 0, damping: 4, restLength: 1,
    }];

    // The endpoint moves away at 3, so damping is 12 back toward ground: a = −12/2.
    expect(acceleration(spec)[0]).toBeCloseTo(-6, 12);
  });

  it('applies equal and opposite forces to two moving bodies', () => {
    const spec: ModelSpec = {
      bodies: [bodySpec({ name: 'A' }), bodySpec({ name: 'B' })],
      hinges: [
        hingeSpec({ name: 'A slide', child: 0, free: [...MASK.slideX] }),
        hingeSpec({ name: 'B slide', child: 1, free: [...MASK.slideX], values: [2, 0, 0, 0, 0, 0] }),
      ],
      springDampers: [{
        name: 'Coupler', bodyA: 0, pointA: [0, 0, 0], bodyB: 1, pointB: [0, 0, 0],
        stiffness: 10, damping: 0, restLength: 1,
      }],
      gravity: [0, 0, 0],
    };

    const qdd = acceleration(spec);
    expect(qdd[0]).toBeCloseTo(10, 12);
    expect(qdd[1]).toBeCloseTo(-10, 12);
    expect(qdd[0]! + qdd[1]!).toBeCloseTo(0, 12);
  });

  it('accounts for spring energy and remains finite at zero length', () => {
    const loaded = slider(1, undefined, 2, 0);
    loaded.springDampers = [{
      name: 'Energy spring', bodyA: -1, pointA: [0, 0, 0], bodyB: 0, pointB: [0, 0, 0],
      stiffness: 20, damping: 0, restLength: 1,
    }];
    const model = buildModel(loaded);
    const dynamics = makeDynamics(model);
    expect(totalEnergy(dynamics, model.q0, model.v0).potential).toBeCloseTo(10, 12);

    const collapsed = slider(1, undefined, 0, 0);
    collapsed.springDampers = [{
      name: 'Collapsed', bodyA: -1, pointA: [0, 0, 0], bodyB: 0, pointB: [0, 0, 0],
      stiffness: 100, damping: 10, restLength: 1,
    }];
    const qdd = acceleration(collapsed);
    expect(Number.isFinite(qdd[0]!)).toBe(true);
    expect(qdd[0]).toBe(0);
  });
});
