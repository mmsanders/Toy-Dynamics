import { describe, expect, it } from 'vitest';
import {
  queryHeightfieldSurface,
  queryPlaneSurface,
  sampleHeightfield,
  type HeightfieldGeometry,
  type SurfaceQueryResult,
} from './contact';
import { buildModel, type ContactMaterialSpec } from './model';
import { forwardDynamics, makeDynamics } from './forward';
import { makeStepScratch, step } from './integrate';
import { bodySpec, hingeSpec } from './fixtures';
import { planeBasis } from './contact';
import { v3 } from './spatial';

const result = (): SurfaceQueryResult => ({
  separation: 0,
  normal: v3(),
  point: v3(),
  velocity: v3(),
});

const grid = (
  columns: number,
  rows: number,
  heights: number[],
  origin = v3(-1, -1, 0),
  spacing = 1,
): HeightfieldGeometry => ({ origin, spacing, columns, rows, heights: Float64Array.from(heights) });

describe('regular heightfield contact', () => {
  it('matches an analytical plane for a flat grid', () => {
    const field = grid(3, 3, Array(9).fill(0));
    const heightfield = result();
    expect(queryHeightfieldSurface(field, 0.25, -0.4, -0.1, 0, heightfield)).toBe(true);

    const normal = v3(0, 0, 1);
    const tangentU = v3();
    const tangentV = v3();
    planeBasis(normal, tangentU, tangentV);
    const plane = result();
    queryPlaneSurface({ point: v3(0, 0, 0), normal, tangentU, tangentV, halfSize: 0, bounded: false }, 0.25, -0.4, -0.1, 0, plane);

    expect(heightfield.separation).toBeCloseTo(plane.separation, 12);
    expect(heightfield.normal[0]).toBeCloseTo(plane.normal[0]!, 12);
    expect(heightfield.normal[1]).toBeCloseTo(plane.normal[1]!, 12);
    expect(heightfield.normal[2]).toBeCloseTo(plane.normal[2]!, 12);
    expect([...heightfield.velocity]).toEqual([0, 0, 0]);
  });

  it('matches the normal and separation of a constant-slope plane', () => {
    const slopeX = 0.25;
    const slopeY = -0.1;
    const heights = Array.from({ length: 9 }, (_, index) => {
      const x = (index % 3) - 1;
      const y = Math.floor(index / 3) - 1;
      return slopeX * x + slopeY * y;
    });
    const field = grid(3, 3, heights);
    const sampled = result();
    expect(queryHeightfieldSurface(field, 0.2, 0.3, 0.7, 0.15, sampled)).toBe(true);

    const normal = v3(-slopeX, -slopeY, 1);
    const length = Math.hypot(...normal);
    normal[0] = normal[0]! / length;
    normal[1] = normal[1]! / length;
    normal[2] = normal[2]! / length;
    const tangentU = v3();
    const tangentV = v3();
    planeBasis(normal, tangentU, tangentV);
    const analytical = result();
    queryPlaneSurface({ point: v3(0, 0, 0), normal, tangentU, tangentV, halfSize: 0, bounded: false }, 0.2, 0.3, 0.7, 0.15, analytical);

    expect([...sampled.normal]).toEqual(expect.arrayContaining([
      expect.closeTo(analytical.normal[0]!, 12),
      expect.closeTo(analytical.normal[1]!, 12),
      expect.closeTo(analytical.normal[2]!, 12),
    ]));
    expect(sampled.separation).toBeCloseTo(analytical.separation, 12);
  });

  it('is height-continuous at cell boundaries and includes the outer edge', () => {
    const field = grid(4, 2, [0, 1, 1.5, 3, 0, 1, 1.5, 3], v3(0, 0, 0));
    const left = { height: 0, dx: 0, dy: 0 };
    const right = { height: 0, dx: 0, dy: 0 };
    expect(sampleHeightfield(field, 1 - 1e-9, 0.5, left)).toBe(true);
    expect(sampleHeightfield(field, 1 + 1e-9, 0.5, right)).toBe(true);
    expect(Math.abs(left.height - right.height)).toBeLessThan(2e-9);
    expect(sampleHeightfield(field, 3, 1, right)).toBe(true);
    expect(right.height).toBeCloseTo(3, 12);
  });

  it('returns no contact outside the footprint or across no-data', () => {
    const field = grid(3, 3, [0, 0, 0, 0, Number.NaN, 0, 0, 0, 0]);
    expect(queryHeightfieldSurface(field, -1.01, 0, 0, 0, result())).toBe(false);
    expect(queryHeightfieldSurface(field, 1.01, 0, 0, 0, result())).toBe(false);
    expect(queryHeightfieldSurface(field, -0.5, -0.5, 0, 0, result())).toBe(false);
  });

  it('supports a sphere traversing a smooth procedural hill', () => {
    const columns = 41;
    const rows = 2;
    const spacing = 0.1;
    const hill = (x: number) => 0.3 * Math.exp(-(x * x) / 0.32);
    const heights = Array.from({ length: columns * rows }, (_, index) => {
      const x = -2 + (index % columns) * spacing;
      return hill(x);
    });
    const material: ContactMaterialSpec = {
      stiffness: 20000,
      damping: 120,
      friction: 0,
      frictionVelocity: 0.01,
    };
    const radius = 0.1;
    const gravity = 9.81;
    const model = buildModel({
      bodies: [bodySpec()],
      hinges: [hingeSpec({
        free: [true, false, true, false, false, false],
        values: [-1.5, 0, radius + hill(-1.5) - gravity / material.stiffness, 0, 0, 0],
        rates: [3, 0, 0, 0, 0, 0],
      })],
      contactSpheres: [{ name: 'Ball', body: 0, point: [0, 0, 0], radius, material }],
      contactHeightfields: [{
        name: 'Hill', origin: [-2, -0.05, 0], spacing, columns, rows, heights, material,
      }],
      gravity: [0, 0, -gravity],
    });
    const dynamics = makeDynamics(model);
    const initialAcceleration = new Float64Array(model.nv);
    forwardDynamics(dynamics, model.q0, model.v0, 0, initialAcceleration, 0.001);
    expect(initialAcceleration.every(Number.isFinite)).toBe(true);

    const state = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };
    const scratch = makeStepScratch(model);
    let highest = state.q[1]!;
    const dt = 0.0005;
    for (let i = 0; i < 2 / dt; i++) {
      expect(step(dynamics, state, i * dt, dt, 'rk4', scratch)).toBe(true);
      highest = Math.max(highest, state.q[1]!);
    }
    expect(state.q[0]).toBeGreaterThan(1);
    expect(highest).toBeGreaterThan(0.25);
    expect([...state.q, ...state.v].every(Number.isFinite)).toBe(true);
  });
});
