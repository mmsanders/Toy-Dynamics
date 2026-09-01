import { describe, expect, it } from 'vitest';
import { forwardDynamics, makeDynamics, totalEnergy } from './forward';
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
