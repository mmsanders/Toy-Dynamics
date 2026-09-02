import { describe, expect, it } from 'vitest';
import { decodeModel, encodeModel } from './modelLink';
import { initialModel } from '../store/defaults';
import { GROUND_ID } from '../types';
import type { ModelPersisted } from '../store/modelRepair';

/**
 * Share-link round-tripping.
 *
 * A link is the least trustworthy input the app takes — it can be truncated by a chat
 * client, hand-edited, or produced by an older version — so these check both that a healthy
 * model survives the trip intact and that a damaged one degrades to something that renders
 * rather than throwing.
 */

const snapshot = (): ModelPersisted => {
  const model = initialModel();
  return {
    bodies: model.bodies,
    bodyOrder: model.bodyOrder,
    hinges: model.hinges,
    hingeOrder: model.hingeOrder,
    actuators: model.actuators,
    actuatorOrder: model.actuatorOrder,
    springDampers: model.springDampers,
    springDamperOrder: model.springDamperOrder,
    contactSpheres: model.contactSpheres,
    contactSphereOrder: model.contactSphereOrder,
    contactPlanes: model.contactPlanes,
    contactPlaneOrder: model.contactPlaneOrder,
    settings: model.settings,
    conventions: model.conventions,
    selectedBodyId: model.selectedBodyId,
    selectedHingeId: model.selectedHingeId,
    selectedActuatorId: model.selectedActuatorId,
    selectedSpringDamperId: model.selectedSpringDamperId,
  };
};

describe('share links', () => {
  it('round-trips a model', () => {
    const original = snapshot();
    const decoded = decodeModel(encodeModel(original))!;
    expect(decoded).not.toBeNull();

    expect(decoded.bodyOrder).toHaveLength(original.bodyOrder.length);
    expect(decoded.hingeOrder).toHaveLength(original.hingeOrder.length);
    expect(decoded.actuatorOrder).toHaveLength(original.actuatorOrder.length);

    const names = (m: ModelPersisted) => m.bodyOrder.map((id) => m.bodies[id]!.name);
    expect(names(decoded)).toEqual(names(original));
  });

  it('round-trips contact geometry and materials', () => {
    const original = snapshot();
    original.contactSpheres.ball = {
      id: 'ball', name: 'Foot', bodyId: 'lower', nodeId: 'lower-tip', radius: 0.12,
      material: { stiffness: 2500, damping: 35, friction: 0.4, frictionVelocity: 0.02 }, enabled: true,
    };
    original.contactSphereOrder.push('ball');
    original.contactPlanes.floor = {
      id: 'floor', name: 'Floor', point: [0, 0, -2], normal: [0, 0, 1],
      size: 7.5, bounded: true,
      material: { stiffness: 3000, damping: 40, friction: 0.6, frictionVelocity: 0.03 }, enabled: true,
    };
    original.contactPlaneOrder.push('floor');

    const decoded = decodeModel(encodeModel(original))!;
    const sphere = decoded.contactSpheres[decoded.contactSphereOrder[0]!]!;
    const plane = decoded.contactPlanes[decoded.contactPlaneOrder[0]!]!;
    expect(decoded.bodies[sphere.bodyId]!.name).toBe('Forearm');
    expect(decoded.bodies[sphere.bodyId]!.nodes[sphere.nodeId]!.name).toBe('Tip');
    expect(sphere.radius).toBe(0.12);
    expect(sphere.material).toEqual({ stiffness: 2500, damping: 35, friction: 0.4, frictionVelocity: 0.02 });
    expect(plane.point).toEqual([0, 0, -2]);
    expect(plane.normal).toEqual([0, 0, 1]);
    expect(plane.size).toBe(7.5);
    expect(plane.bounded).toBe(true);
    expect(plane.material).toEqual({ stiffness: 3000, damping: 40, friction: 0.6, frictionVelocity: 0.03 });
  });

  it('round-trips two-node spring-damper devices', () => {
    const original = snapshot();
    original.springDampers.coupler = {
      id: 'coupler', name: 'Elbow return',
      bodyAId: GROUND_ID, nodeAId: 'ground-origin', bodyBId: 'lower', nodeBId: 'lower-tip',
      stiffness: 125, damping: 4.5, restLength: 0.8, enabled: true, color: '#fb7185',
    };
    original.springDamperOrder.push('coupler');

    const decoded = decodeModel(encodeModel(original))!;
    expect(decoded.springDamperOrder).toHaveLength(1);
    const device = decoded.springDampers[decoded.springDamperOrder[0]!]!;
    expect(decoded.bodies[device.bodyAId]!.isGround).toBe(true);
    expect(decoded.bodies[device.bodyBId]!.name).toBe('Forearm');
    expect(decoded.bodies[device.bodyBId]!.nodes[device.nodeBId]!.name).toBe('Tip');
    expect(device).toMatchObject({ stiffness: 125, damping: 4.5, restLength: 0.8 });
  });

  it('preserves mass properties exactly', () => {
    const original = snapshot();
    const decoded = decodeModel(encodeModel(original))!;

    const upper = decoded.bodyOrder.map((id) => decoded.bodies[id]!).find((b) => b.name === 'Upper Arm')!;
    expect(upper.mass).toBe(original.bodies.upper!.mass);
    expect(upper.inertia.about).toBe(original.bodies.upper!.inertia.about);
    expect(upper.inertia.ixx).toBeCloseTo(original.bodies.upper!.inertia.ixx, 6);
    expect(upper.nodeOrder).toHaveLength(original.bodies.upper!.nodeOrder.length);
  });

  it('keeps the topology and the origin / centre-of-mass designations', () => {
    const original = snapshot();
    const decoded = decodeModel(encodeModel(original))!;

    // Ground stays index 0 and stays ground.
    expect(decoded.bodies[GROUND_ID]?.isGround).toBe(true);

    const upper = Object.values(decoded.bodies).find((b) => b.name === 'Upper Arm')!;
    expect(upper.nodes[upper.originNodeId]!.name).toBe('Root');
    expect(upper.nodes[upper.comNodeId]!.name).toBe('CoM');

    // The chain survives: one hinge hangs off ground, the other off the upper arm.
    const hinges = Object.values(decoded.hinges);
    expect(hinges.filter((h) => h.parentBodyId === GROUND_ID)).toHaveLength(1);
    const forearm = Object.values(decoded.bodies).find((b) => b.name === 'Forearm')!;
    const elbow = hinges.find((h) => h.childBodyId === forearm.id)!;
    expect(decoded.bodies[elbow.parentBodyId]!.name).toBe('Upper Arm');
  });

  it('preserves every degree-of-freedom setting', () => {
    const original = snapshot();
    original.hinges.elbow!.dof[4] = {
      free: true,
      q0: -1.1,
      u0: 0.35,
      stiffness: 12.5,
      rest: 0.2,
      damping: 0.4,
      friction: 0.15,
      stiction: 0.9,
      limit: { enabled: true, lo: -1.5, hi: 0.8, stiffness: 2500 },
    };

    const decoded = decodeModel(encodeModel(original))!;
    const forearm = Object.values(decoded.bodies).find((b) => b.name === 'Forearm')!;
    const elbow = Object.values(decoded.hinges).find((h) => h.childBodyId === forearm.id)!;

    expect(elbow.dof[4]).toEqual(original.hinges.elbow!.dof[4]);
    // And the locked axes stay locked.
    expect(elbow.dof.filter((d) => d.free)).toHaveLength(1);
  });

  it('round-trips every actuator profile shape', () => {
    const profiles = [
      { kind: 'constant' as const },
      { kind: 'step' as const, tOn: 0.5, tOff: 2.25 },
      { kind: 'ramp' as const, t0: 0, t1: 3, from: -1, to: 2.5 },
      { kind: 'sine' as const, frequency: 2.5, phase: 0.75, offset: -0.25 },
      { kind: 'impulse' as const, t0: 1.5, width: 0.02 },
      { kind: 'expr' as const, source: '2*pulse(t, 1, 3) + sin(10*t)' },
    ];

    for (const profile of profiles) {
      const original = snapshot();
      original.actuators.drive!.profile = profile;
      const decoded = decodeModel(encodeModel(original))!;
      const actuator = Object.values(decoded.actuators)[0]!;
      expect(actuator.profile).toEqual(profile);
    }
  });

  it('preserves settings and conventions', () => {
    const original = snapshot();
    original.settings = {
      units: 'imperial',
      gravity: [0, 0, -32.174],
      dt: 0.0025,
      duration: 12.5,
      integrator: 'rk2',
      sampleRate: 120,
    };
    original.conventions = {
      upAxis: 'Y',
      eulerOrder: 'XYZ',
      rotationMode: 'extrinsic',
      angleUnit: 'rad',
    };

    const decoded = decodeModel(encodeModel(original))!;
    expect(decoded.settings).toEqual(original.settings);
    expect(decoded.conventions).toEqual(original.conventions);
  });

  it('produces a link short enough to paste into a chat app', () => {
    // A few hundred characters for a typical model, well inside every practical URL limit.
    expect(encodeModel(snapshot()).length).toBeLessThan(2000);
  });

  it('survives names in any language', () => {
    const original = snapshot();
    original.bodies.upper!.name = 'アーム · Ω';
    const decoded = decodeModel(encodeModel(original))!;
    expect(Object.values(decoded.bodies).map((b) => b.name)).toContain('アーム · Ω');
  });

  it('rejects nonsense instead of throwing', () => {
    expect(decodeModel('')).toBeNull();
    expect(decodeModel('not-base64!!')).toBeNull();
    expect(decodeModel(btoa('{"nope":1}'))).toBeNull();
    // A future format version is rejected rather than half-read.
    expect(decodeModel(btoa(JSON.stringify({ v: 99, b: [] })))).toBeNull();
  });

  it('degrades a truncated link to a model that still renders', () => {
    const encoded = encodeModel(snapshot());
    // Chat clients cut long links; whatever comes back must not be a blank screen.
    for (const fraction of [0.9, 0.7, 0.5, 0.25]) {
      const truncated = encoded.slice(0, Math.floor(encoded.length * fraction));
      const decoded = decodeModel(truncated);
      // Either cleanly rejected, or repaired into something with a ground body.
      if (decoded) expect(decoded.bodies[GROUND_ID]).toBeDefined();
    }
  });

  it('repairs a payload whose hinges point at bodies that are not there', () => {
    const decoded = decodeModel(
      btoa(
        JSON.stringify({
          v: 1,
          b: [['Ground', [['Origin', 0, 0, 0, 0, 0, 0, 1]], 0, 0, 0, [0, 0, 0, 0, 0, 0, 0], '#888', 1]],
          h: [['Dangler', 9, 9, 9, 9, [0, 0, 0, 1], []]],
          a: [],
          s: ['si', 0, 0, -9.81, 0.001, 10, 'rk4', 60],
          c: ['Z', 'ZYX', 'intrinsic', 'deg'],
        }),
      ),
    );
    expect(decoded).not.toBeNull();
    expect(decoded!.bodies[GROUND_ID]).toBeDefined();
  });
});
