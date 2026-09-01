import { beforeEach, describe, expect, it } from 'vitest';
import { useModelStore } from '../store/useModelStore';
import { initialModel, neutralDof, neutralDofSet } from '../store/defaults';
import { repairModel } from '../store/modelRepair';
import { runDiagnostics } from './diagnostics';
import { buildSpec, compileProfile } from './adapter';
import { ancestorsOf, orderHingeIds, wouldCreateCycle } from './topology';
import { jointPose } from '../math/transforms';
import { jcalc, makeJointModel, makeJointWorkspace, writeInitialQ } from '../dyn/joints';
import { m3FromQuat } from '../dyn/spatial';
import { buildModel } from '../dyn/model';
import { makeDynamics } from '../dyn/forward';
import { makeStepScratch, step, type State } from '../dyn/integrate';
import { gravityOnSystemChange, STANDARD_GRAVITY_IMPERIAL, STANDARD_GRAVITY_SI } from '../units';
import { GROUND_ID, type Vec3 } from '../types';

const reset = () => useModelStore.setState({ ...initialModel() }, false);

beforeEach(reset);

describe('conventions agree between the two implementations', () => {
  it('jointPose matches the solver joint transform', () => {
    // The store and the scene compose poses with three.js; the solver builds the same
    // transform from scratch inside jcalc. Two implementations of one convention is a real
    // risk, so they are pinned against each other here.
    const values = [0.4, -0.25, 0.6, 0.3, -0.8, 1.2];
    const dof = neutralDofSet().map((d, i) => ({ ...d, free: true, q0: values[i]! }));

    const joint = makeJointModel(dof.map((d) => d.free), values, 0, 0);
    const q = new Float64Array(joint.nq);
    writeInitialQ(joint, values, q);
    const w = makeJointWorkspace(joint);
    jcalc(joint, q, new Float64Array(joint.nv), w);

    const pose = jointPose(dof);
    const R = m3FromQuat(pose.orientation);

    // jcalc stores Rᵀ (parent → child); jointPose gives R (child → parent).
    for (let row = 0; row < 3; row++) {
      for (let col = 0; col < 3; col++) {
        expect(w.XJ.E[row * 3 + col]!).toBeCloseTo(R[col * 3 + row]!, 12);
      }
    }
    for (let i = 0; i < 3; i++) expect(w.XJ.r[i]!).toBeCloseTo(pose.position[i]!, 12);
  });
});

describe('topology', () => {
  it('walks ancestors up to ground', () => {
    const { hinges } = useModelStore.getState();
    expect(ancestorsOf(hinges, 'lower')).toEqual(['upper', GROUND_ID]);
    expect(ancestorsOf(hinges, 'upper')).toEqual([GROUND_ID]);
    expect(ancestorsOf(hinges, GROUND_ID)).toEqual([]);
  });

  it('refuses a re-parenting that would close a loop', () => {
    const { hinges } = useModelStore.getState();
    expect(wouldCreateCycle(hinges, 'upper', 'lower')).toBe(true);
    expect(wouldCreateCycle(hinges, 'upper', 'upper')).toBe(true);
    expect(wouldCreateCycle(hinges, 'lower', GROUND_ID)).toBe(false);
  });

  it('orders hinges parent before child', () => {
    const { ordered, cycle } = orderHingeIds(useModelStore.getState().hinges);
    expect(cycle).toBeNull();
    expect(ordered.indexOf('shoulder')).toBeLessThan(ordered.indexOf('elbow'));
  });

  it('the store blocks a cyclic re-parent rather than accepting it', () => {
    useModelStore.getState().setHingeParent('shoulder', 'lower');
    expect(useModelStore.getState().hinges.shoulder!.parentBodyId).toBe(GROUND_ID);
  });
});

describe('origin node', () => {
  it('re-expresses every node so the body does not move', () => {
    const before = useModelStore.getState().bodies.upper!;
    const tipBefore = before.nodes['upper-tip']!.position;
    const rootBefore = before.nodes['upper-root']!.position;

    useModelStore.getState().setOriginNode('upper', 'upper-tip');
    const after = useModelStore.getState().bodies.upper!;

    // The new origin sits at zero, and every separation between nodes is unchanged.
    expect(after.originNodeId).toBe('upper-tip');
    expect(after.nodes['upper-tip']!.position).toEqual([0, 0, 0]);
    for (let i = 0; i < 3; i++) {
      expect(after.nodes['upper-root']!.position[i]!).toBeCloseTo(rootBefore[i]! - tipBefore[i]!, 12);
    }
  });

  it('re-references an origin-stated inertia so the physical body is unchanged', () => {
    // State the inertia about the origin, then move the origin. The tensor must follow,
    // because the body has not changed — only which point we are measuring from.
    useModelStore.getState().setInertiaReference('upper', 'origin');
    useModelStore.getState().setInertia('upper', { ixx: 1.44, iyy: 1.44, izz: 0.02 });
    const before = useModelStore.getState().bodies.upper!.inertia;

    useModelStore.getState().setOriginNode('upper', 'upper-tip');
    const after = useModelStore.getState().bodies.upper!.inertia;

    // CoM was 0.6 below the old origin and is 0.6 above the new one, so the parallel-axis
    // term is identical and the tensor comes back unchanged. A different offset would
    // change it; what must never happen is the number staying put while its meaning moves.
    expect(after.ixx).toBeCloseTo(before.ixx, 10);
    expect(after.izz).toBeCloseTo(before.izz, 10);
  });

  it('leaves a CoM-stated inertia alone, since it does not depend on the origin', () => {
    const before = { ...useModelStore.getState().bodies.upper!.inertia };
    useModelStore.getState().setOriginNode('upper', 'upper-tip');
    expect(useModelStore.getState().bodies.upper!.inertia).toEqual(before);
  });
});

describe('inertia reference toggle', () => {
  it('reinterpreting keeps the numbers and changes the physics', () => {
    const before = { ...useModelStore.getState().bodies.upper!.inertia };
    useModelStore.getState().setInertiaReference('upper', 'origin');
    const after = useModelStore.getState().bodies.upper!.inertia;
    expect(after.about).toBe('origin');
    expect(after.ixx).toBe(before.ixx);
    expect(after.iyy).toBe(before.iyy);
  });

  it('converting rewrites the numbers and keeps the physics', () => {
    const body = useModelStore.getState().bodies.upper!;
    const comZ = body.nodes[body.comNodeId]!.position[2];
    const before = { ...body.inertia };

    useModelStore.getState().convertInertiaReference('upper', 'origin');
    const after = useModelStore.getState().bodies.upper!.inertia;

    // Shifting from the CoM to an origin 0.6 away adds m·d² to the two perpendicular axes
    // and leaves the parallel one alone.
    expect(after.about).toBe('origin');
    expect(after.ixx).toBeCloseTo(before.ixx + body.mass * comZ * comZ, 10);
    expect(after.iyy).toBeCloseTo(before.iyy + body.mass * comZ * comZ, 10);
    expect(after.izz).toBeCloseTo(before.izz, 10);
  });

  it('converting back and forth round-trips', () => {
    const before = { ...useModelStore.getState().bodies.upper!.inertia };
    useModelStore.getState().convertInertiaReference('upper', 'origin');
    useModelStore.getState().convertInertiaReference('upper', 'com');
    const after = useModelStore.getState().bodies.upper!.inertia;
    expect(after.ixx).toBeCloseTo(before.ixx, 10);
    expect(after.iyy).toBeCloseTo(before.iyy, 10);
    expect(after.izz).toBeCloseTo(before.izz, 10);
  });
});

describe('deleting a body', () => {
  it('re-homes the children without moving them', () => {
    // Where the forearm's root sits in the world, before and after deleting the arm it
    // hangs from. Deletion must not teleport it.
    const worldPositionOfLowerRoot = (): Vec3 => {
      const state = useModelStore.getState();
      const built = buildSpec(state.bodies, state.hinges, state.actuators, state.settings);
      if (!built.ok) throw new Error('model did not build');
      const model = buildModel(built.spec);
      const d = makeDynamics(model);
      const scratch = makeStepScratch(model);
      const s: State = { q: Float64Array.from(model.q0), v: new Float64Array(model.nv) };
      // A zero-length step just refreshes the kinematics.
      step(d, s, 0, 0, 'rk4', scratch);
      const link = model.links[model.links.length - 1]!;
      return [link.Xworld.r[0]!, link.Xworld.r[1]!, link.Xworld.r[2]!];
    };

    const before = worldPositionOfLowerRoot();
    useModelStore.getState().removeBody('upper');
    const after = worldPositionOfLowerRoot();

    expect(useModelStore.getState().bodies.upper).toBeUndefined();
    expect(useModelStore.getState().bodies.lower).toBeDefined();
    for (let i = 0; i < 3; i++) expect(after[i]!).toBeCloseTo(before[i]!, 9);
  });

  it('takes the deleted body\'s actuators with it', () => {
    expect(useModelStore.getState().actuators.drive).toBeDefined();
    useModelStore.getState().removeBody('upper');
    expect(useModelStore.getState().actuators.drive).toBeUndefined();
  });

  it('refuses to delete ground', () => {
    useModelStore.getState().removeBody(GROUND_ID);
    expect(useModelStore.getState().bodies[GROUND_ID]).toBeDefined();
  });
});

describe('unit systems', () => {
  it('never rewrites a customised gravity when switching', () => {
    useModelStore.getState().setSettings({ gravity: [0, 0, -1.62] }); // the Moon
    useModelStore.getState().setUnits('imperial');
    expect(useModelStore.getState().settings.gravity).toEqual([0, 0, -1.62]);
    expect(useModelStore.getState().settings.units).toBe('imperial');
  });

  it('carries an untouched gravity across to the new system', () => {
    useModelStore.getState().setUnits('imperial');
    const g = useModelStore.getState().settings.gravity;
    expect(Math.hypot(...g)).toBeCloseTo(STANDARD_GRAVITY_IMPERIAL, 6);
    expect(g[2]).toBeLessThan(0); // still pointing down
  });

  it('zeroes gravity in Generic, which has no standard value', () => {
    useModelStore.getState().setUnits('generic');
    expect(useModelStore.getState().settings.gravity).toEqual([0, 0, 0]);
  });

  it('never touches mass, length or inertia', () => {
    const before = JSON.stringify(useModelStore.getState().bodies);
    useModelStore.getState().setUnits('imperial');
    useModelStore.getState().setUnits('generic');
    useModelStore.getState().setUnits('si');
    expect(JSON.stringify(useModelStore.getState().bodies)).toBe(before);
  });

  it('preserves a non-vertical gravity direction across the switch', () => {
    const tilted: Vec3 = [STANDARD_GRAVITY_SI * 0.6, 0, -STANDARD_GRAVITY_SI * 0.8];
    const next = gravityOnSystemChange(tilted, 'si', 'imperial');
    expect(next).not.toBeNull();
    expect(Math.hypot(...next!)).toBeCloseTo(STANDARD_GRAVITY_IMPERIAL, 6);
    // Same direction, new magnitude.
    expect(next![0] / next![2]!).toBeCloseTo(tilted[0] / tilted[2], 9);
  });
});

describe('diagnostics', () => {
  const run = () => {
    const s = useModelStore.getState();
    return runDiagnostics(s.bodies, s.hinges, s.actuators, s.settings, s.contactSpheres, s.contactPlanes);
  };
  const ids = () => run().map((d) => d.id);

  it('is quiet on the default model', () => {
    expect(run().filter((d) => d.severity === 'error')).toHaveLength(0);
  });

  it('flags gravity that looks like the other system', () => {
    useModelStore.getState().setSettings({ gravity: [0, 0, -STANDARD_GRAVITY_IMPERIAL] });
    expect(ids()).toContain('gravity-units');

    const fix = run().find((d) => d.id === 'gravity-units')!.fix!;
    expect(fix.kind).toBe('setGravity');
    useModelStore.getState().applyFix(fix);
    expect(Math.hypot(...useModelStore.getState().settings.gravity)).toBeCloseTo(STANDARD_GRAVITY_SI, 6);
    expect(ids()).not.toContain('gravity-units');
  });

  it('does not flag legitimate low-gravity bodies', () => {
    for (const g of [1.62, 3.72, 0, 24.79]) {
      useModelStore.getState().setSettings({ gravity: [0, 0, -g] });
      expect(ids()).not.toContain('gravity-units');
    }
  });

  it('skips unit-specific checks entirely in Generic', () => {
    useModelStore.getState().setSettings({
      units: 'generic',
      gravity: [0, 0, -STANDARD_GRAVITY_IMPERIAL],
    });
    expect(ids()).not.toContain('gravity-units');
  });

  it('catches an inertia that violates the triangle inequality', () => {
    useModelStore.getState().setInertia('upper', { ixx: 0.01, iyy: 0.01, izz: 5 });
    expect(ids()).toContain('inertia-triangle:upper');
  });

  it('catches a negative principal moment', () => {
    useModelStore.getState().setInertia('upper', { ixx: -1 });
    expect(ids()).toContain('inertia-pd:upper');
  });

  it('catches an inertia inconsistent with the body size', () => {
    // A body spanning about a metre cannot have a radius of gyration of hundreds — the
    // fingerprint of a tensor entered in g·mm².
    useModelStore.getState().setInertia('upper', { ixx: 1e6, iyy: 1e6, izz: 1e6 });
    expect(ids()).toContain('gyration:upper');
  });

  it('warns when the timestep cannot resolve a stiff travel stop', () => {
    useModelStore.getState().setDofLimit('elbow', 4, { enabled: true, lo: -2, hi: 2, stiffness: 1e7 });
    const stiff = run().find((d) => d.id === 'stiff-timestep');
    expect(stiff).toBeDefined();
    expect(stiff!.fix?.kind).toBe('setTimestep');

    useModelStore.getState().applyFix(stiff!.fix!);
    expect(run().find((d) => d.id === 'stiff-timestep')).toBeUndefined();
  });

  it('flags invalid contact geometry and material values', () => {
    const sphereId = useModelStore.getState().addContactSphere('lower')!;
    const planeId = useModelStore.getState().addContactPlane();
    useModelStore.getState().setContactSphere(sphereId, { radius: -1 });
    useModelStore.getState().setContactMaterial('sphere', sphereId, { stiffness: -2 });
    useModelStore.getState().setContactPlane(planeId, { normal: [0, 0, 0] });

    expect(ids()).toContain(`contact-radius:${sphereId}`);
    expect(ids()).toContain(`contact-material:sphere:${sphereId}`);
    expect(ids()).toContain(`contact-normal:${planeId}`);
  });

  it('reports contact that begins penetrated', () => {
    const sphereId = useModelStore.getState().addContactSphere('lower')!;
    const planeId = useModelStore.getState().addContactPlane();
    useModelStore.getState().setContactSphere(sphereId, { radius: 10 });
    useModelStore.getState().setContactPlane(planeId, { point: [0, 0, 0], normal: [0, 0, 1] });

    expect(ids().some((id) => id.startsWith('contact-overlap:plane:'))).toBe(true);
  });

  it('warns when the timestep cannot resolve contact stiffness', () => {
    const sphereId = useModelStore.getState().addContactSphere('lower')!;
    const planeId = useModelStore.getState().addContactPlane();
    useModelStore.getState().setContactMaterial('sphere', sphereId, { stiffness: 1e9 });
    useModelStore.getState().setContactMaterial('plane', planeId, { stiffness: 1e9 });

    const found = run().find((diagnostic) => diagnostic.id === 'contact-stiff-timestep');
    expect(found?.fix?.kind).toBe('setTimestep');
  });

  it('notices when everything is locked', () => {
    for (const axis of [0, 1, 2, 3, 4, 5]) {
      useModelStore.getState().setDof('shoulder', axis, { free: false });
      useModelStore.getState().setDof('elbow', axis, { free: false });
    }
    expect(ids()).toContain('no-dof');
  });

  it('reports a gimbal-locked two-rotation hinge', () => {
    useModelStore.getState().setDof('elbow', 3, { free: true });
    useModelStore.getState().setDof('elbow', 4, { free: false, q0: Math.PI / 2 });
    useModelStore.getState().setDof('elbow', 5, { free: true });
    expect(ids()).toContain('gimbal:elbow');
  });

  it('explains that a ball joint cannot carry a spring', () => {
    useModelStore.getState().setDof('elbow', 3, { free: true });
    useModelStore.getState().setDof('elbow', 4, { free: true });
    useModelStore.getState().setDof('elbow', 5, { free: true, stiffness: 10 });

    const found = run().find((d) => d.id === 'quat-spring:elbow');
    expect(found).toBeDefined();
    // Damping is still usable on a ball joint, so the message must not claim otherwise.
    expect(found!.detail).toMatch(/Damping and friction still apply/);
    expect(found!.title).toContain('rz');
  });

  it('names every affected axis, not just the first', () => {
    useModelStore.getState().setDof('elbow', 3, { free: true, stiffness: 5 });
    useModelStore.getState().setDof('elbow', 4, { free: true });
    useModelStore.getState().setDof('elbow', 5, { free: true, stiffness: 10 });
    const found = run().find((d) => d.id === 'quat-spring:elbow')!;
    expect(found.title).toContain('rx');
    expect(found.title).toContain('rz');
  });

  it('reports an unparseable actuator expression', () => {
    useModelStore.getState().setActuatorProfile('drive', { kind: 'expr', source: 'sin(' });
    expect(ids()).toContain('build:expression:drive');
  });

  it('reports a massless body carrying free axes as degenerate', () => {
    useModelStore.getState().setMass('lower', 0);
    useModelStore.getState().setInertia('lower', { ixx: 0, iyy: 0, izz: 0 });
    const found = run();
    expect(found.map((d) => d.id)).toContain('mass:lower');
    expect(found.some((d) => d.id === 'singular')).toBe(true);
  });
});

describe('actuator profiles', () => {
  it('compiles each GUI profile to the shape it advertises', () => {
    expect(compileProfile({ kind: 'constant' }).fn(5)).toBe(1);

    const stepped = compileProfile({ kind: 'step', tOn: 1, tOff: 3 }).fn;
    expect([stepped(0), stepped(2), stepped(3.5)]).toEqual([0, 1, 0]);

    const ramp = compileProfile({ kind: 'ramp', t0: 0, t1: 2, from: 0, to: 10 }).fn;
    expect(ramp(-1)).toBe(0);
    expect(ramp(1)).toBe(5);
    expect(ramp(9)).toBe(10);

    const sine = compileProfile({ kind: 'sine', frequency: 1, phase: 0, offset: 0 }).fn;
    expect(sine(0)).toBeCloseTo(0, 12);
    expect(sine(0.25)).toBeCloseTo(1, 12);

    // The impulse is normalized so its area is 1 regardless of width — narrowing it does
    // not quietly change how much impulse gets delivered.
    const narrow = compileProfile({ kind: 'impulse', t0: 0, width: 0.01 }).fn;
    const wide = compileProfile({ kind: 'impulse', t0: 0, width: 0.5 }).fn;
    expect(narrow(0.005) * 0.01).toBeCloseTo(wide(0.25) * 0.5, 12);
  });

  it('degrades a broken expression to zero and says why', () => {
    const result = compileProfile({ kind: 'expr', source: '2 +' });
    expect(result.error).toBeTruthy();
    expect(result.fn(1)).toBe(0);
  });
});

describe('persistence repair', () => {
  it('rejects payloads with nothing salvageable', () => {
    expect(repairModel(null)).toBeNull();
    expect(repairModel('nope')).toBeNull();
    expect(repairModel({})).toBeNull();
  });

  it('restores ground when a payload has lost it', () => {
    const repaired = repairModel({ bodies: { a: { name: 'A' } } })!;
    expect(repaired.bodies[GROUND_ID]).toBeDefined();
    expect(repaired.bodies[GROUND_ID]!.isGround).toBe(true);
    expect(repaired.bodyOrder[0]).toBe(GROUND_ID);
  });

  it('gives an unattached body a hinge so it still exists', () => {
    const repaired = repairModel({ bodies: { a: { name: 'A' } } })!;
    const hinge = Object.values(repaired.hinges).find((h) => h.childBodyId === 'a');
    expect(hinge).toBeDefined();
    expect(hinge!.parentBodyId).toBe(GROUND_ID);
  });

  it('breaks a smuggled-in cycle by re-rooting onto ground', () => {
    const repaired = repairModel({
      bodies: { a: { name: 'A' }, b: { name: 'B' } },
      hinges: {
        h1: { childBodyId: 'a', parentBodyId: 'b', dof: [] },
        h2: { childBodyId: 'b', parentBodyId: 'a', dof: [] },
      },
    })!;
    const { cycle } = orderHingeIds(repaired.hinges);
    expect(cycle).toBeNull();
  });

  it('keeps only one inbound hinge per body', () => {
    const repaired = repairModel({
      bodies: { a: { name: 'A' } },
      hinges: {
        h1: { childBodyId: 'a', parentBodyId: GROUND_ID, dof: [] },
        h2: { childBodyId: 'a', parentBodyId: GROUND_ID, dof: [] },
      },
    })!;
    expect(Object.values(repaired.hinges).filter((h) => h.childBodyId === 'a')).toHaveLength(1);
  });

  it('shifts nodes when a payload claims a non-zero origin', () => {
    const repaired = repairModel({
      bodies: {
        a: {
          name: 'A',
          nodes: { n1: { name: 'One', position: [1, 2, 3] }, n2: { name: 'Two', position: [1, 2, 5] } },
          originNodeId: 'n1',
        },
      },
    })!;
    expect(repaired.bodies.a!.nodes.n1!.position).toEqual([0, 0, 0]);
    expect(repaired.bodies.a!.nodes.n2!.position).toEqual([0, 0, 2]);
  });

  it('normalizes a malformed quaternion instead of propagating NaN', () => {
    const repaired = repairModel({
      bodies: { a: { name: 'A', nodes: { n: { position: [0, 0, 0], orientation: [5, 0, 0, 0] } } } },
    })!;
    const q = repaired.bodies.a!.nodes.n!.orientation;
    expect(Math.hypot(...q)).toBeCloseTo(1, 12);
  });

  it('refuses a non-positive timestep', () => {
    const repaired = repairModel({ bodies: { a: {} }, settings: { dt: 0, duration: -5 } })!;
    expect(repaired.settings.dt).toBeGreaterThan(0);
    expect(repaired.settings.duration).toBeGreaterThan(0);
  });

  it('round-trips a healthy model unchanged', () => {
    const original = initialModel();
    const repaired = repairModel(JSON.parse(JSON.stringify(original)))!;
    expect(repaired.bodyOrder).toEqual(original.bodyOrder);
    expect(Object.keys(repaired.hinges).sort()).toEqual(Object.keys(original.hinges).sort());
    expect(repaired.settings).toEqual(original.settings);
    expect(repaired.bodies.upper!.mass).toBe(original.bodies.upper!.mass);
  });
});

describe('node editing guards', () => {
  it('pins the origin node at zero', () => {
    useModelStore.getState().setNodePosition('upper', 'upper-root', [1, 2, 3]);
    expect(useModelStore.getState().bodies.upper!.nodes['upper-root']!.position).toEqual([0, 0, 0]);
  });

  it('will not delete a node a hinge depends on', () => {
    useModelStore.getState().removeNode('upper', 'upper-tip'); // the elbow hangs off this
    expect(useModelStore.getState().bodies.upper!.nodes['upper-tip']).toBeDefined();
  });

  it('will not delete the origin or the centre of mass', () => {
    useModelStore.getState().removeNode('upper', 'upper-root');
    useModelStore.getState().removeNode('upper', 'upper-com');
    expect(useModelStore.getState().bodies.upper!.nodes['upper-root']).toBeDefined();
    expect(useModelStore.getState().bodies.upper!.nodes['upper-com']).toBeDefined();
  });

  it('deletes a node nothing is using', () => {
    const id = useModelStore.getState().addNode('upper')!;
    expect(useModelStore.getState().bodies.upper!.nodes[id]).toBeDefined();
    useModelStore.getState().removeNode('upper', id);
    expect(useModelStore.getState().bodies.upper!.nodes[id]).toBeUndefined();
  });
});

describe('adding bodies', () => {
  it('attaches a new body to the selection with one free axis', () => {
    const id = useModelStore.getState().addBody('lower');
    const state = useModelStore.getState();
    const hinge = Object.values(state.hinges).find((h) => h.childBodyId === id)!;
    expect(hinge.parentBodyId).toBe('lower');
    expect(hinge.dof.filter((d) => d.free)).toHaveLength(1);
    expect(state.selectedBodyId).toBe(id);
  });

  it('keeps the model solvable after each addition', () => {
    for (let i = 0; i < 4; i++) useModelStore.getState().addBody();
    const s = useModelStore.getState();
    const errors = runDiagnostics(s.bodies, s.hinges, s.actuators, s.settings).filter(
      (d) => d.severity === 'error',
    );
    expect(errors).toHaveLength(0);
  });
});

describe('hinge degrees of freedom', () => {
  it('resets to fully locked', () => {
    useModelStore.getState().resetHingeDof('shoulder');
    expect(useModelStore.getState().hinges.shoulder!.dof).toEqual(neutralDofSet());
  });

  it('stores per-axis spring, damper, friction and limits', () => {
    useModelStore.getState().setDof('elbow', 4, { stiffness: 12, damping: 0.4, friction: 0.2 });
    useModelStore.getState().setDofLimit('elbow', 4, { enabled: true, lo: -1, hi: 1 });
    const dof = useModelStore.getState().hinges.elbow!.dof[4]!;
    expect(dof).toMatchObject({ stiffness: 12, damping: 0.4, friction: 0.2 });
    expect(dof.limit).toMatchObject({ enabled: true, lo: -1, hi: 1 });
    expect(neutralDof().stiffness).toBe(0);
  });
});
