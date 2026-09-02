import { GROUND_ID, type Actuator, type Body, type ContactPlane, type ContactSphere, type Hinge, type SimSettings, type SpringDamper, type UnitSystem, type Vec3 } from '../types';
import { buildModel, pointToWorld } from '../dyn/model';
import { makeDynamics } from '../dyn/forward';
import { crba, factorize } from '../dyn/crba';
import { updateKinematics, updateVelocities } from '../dyn/model';
import { checkInertia, tensorOf } from '../dyn/inertia';
import { makeJointModel, rotationalAxisSeparation } from '../dyn/joints';
import { buildSpec } from './adapter';
import { DOF_LABELS } from '../types';
import { m3, v3 } from '../dyn/spatial';
import { spherePlanePenetration } from '../dyn/contact';
import {
  STANDARD_GRAVITY_IMPERIAL,
  STANDARD_GRAVITY_SI,
  UNIT_SYSTEMS,
  unitLabel,
} from '../units';

/**
 * The "something looks wrong" subsystem.
 *
 * Everything here is **advisory and never blocking**. A back-of-the-envelope tool that
 * refuses to run because it dislikes your numbers is useless — sometimes the whole point is
 * to see what an absurd model does. So these warn, explain, and where there is an obvious
 * correction, offer it as one tap.
 *
 * The checks divide in two. Most are structural or built from dimensionless ratios, so they
 * are meaningful whatever the numbers mean and run in every unit system. A few compare
 * against known physical constants and only make sense once you have said which system you
 * are in; those are skipped entirely in Generic, where whatever you type is taken at face
 * value.
 */

export type DiagnosticSeverity = 'error' | 'warning' | 'info';

export type DiagnosticFix =
  | { label: string; kind: 'setGravity'; value: Vec3 }
  | { label: string; kind: 'setUnits'; value: UnitSystem }
  | { label: string; kind: 'setTimestep'; value: number };

export type Diagnostic = {
  id: string;
  severity: DiagnosticSeverity;
  title: string;
  detail: string;
  target?: { kind: 'body' | 'hinge' | 'actuator' | 'springDamper' | 'settings'; id: string };
  fix?: DiagnosticFix;
};

/** Below this the free axes of a two-rotation hinge are effectively the same axis. */
const AXIS_SEPARATION_FLOOR = 0.05;

/** `ωₙ·dt` above this and an explicit integrator starts misrepresenting the oscillation. */
const STIFFNESS_STEP_LIMIT = 0.2;

/** More than half a sphere radius per step makes discrete contact easy to skip. */
const CONTACT_TRAVEL_LIMIT = 0.5;

/** Ratio between radius of gyration and body size that suggests a unit mix-up. */
const GYRATION_RATIO_LIMIT = 30;

/** Fractional closeness at which a gravity value is "suspiciously the other system's". */
const GRAVITY_MATCH_TOLERANCE = 0.02;

const formatNumber = (value: number): string => {
  if (value === 0) return '0';
  const abs = Math.abs(value);
  if (abs >= 1e5 || abs < 1e-3) return value.toExponential(2);
  return String(Math.round(value * 1000) / 1000);
};

/** The largest distance between any two nodes on a body — its effective size. */
function bodyExtent(body: Body): number {
  const nodes = Object.values(body.nodes);
  let max = 0;
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      const a = nodes[i]!.position;
      const b = nodes[j]!.position;
      max = Math.max(max, Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]));
    }
  }
  return max;
}

export function runDiagnostics(
  bodies: Record<string, Body>,
  hinges: Record<string, Hinge>,
  actuators: Record<string, Actuator>,
  settings: SimSettings,
  contactSpheres: Record<string, ContactSphere> = {},
  contactPlanes: Record<string, ContactPlane> = {},
  springDampers: Record<string, SpringDamper> = {},
): Diagnostic[] {
  const out: Diagnostic[] = [];
  const system = UNIT_SYSTEMS[settings.units];
  const massUnit = unitLabel(settings.units, 'mass');
  const lengthUnit = unitLabel(settings.units, 'length');
  const inertiaUnit = unitLabel(settings.units, 'inertia');

  const built = buildSpec(bodies, hinges, actuators, settings, contactSpheres, contactPlanes, springDampers);

  // Contact data is deliberately editable even when implausible; diagnose it before the
  // compiler clamps invalid values or drops a plane with no usable normal.
  for (const sphere of Object.values(contactSpheres)) {
    if (sphere.radius < 0) {
      out.push({
        id: `contact-radius:${sphere.id}`,
        severity: 'warning',
        title: `${sphere.name} has a negative radius`,
        detail: 'The solver treats it as zero. Use zero explicitly for a point contact, or enter a positive sphere radius.',
      });
    }
    if (sphere.material.stiffness < 0 || sphere.material.damping < 0 || sphere.material.friction < 0 || sphere.material.frictionVelocity <= 0) {
      out.push({
        id: `contact-material:sphere:${sphere.id}`,
        severity: 'warning',
        title: `${sphere.name} has a negative contact property`,
        detail: 'Stiffness, damping, and friction must be non-negative; friction velocity must be positive. The solver clamps invalid values.',
      });
    }
  }
  for (const plane of Object.values(contactPlanes)) {
    if (!(Math.hypot(...plane.normal) > 1e-12)) {
      out.push({
        id: `contact-normal:${plane.id}`,
        severity: 'warning',
        title: `${plane.name} has no normal`,
        detail: 'A plane needs a non-zero allowed-side normal. This plane is ignored by the solver.',
      });
    }
    if (plane.material.stiffness < 0 || plane.material.damping < 0 || plane.material.friction < 0 || plane.material.frictionVelocity <= 0) {
      out.push({
        id: `contact-material:plane:${plane.id}`,
        severity: 'warning',
        title: `${plane.name} has a negative contact property`,
        detail: 'Stiffness, damping, and friction must be non-negative; friction velocity must be positive. The solver clamps invalid values.',
      });
    }
  }
  for (const device of Object.values(springDampers)) {
    if (device.bodyAId === device.bodyBId) {
      out.push({
        id: `spring-damper-same-body:${device.id}`,
        severity: 'warning',
        title: `${device.name} attaches to one body twice`,
        detail: 'A spring-damper must connect nodes on two different bodies. This device is ignored by the solver.',
        target: { kind: 'springDamper', id: device.id },
      });
    }
    if (device.stiffness < 0 || device.damping < 0 || device.restLength < 0) {
      out.push({
        id: `spring-damper-values:${device.id}`,
        severity: 'warning',
        title: `${device.name} has a negative property`,
        detail: 'Stiffness, damping, and rest length must be non-negative. The solver clamps invalid values.',
        target: { kind: 'springDamper', id: device.id },
      });
    }
  }

  // --- structural problems surfaced by the build -----------------------------------
  for (const problem of built.problems ?? []) {
    out.push({
      id: `build:${problem.kind}:${problem.targetId ?? 'model'}`,
      severity: problem.kind === 'cycle' ? 'error' : 'warning',
      title:
        problem.kind === 'cycle'
          ? 'Closed loop'
          : problem.kind === 'expression'
            ? 'Expression will not parse'
            : problem.kind === 'orphan'
              ? 'Actuator has no effect'
              : problem.kind === 'springDamper'
                ? 'Spring-damper needs attention'
              : 'Dangling reference',
      detail: problem.message,
      ...(problem.targetId
        ? {
            target: {
              kind: (problem.targetKind ?? (problem.kind === 'expression' || problem.kind === 'orphan'
                ? 'actuator'
                : 'hinge')) as 'actuator' | 'hinge' | 'springDamper',
              id: problem.targetId,
            },
          }
        : {}),
    });
  }
  if (!built.ok) return out;

  const movable = Object.values(bodies).filter((b) => !b.isGround);
  const freeDofCount = Object.values(hinges).reduce(
    (n, hinge) => n + hinge.dof.filter((d) => d.free).length,
    0,
  );

  // --- per-body mass properties -----------------------------------------------------
  for (const body of movable) {
    if (body.mass <= 0) {
      out.push({
        id: `mass:${body.id}`,
        severity: body.mass < 0 ? 'error' : 'warning',
        title: body.mass < 0 ? `${body.name} has negative mass` : `${body.name} is massless`,
        detail:
          body.mass < 0
            ? 'Negative mass is not physical and will make the mass matrix indefinite.'
            : `A zero-mass body is fine as a pure attachment point, but if it carries free axes of its own the solve can become singular.`,
        target: { kind: 'body', id: body.id },
      });
    }

    const tensor = tensorOf(body.inertia);
    const check = checkInertia(body.mass, tensor);
    const anyInertia = check.principal.some((p) => Math.abs(p) > 0);

    if (anyInertia && !check.positiveDefinite) {
      out.push({
        id: `inertia-pd:${body.id}`,
        severity: 'error',
        title: `${body.name} has a non-physical inertia tensor`,
        detail:
          `Its principal moments are ${check.principal.map(formatNumber).join(', ')} ${inertiaUnit}. ` +
          'A real mass distribution has all three strictly positive — a negative one usually means a sign error on a product of inertia.',
        target: { kind: 'body', id: body.id },
      });
    } else if (anyInertia && !check.triangleInequality) {
      const [i1, i2, i3] = check.principal;
      out.push({
        id: `inertia-triangle:${body.id}`,
        severity: 'warning',
        title: `${body.name} violates the inertia triangle inequality`,
        detail:
          `Its principal moments are ${check.principal.map(formatNumber).join(', ')} ${inertiaUnit}, ` +
          `and ${formatNumber(i1)} + ${formatNumber(i2)} < ${formatNumber(i3)}. No physical body can do that. ` +
          'The simulation will still run, but it is describing something that cannot exist.',
        target: { kind: 'body', id: body.id },
      });
    }

    // Radius of gyration against the body's actual size. A pure ratio, so this is
    // meaningful in any unit system — and it is the classic fingerprint of an inertia
    // entered in the wrong ones.
    const extent = bodyExtent(body);
    if (anyInertia && check.radiusOfGyration > 0 && extent > 0) {
      const ratio = check.radiusOfGyration / extent;
      if (ratio > GYRATION_RATIO_LIMIT || ratio < 1 / GYRATION_RATIO_LIMIT) {
        const tooLarge = ratio > 1;
        const hint =
          settings.units === 'si'
            ? tooLarge
              ? 'Inertia entered in g·mm² would be about a million times too large for kg·m².'
              : 'Inertia entered in kg·mm² would be about a million times too small for kg·m².'
            : settings.units === 'imperial'
              ? tooLarge
                ? 'Inertia entered in lbm·in² is about 3800× too large for slug·ft².'
                : 'Check whether the moments are in slug·ft².'
              : 'Check that the inertia and the node positions use the same length unit.';
        out.push({
          id: `gyration:${body.id}`,
          severity: 'warning',
          title: `${body.name}'s inertia looks inconsistent with its size`,
          detail:
            `Its radius of gyration is ${formatNumber(check.radiusOfGyration)} ${lengthUnit} but the body spans only ` +
            `${formatNumber(extent)} ${lengthUnit} — a factor of ${formatNumber(tooLarge ? ratio : 1 / ratio)}. ${hint}`,
          target: { kind: 'body', id: body.id },
        });
      }
    }

    if (settings.units !== 'generic') {
      if (body.mass > 0 && (body.mass < 1e-9 || body.mass > 1e12)) {
        out.push({
          id: `mass-magnitude:${body.id}`,
          severity: 'warning',
          title: `${body.name}'s mass is an unusual magnitude`,
          detail: `${formatNumber(body.mass)} ${massUnit} is far outside the range of anything normally modelled. Check for a misplaced exponent.`,
          target: { kind: 'body', id: body.id },
        });
      }
      const farthest = Math.max(
        0,
        ...Object.values(body.nodes).map((n) => Math.hypot(...n.position)),
      );
      if (farthest > 1e9) {
        out.push({
          id: `extent-magnitude:${body.id}`,
          severity: 'warning',
          title: `${body.name} has a node very far from its origin`,
          detail: `${formatNumber(farthest)} ${lengthUnit} from the body origin. Check for a misplaced exponent.`,
          target: { kind: 'body', id: body.id },
        });
      }
    }
  }

  // --- hinges ------------------------------------------------------------------------
  for (const hinge of Object.values(hinges)) {
    const joint = makeJointModel(
      hinge.dof.map((d) => d.free),
      hinge.dof.map((d) => d.q0),
      0,
      0,
    );

    const separation = rotationalAxisSeparation(joint);
    if (separation < AXIS_SEPARATION_FLOOR) {
      out.push({
        id: `gimbal:${hinge.id}`,
        severity: 'warning',
        title: `${hinge.name} is at gimbal lock`,
        detail:
          'Its free rx and rz axes point the same way because ry is locked near ±90°, so the two coordinates ' +
          'describe the same rotation and the mass matrix is singular. Free ry as well to make it a ball joint, ' +
          'or move the locked ry angle away from ±90°.',
        target: { kind: 'hinge', id: hinge.id },
      });
    }

    // Springs and stops need a scalar coordinate to measure against; a fully-free rotation
    // is a quaternion and has none.
    if (joint.useQuaternion) {
      const inactive = joint.freeRot.filter((axis) => {
        const dof = hinge.dof[axis + 3]!;
        return dof.stiffness !== 0 || dof.limit.enabled;
      });
      if (inactive.length > 0) {
        const names = inactive.map((axis) => DOF_LABELS[axis + 3]).join(', ');
        out.push({
          id: `quat-spring:${hinge.id}`,
          severity: 'info',
          title: `${hinge.name}: ${names} spring and stop ${inactive.length > 1 ? 'are' : 'is'} inactive`,
          detail:
            'With all three rotations free this hinge is stored as a quaternion, which has no single angle for a ' +
            'spring or a travel stop to work against. Damping and friction still apply. Lock one rotation to get ' +
            'the angles back.',
          target: { kind: 'hinge', id: hinge.id },
        });
      }
    }

    for (let axis = 0; axis < 6; axis++) {
      const dof = hinge.dof[axis]!;
      if (!dof.free) continue;
      if (dof.limit.enabled && dof.limit.lo > dof.limit.hi) {
        out.push({
          id: `limit-order:${hinge.id}:${axis}`,
          severity: 'warning',
          title: `${hinge.name}: ${DOF_LABELS[axis]} travel limits are inverted`,
          detail: `The lower stop (${formatNumber(dof.limit.lo)}) is above the upper stop (${formatNumber(dof.limit.hi)}), so the axis is trapped between them.`,
          target: { kind: 'hinge', id: hinge.id },
        });
      }
      if (dof.free && dof.limit.enabled && (dof.q0 < dof.limit.lo || dof.q0 > dof.limit.hi)) {
        out.push({
          id: `limit-start:${hinge.id}:${axis}`,
          severity: 'warning',
          title: `${hinge.name}: ${DOF_LABELS[axis]} starts outside its travel limits`,
          detail: `It begins at ${formatNumber(dof.q0)}, past a stop, so the run opens with the penalty spring already pushing hard.`,
          target: { kind: 'hinge', id: hinge.id },
        });
      }
    }
  }

  // --- whole-model checks that need the assembled solver ------------------------------
  if (freeDofCount === 0) {
    out.push({
      id: 'no-dof',
      severity: 'info',
      title: 'Nothing can move',
      detail:
        movable.length === 0
          ? 'The model has no bodies yet.'
          : 'Every axis on every hinge is locked, so the model is one rigid assembly bolted to ground. Free an axis to give it somewhere to go.',
      target: { kind: 'settings', id: 'dof' },
    });
  } else {
    try {
      const model = buildModel(built.spec);
      const dynamics = makeDynamics(model);
      const q = Float64Array.from(model.q0);
      const v = Float64Array.from(model.v0);
      updateKinematics(model, q, v, dynamics.kin);
      updateVelocities(model, dynamics.kin);
      crba(model, dynamics.H, dynamics.crbaScratch);

      // Report initial overlap using the exact compiled geometry and initial kinematics.
      // Starting compressed is supported, but it opens with an impulsive-looking spring load.
      const spherePositions = model.contactSpheres.map((sphere) =>
        pointToWorld(model.links[sphere.link]!, sphere.point, v3(), m3()),
      );
      const contactNormal = v3();

      // Discrete contact only samples geometry at step boundaries. Warn when an initially
      // moving finite-radius sphere can travel a substantial fraction of its radius in one
      // step; exact point contacts have no meaningful length scale for this heuristic.
      let worstTravelRatio = 0;
      let worstTravelLabel = '';
      let worstTravelSpeed = 0;
      let worstTravelRadius = 0;
      const hasPossibleContact = model.contactPlanes.length > 0 || model.contactSpheres.length > 1;
      for (const sphere of hasPossibleContact ? model.contactSpheres : []) {
        if (!(sphere.radius > 0)) continue;
        const link = model.links[sphere.link]!;
        const point = sphere.point;
        const wx = link.v[0]!, wy = link.v[1]!, wz = link.v[2]!;
        const lx = link.v[3]! + wy * point[2]! - wz * point[1]!;
        const ly = link.v[4]! + wz * point[0]! - wx * point[2]!;
        const lz = link.v[5]! + wx * point[1]! - wy * point[0]!;
        // Rotation preserves magnitude, so the link-frame point speed is sufficient here.
        const speed = Math.hypot(lx, ly, lz);
        const ratio = speed * settings.dt / sphere.radius;
        if (ratio > worstTravelRatio) {
          worstTravelRatio = ratio;
          worstTravelLabel = sphere.name;
          worstTravelSpeed = speed;
          worstTravelRadius = sphere.radius;
        }
      }
      if (worstTravelRatio > CONTACT_TRAVEL_LIMIT) {
        const suggested = CONTACT_TRAVEL_LIMIT * worstTravelRadius / worstTravelSpeed;
        out.push({
          id: 'contact-tunnelling',
          severity: 'warning',
          title: 'A contact sphere may cross a surface between steps',
          detail:
            `${worstTravelLabel} initially travels ${formatNumber(worstTravelRatio)} radii per step. ` +
            `The contact model is discrete, so use a step near or below ${formatNumber(suggested)} to reduce tunnelling risk.`,
          target: { kind: 'settings', id: 'dt' },
          fix: { label: `Use dt = ${formatNumber(suggested)}`, kind: 'setTimestep', value: suggested },
        });
      }
      for (let i = 0; i < model.contactSpheres.length; i++) {
        const sphere = model.contactSpheres[i]!;
        const position = spherePositions[i]!;
        for (const plane of model.contactPlanes) {
          // The solver's own geometry, so a bounded plate the sphere is nowhere near does
          // not get reported as an overlap.
          const penetration = spherePlanePenetration(
            plane, position[0]!, position[1]!, position[2]!, sphere.radius, contactNormal,
          );
          if (penetration > 0) {
            out.push({
              id: `contact-overlap:plane:${i}:${plane.name}`,
              severity: 'warning',
              title: `${sphere.name} starts inside ${plane.name}`,
              detail: `Initial penetration is ${formatNumber(penetration)} ${lengthUnit}, so the contact spring is already loaded at t = 0.`,
            });
          }
        }
      }
      for (let i = 0; i < model.contactSpheres.length; i++) {
        for (let j = i + 1; j < model.contactSpheres.length; j++) {
          const a = model.contactSpheres[i]!;
          const b = model.contactSpheres[j]!;
          if (a.link === b.link) continue;
          const pa = spherePositions[i]!;
          const pb = spherePositions[j]!;
          const penetration = a.radius + b.radius - Math.hypot(pa[0]! - pb[0]!, pa[1]! - pb[1]!, pa[2]! - pb[2]!);
          if (penetration > 0) {
            out.push({
              id: `contact-overlap:spheres:${i}:${j}`,
              severity: 'warning',
              title: `${a.name} and ${b.name} start overlapped`,
              detail: `Initial penetration is ${formatNumber(penetration)} ${lengthUnit}, so their contact spring is already loaded at t = 0.`,
            });
          }
        }
      }

      const n = model.nv;
      // Singularity: a non-positive pivot names the exact coordinate that went degenerate,
      // which is far more actionable than "the solve failed".
      const factor = factorize(dynamics.H, dynamics.factorization);
      if (factor.failedAt >= 0) {
        out.push({
          id: 'singular',
          severity: 'error',
          title: 'The model is degenerate',
          detail:
            `The mass matrix is not invertible at ${model.dofNames[factor.failedAt] ?? `coordinate ${factor.failedAt}`}. ` +
            'Usually this means a massless body carrying free axes, two hinge axes that point the same way, or an ' +
            'inertia that is not positive definite.',
          target: { kind: 'settings', id: 'singular' },
        });
      } else if (factor.pivotSpread > 1e10) {
        out.push({
          id: 'ill-conditioned',
          severity: 'warning',
          title: 'The model is close to degenerate',
          detail:
            `The mass matrix spans a factor of ${factor.pivotSpread.toExponential(1)} between its stiffest and ` +
            'softest directions. Results will be noisy. This usually means one body is many orders of magnitude ' +
            'lighter than another, or an axis is nearly redundant.',
          target: { kind: 'settings', id: 'conditioning' },
        });
      }

      // Stiffness against the timestep. ωₙ·dt is dimensionless, so this is valid in any
      // unit system — and it is the check that saves a run when a hard stop is involved.
      let worstOmega = 0;
      let worstLabel = '';
      for (let i = 0; i < n; i++) {
        const binding = model.dofBindings[i]!;
        const effectiveMass = Math.max(dynamics.H[i * n + i]!, 1e-30);
        const stiffest = Math.max(
          binding.params.stiffness,
          binding.params.limitEnabled ? binding.params.limitStiffness : 0,
        );
        if (stiffest <= 0) continue;
        const omega = Math.sqrt(stiffest / effectiveMass);
        if (omega > worstOmega) {
          worstOmega = omega;
          worstLabel = model.dofNames[i] ?? `coordinate ${i}`;
        }
      }
      if (worstOmega * settings.dt > STIFFNESS_STEP_LIMIT) {
        const suggested = STIFFNESS_STEP_LIMIT / worstOmega;
        out.push({
          id: 'stiff-timestep',
          severity: 'warning',
          title: 'Timestep is too large for the stiffest spring',
          detail:
            `${worstLabel} oscillates at ${formatNumber(worstOmega)} rad per unit time, which needs a step below ` +
            `${formatNumber(suggested)} to resolve. At ${formatNumber(settings.dt)} the motion will be wrong and may ` +
            'diverge. Travel stops are the usual culprit — they are stiff by design.',
          target: { kind: 'settings', id: 'dt' },
          fix: { label: `Use dt = ${formatNumber(suggested)}`, kind: 'setTimestep', value: suggested },
        });
      }

      // Conservative contact estimate: the lightest participating body mass bounds the
      // effective normal mass from above. It errs toward a smaller, safer suggested step.
      let contactOmega = 0;
      let contactLabel = '';
      for (const sphere of model.contactSpheres) {
        const mass = Math.max(model.links[sphere.link]!.mass, 1e-30);
        for (const plane of model.contactPlanes) {
          const omega = Math.sqrt(Math.min(sphere.stiffness, plane.stiffness) / mass);
          if (omega > contactOmega) { contactOmega = omega; contactLabel = `${sphere.name} against ${plane.name}`; }
        }
      }
      if (contactOmega * settings.dt > STIFFNESS_STEP_LIMIT) {
        const suggested = STIFFNESS_STEP_LIMIT / contactOmega;
        out.push({
          id: 'contact-stiff-timestep',
          severity: 'warning',
          title: 'Timestep is too large for contact stiffness',
          detail: `${contactLabel} needs a step near or below ${formatNumber(suggested)}. This conservative estimate uses body mass and may over-warn for an off-centre contact.`,
          target: { kind: 'settings', id: 'dt' },
          fix: { label: `Use dt = ${formatNumber(suggested)}`, kind: 'setTimestep', value: suggested },
        });
      }

      // Thrust-to-weight, as context rather than a complaint: it is the fastest way to know
      // whether a run will show anything at all.
      const totalMass = movable.reduce((sum, b) => sum + Math.max(b.mass, 0), 0);
      const gravityMagnitude = Math.hypot(...settings.gravity);
      const totalThrust = Object.values(actuators)
        .filter((a) => a.enabled && a.kind === 'force')
        .reduce((sum, a) => sum + Math.hypot(...a.vector), 0);
      if (totalThrust > 0 && totalMass > 0 && gravityMagnitude > 0) {
        const ratio = totalThrust / (totalMass * gravityMagnitude);
        if (ratio < 0.05 || ratio > 100) {
          out.push({
            id: 'thrust-weight',
            severity: 'info',
            title: `Thrust is ${formatNumber(ratio)}× the model's weight`,
            detail:
              ratio < 0.05
                ? 'The actuators are weak relative to gravity, so expect very little motion over the run.'
                : 'The actuators overwhelm gravity by a wide margin — the run will be dominated by them.',
            target: { kind: 'settings', id: 'thrust' },
          });
        }
      }
    } catch (err) {
      out.push({
        id: 'build-failed',
        severity: 'error',
        title: 'The model could not be assembled',
        detail: err instanceof Error ? err.message : String(err),
      });
    }
  }

  // --- unit-system plausibility, skipped entirely in Generic --------------------------
  if (settings.units !== 'generic') {
    const magnitude = Math.hypot(...settings.gravity);
    const own = system.standardGravity ?? 0;
    const other = settings.units === 'si' ? STANDARD_GRAVITY_IMPERIAL : STANDARD_GRAVITY_SI;
    const otherName = settings.units === 'si' ? 'ft/s²' : 'm/s²';
    const ownName = settings.units === 'si' ? 'm/s²' : 'ft/s²';
    const otherSystem: UnitSystem = settings.units === 'si' ? 'imperial' : 'si';

    // Deliberately not a range check: Moon and Mars gravity are perfectly legitimate
    // values. What is suspicious is gravity sitting on the *other* system's constant.
    if (magnitude > 0 && Math.abs(magnitude - other) < GRAVITY_MATCH_TOLERANCE * other) {
      const scale = own / magnitude;
      out.push({
        id: 'gravity-units',
        severity: 'warning',
        title: `Gravity looks like ${otherName}, not ${ownName}`,
        detail:
          `It is set to ${formatNumber(magnitude)}, which is standard gravity in ${otherName} while this model is in ` +
          `${system.name}. Everything else will be off by a factor of ${formatNumber(other / own)} unless that was deliberate.`,
        target: { kind: 'settings', id: 'gravity' },
        fix: {
          label: `Rescale to ${formatNumber(own)} ${ownName}`,
          kind: 'setGravity',
          value: [
            settings.gravity[0] * scale,
            settings.gravity[1] * scale,
            settings.gravity[2] * scale,
          ],
        },
      });
      out.push({
        id: 'gravity-units-switch',
        severity: 'info',
        title: `…or switch to ${UNIT_SYSTEMS[otherSystem].name}`,
        detail: `If the rest of the model really is in ${UNIT_SYSTEMS[otherSystem].name}, change the unit system instead — no numbers will be rewritten.`,
        target: { kind: 'settings', id: 'gravity' },
        fix: { label: `Switch to ${UNIT_SYSTEMS[otherSystem].name}`, kind: 'setUnits', value: otherSystem },
      });
    }
  }

  // Ground carrying an inbound hinge would mean the tree has two roots.
  for (const hinge of Object.values(hinges)) {
    if (hinge.childBodyId === GROUND_ID) {
      out.push({
        id: `ground-child:${hinge.id}`,
        severity: 'error',
        title: 'Ground cannot be a child',
        detail: `"${hinge.name}" tries to hang Ground off another body. Ground is the inertial frame and is always the root.`,
        target: { kind: 'hinge', id: hinge.id },
      });
    }
  }

  return out;
}
