import {
  type KinematicsScratch,
  type MultibodyModel,
  makeKinematicsScratch,
  updateKinematics,
  updateVelocities,
} from './model';
import { type CrbaScratch, type Factorization, crba, factorize, makeCrbaScratch, makeFactorization, solveFactorized } from './crba';
import { type RneaScratch, kineticEnergy, makeRneaScratch, rnea } from './rnea';
import { type SV, type V3, sv, v3 } from './spatial';

/**
 * Forward dynamics: from a state to its accelerations.
 *
 * The equation solved is the ordinary one,
 *
 *     H(q)·q̈ = τ − C(q, q̇)
 *
 * where `C` carries Coriolis, centrifugal, gravity *and* the actuator contributions
 * (folded in as external forces during the inverse-dynamics sweep), and `τ` carries the
 * joint-local effects — springs, dampers, friction and travel stops.
 *
 * In reduced coordinates on a tree this is Kane's method: the generalized speeds are the
 * joint rates, and the columns of each joint's motion subspace are the partial velocities
 * that project applied forces onto them. `Sᵀ·f` in the inverse-dynamics sweep is precisely
 * the generalized-force projection Kane's equations call for.
 */

/**
 * Velocity scale over which *sliding* friction reverses.
 *
 * True Coulomb friction is discontinuous at zero, which an explicit integrator turns into
 * chatter — the force flips sign every step and pumps energy in. Regularizing over a small
 * velocity is the standard fix. On its own this models sliding friction only; holding a
 * loaded joint still is what the stick/slip machinery below is for.
 */
const FRICTION_VELOCITY_SCALE = 1e-3;

/**
 * Bound on how many axes may break free in one evaluation.
 *
 * Releasing one axis changes the load on every other, so the set has to be settled by
 * iteration. In practice one or two passes is the whole story; the cap only exists so a
 * pathological model cannot spin here.
 */
const MAX_BREAKAWAY_PASSES = 8;

/** Damping ratio applied at a travel stop, as a fraction of critical for the local inertia. */
const LIMIT_DAMPING_RATIO = 1.0;

export type Dynamics = {
  model: MultibodyModel;
  /** Mass matrix, nv × nv row-major. Refreshed by every `forwardDynamics` call. */
  H: Float64Array;
  /** Bias forces from the last evaluation. */
  C: Float64Array;
  /** Joint-local generalized forces from the last evaluation. */
  tau: Float64Array;
  factorization: Factorization;
  fext: (SV | null)[];
  fextStorage: SV[];
  kin: KinematicsScratch;
  rneaScratch: RneaScratch;
  crbaScratch: CrbaScratch;
  scratchV: V3;
  /** True when the last solve hit a non-positive pivot. */
  singular: boolean;
  singularDof: number;

  /**
   * 1 where an axis is currently held motionless by static friction.
   *
   * Frozen for the duration of an integrator step — see `forwardDynamics`'s `settleDt`.
   */
  stuck: Uint8Array;
  /**
   * True when any axis has a breakaway force set.
   *
   * When false every line of the stick/slip machinery is skipped and the solve is the
   * plain full-rank one, so a model that does not use static friction pays nothing for it.
   */
  hasStiction: boolean;
  /** Force each stuck axis is carrying, from the last solve. Compared against breakaway. */
  holdForce: Float64Array;

  /** Indices of the axes still free to accelerate, and the reduced system built from them. */
  freeIndex: Int32Array;
  reducedH: Float64Array;
  reducedRhs: Float64Array;
  reducedAcc: Float64Array;
  reducedFactor: Factorization;
  /** Right-hand side, held across the breakaway passes so no pass allocates. */
  rhsBuffer: Float64Array;

  /** Cached sphere world centre positions, centre velocities and angular velocities. */
  spherePosition: V3[];
  sphereVelocity: V3[];
  sphereAngularVelocity: V3[];
  /** Scratch for a contact point expressed in link coordinates. */
  scratchP: V3;
  /** Plane pairs first, then sphere pairs in stable nested-loop order. */
  activeContact: Uint8Array;
  contactSetInitialized: boolean;
};

export function makeDynamics(model: MultibodyModel): Dynamics {
  const nv = model.nv;
  const sphereCount = model.contactSpheres.length;
  const contactPairCount = sphereCount * model.contactPlanes.length + (sphereCount * (sphereCount - 1)) / 2;
  return {
    model,
    H: new Float64Array(nv * nv),
    C: new Float64Array(nv),
    tau: new Float64Array(nv),
    factorization: makeFactorization(nv),
    fext: model.links.map(() => null),
    fextStorage: model.links.map(() => sv()),
    kin: makeKinematicsScratch(),
    rneaScratch: makeRneaScratch(model),
    crbaScratch: makeCrbaScratch(model),
    scratchV: v3(),
    singular: false,
    singularDof: -1,

    stuck: new Uint8Array(nv),
    hasStiction: model.dofBindings.some((binding) => binding.params.stiction > 0),
    holdForce: new Float64Array(nv),
    freeIndex: new Int32Array(nv),
    reducedH: new Float64Array(nv * nv),
    reducedRhs: new Float64Array(nv),
    reducedAcc: new Float64Array(nv),
    reducedFactor: makeFactorization(nv),
    rhsBuffer: new Float64Array(nv),
    spherePosition: model.contactSpheres.map(() => v3()),
    sphereVelocity: model.contactSpheres.map(() => v3()),
    sphereAngularVelocity: model.contactSpheres.map(() => v3()),
    scratchP: v3(),
    activeContact: new Uint8Array(contactPairCount),
    contactSetInitialized: false,
  };
}

function clearExternalForces(d: Dynamics): void {
  for (let i = 0; i < d.fext.length; i++) d.fext[i] = null;
  for (const store of d.fextStorage) store.fill(0);
}

/** Add a world-frame force at a link-frame point to the link's external wrench. */
function addForceAtPoint(d: Dynamics, linkIndex: number, point: V3, worldForce: V3): void {
  const link = d.model.links[linkIndex]!;
  const e = link.Xworld.E;
  const x = e[0]! * worldForce[0]! + e[1]! * worldForce[1]! + e[2]! * worldForce[2]!;
  const y = e[3]! * worldForce[0]! + e[4]! * worldForce[1]! + e[5]! * worldForce[2]!;
  const z = e[6]! * worldForce[0]! + e[7]! * worldForce[1]! + e[8]! * worldForce[2]!;
  const f = d.fextStorage[linkIndex]!;
  f[0] = f[0]! + point[1]! * z - point[2]! * y;
  f[1] = f[1]! + point[2]! * x - point[0]! * z;
  f[2] = f[2]! + point[0]! * y - point[1]! * x;
  f[3] = f[3]! + x;
  f[4] = f[4]! + y;
  f[5] = f[5]! + z;
  d.fext[linkIndex] = f;
}

/**
 * Turn the actuators into per-link spatial forces, in link coordinates.
 *
 * A force applied at a point carries a moment about the link origin, hence the `r × f`.
 * A moment is a free vector and gets no such term — which is exactly the distinction
 * between the two actuator kinds.
 */
function applyActuators(d: Dynamics, t: number): void {
  const { model } = d;
  if (model.actuators.length === 0) return;

  for (const act of model.actuators) {
    const scale = act.profile(t);
    if (scale === 0 || !Number.isFinite(scale)) continue;

    const link = model.links[act.link]!;
    let x = act.vector[0]! * scale;
    let y = act.vector[1]! * scale;
    let z = act.vector[2]! * scale;

    if (act.frame === 'world') {
      // Xworld maps world → link, which is the rotation a world-fixed direction needs.
      const e = link.Xworld.E;
      const wx = x, wy = y, wz = z;
      x = e[0]! * wx + e[1]! * wy + e[2]! * wz;
      y = e[3]! * wx + e[4]! * wy + e[5]! * wz;
      z = e[6]! * wx + e[7]! * wy + e[8]! * wz;
    }

    const f = d.fextStorage[act.link]!;
    if (act.kind === 'moment') {
      f[0] = f[0]! + x;
      f[1] = f[1]! + y;
      f[2] = f[2]! + z;
    } else {
      const px = act.point[0]!, py = act.point[1]!, pz = act.point[2]!;
      f[0] = f[0]! + (py * z - pz * y);
      f[1] = f[1]! + (pz * x - px * z);
      f[2] = f[2]! + (px * y - py * x);
      f[3] = f[3]! + x;
      f[4] = f[4]! + y;
      f[5] = f[5]! + z;
    }
    d.fext[act.link] = f;
  }
}

function updateContactPoints(d: Dynamics): void {
  for (let i = 0; i < d.model.contactSpheres.length; i++) {
    const sphere = d.model.contactSpheres[i]!;
    const link = d.model.links[sphere.link]!;
    const point = sphere.point;
    const e = link.Xworld.E;
    const position = d.spherePosition[i]!;
    // Xworld maps world to link. Its transpose maps the local point to world.
    position[0] = e[0]! * point[0]! + e[3]! * point[1]! + e[6]! * point[2]! + link.Xworld.r[0]!;
    position[1] = e[1]! * point[0]! + e[4]! * point[1]! + e[7]! * point[2]! + link.Xworld.r[1]!;
    position[2] = e[2]! * point[0]! + e[5]! * point[1]! + e[8]! * point[2]! + link.Xworld.r[2]!;

    const wx = link.v[0]!, wy = link.v[1]!, wz = link.v[2]!;
    const localX = link.v[3]! + wy * point[2]! - wz * point[1]!;
    const localY = link.v[4]! + wz * point[0]! - wx * point[2]!;
    const localZ = link.v[5]! + wx * point[1]! - wy * point[0]!;
    const velocity = d.sphereVelocity[i]!;
    velocity[0] = e[0]! * localX + e[3]! * localY + e[6]! * localZ;
    velocity[1] = e[1]! * localX + e[4]! * localY + e[7]! * localZ;
    velocity[2] = e[2]! * localX + e[5]! * localY + e[8]! * localZ;
    const omega = d.sphereAngularVelocity[i]!;
    omega[0] = e[0]! * wx + e[3]! * wy + e[6]! * wz;
    omega[1] = e[1]! * wx + e[4]! * wy + e[7]! * wz;
    omega[2] = e[2]! * wx + e[5]! * wy + e[8]! * wz;
  }
}

/**
 * Apply a world force at a sphere's surface point, given as a world offset from its centre.
 *
 * The normal force acts along the line through the centre, so where it is applied makes no
 * difference to it. Friction is different: it acts at the surface, a radius away from the
 * centre, and that lever arm is what makes a ball roll rather than skid to a halt. Applying
 * it at the centre would be the classic mistake of a sliding block that never spins.
 */
function addForceAtSphereSurface(
  d: Dynamics,
  sphere: { link: number; point: V3 },
  ox: number,
  oy: number,
  oz: number,
  worldForce: V3,
): void {
  const e = d.model.links[sphere.link]!.Xworld.E;
  const p = d.scratchP;
  // Xworld maps world → link, so a world offset lands in link coordinates through E.
  p[0] = sphere.point[0]! + e[0]! * ox + e[1]! * oy + e[2]! * oz;
  p[1] = sphere.point[1]! + e[3]! * ox + e[4]! * oy + e[5]! * oz;
  p[2] = sphere.point[2]! + e[6]! * ox + e[7]! * oy + e[8]! * oz;
  addForceAtPoint(d, sphere.link, p, worldForce);
}

/** Detect and apply compliant sphere-plane and sphere-sphere contact. */
function applyContacts(d: Dynamics, settle: boolean): void {
  const { contactSpheres: spheres, contactPlanes: planes } = d.model;
  if (spheres.length === 0) return;
  updateContactPoints(d);
  const refresh = settle || !d.contactSetInitialized;
  let pair = 0;
  const force = d.scratchV;

  for (let i = 0; i < spheres.length; i++) {
    const sphere = spheres[i]!;
    const position = d.spherePosition[i]!;
    const velocity = d.sphereVelocity[i]!;
    for (const plane of planes) {
      const nx = plane.normal[0]!, ny = plane.normal[1]!, nz = plane.normal[2]!;
      const distance = nx * (position[0]! - plane.point[0]!)
        + ny * (position[1]! - plane.point[1]!)
        + nz * (position[2]! - plane.point[2]!);
      const penetration = sphere.radius - distance;
      if (refresh) d.activeContact[pair] = penetration > 0 ? 1 : 0;
      if (d.activeContact[pair]) {
        // The closing speed is the centre's along the normal; the spin adds nothing there.
        const normalSpeed = nx * velocity[0]! + ny * velocity[1]! + nz * velocity[2]!;
        const stiffness = Math.min(sphere.stiffness, plane.stiffness);
        const damping = Math.min(sphere.damping, plane.damping);
        const magnitude = Math.max(0, stiffness * Math.max(0, penetration) - damping * Math.min(0, normalSpeed));
        // Slip is measured at the surface point, a radius below the centre: v + ω × (−r n).
        const ox = -sphere.radius * nx, oy = -sphere.radius * ny, oz = -sphere.radius * nz;
        const w = d.sphereAngularVelocity[i]!;
        const sx = velocity[0]! + (w[1]! * oz - w[2]! * oy);
        const sy = velocity[1]! + (w[2]! * ox - w[0]! * oz);
        const sz = velocity[2]! + (w[0]! * oy - w[1]! * ox);
        const tx = sx - nx * normalSpeed;
        const ty = sy - ny * normalSpeed;
        const tz = sz - nz * normalSpeed;
        const slip = Math.hypot(tx, ty, tz);
        const friction = Math.min(sphere.friction, plane.friction);
        const frictionScale = slip > 0
          ? -friction * magnitude * Math.tanh(slip / Math.min(sphere.frictionVelocity, plane.frictionVelocity)) / slip
          : 0;
        force[0] = nx * magnitude + tx * frictionScale;
        force[1] = ny * magnitude + ty * frictionScale;
        force[2] = nz * magnitude + tz * frictionScale;
        addForceAtSphereSurface(d, sphere, ox, oy, oz, force);
      }
      pair++;
    }
  }

  for (let i = 0; i < spheres.length; i++) {
    for (let j = i + 1; j < spheres.length; j++) {
      const a = spheres[i]!, b = spheres[j]!;
      const pa = d.spherePosition[i]!, pb = d.spherePosition[j]!;
      const dx = pb[0]! - pa[0]!, dy = pb[1]! - pa[1]!, dz = pb[2]! - pa[2]!;
      const length = Math.hypot(dx, dy, dz);
      const penetration = a.radius + b.radius - length;
      // Spheres on one rigid link cannot move relative to one another and must not collide.
      if (refresh) d.activeContact[pair] = a.link !== b.link && penetration > 0 ? 1 : 0;
      if (d.activeContact[pair]) {
        const invLength = length > 1e-12 ? 1 / length : 0;
        // A deterministic fallback keeps coincident centres finite.
        const nx = length > 1e-12 ? dx * invLength : 1;
        const ny = length > 1e-12 ? dy * invLength : 0;
        const nz = length > 1e-12 ? dz * invLength : 0;
        const va = d.sphereVelocity[i]!, vb = d.sphereVelocity[j]!;
        const normalSpeed = nx * (vb[0]! - va[0]!) + ny * (vb[1]! - va[1]!) + nz * (vb[2]! - va[2]!);
        const stiffness = Math.min(a.stiffness, b.stiffness);
        const damping = Math.min(a.damping, b.damping);
        const magnitude = Math.max(0, stiffness * Math.max(0, penetration) - damping * Math.min(0, normalSpeed));
        // Surface points: A's lies a radius along the normal, B's a radius back along it.
        // The relative slip between those two points is what friction acts on.
        const ax = a.radius * nx, ay = a.radius * ny, az = a.radius * nz;
        const bx = -b.radius * nx, by = -b.radius * ny, bz = -b.radius * nz;
        const wa = d.sphereAngularVelocity[i]!, wb = d.sphereAngularVelocity[j]!;
        const rvx = (vb[0]! + wb[1]! * bz - wb[2]! * by) - (va[0]! + wa[1]! * az - wa[2]! * ay);
        const rvy = (vb[1]! + wb[2]! * bx - wb[0]! * bz) - (va[1]! + wa[2]! * ax - wa[0]! * az);
        const rvz = (vb[2]! + wb[0]! * by - wb[1]! * bx) - (va[2]! + wa[0]! * ay - wa[1]! * ax);
        const tx = rvx - nx * normalSpeed, ty = rvy - ny * normalSpeed, tz = rvz - nz * normalSpeed;
        const slip = Math.hypot(tx, ty, tz);
        const friction = Math.min(a.friction, b.friction);
        const frictionScale = slip > 0
          ? friction * magnitude * Math.tanh(slip / Math.min(a.frictionVelocity, b.frictionVelocity)) / slip
          : 0;
        force[0] = -nx * magnitude + tx * frictionScale;
        force[1] = -ny * magnitude + ty * frictionScale;
        force[2] = -nz * magnitude + tz * frictionScale;
        addForceAtSphereSurface(d, a, ax, ay, az, force);
        force[0] = -force[0]!; force[1] = -force[1]!; force[2] = -force[2]!;
        addForceAtSphereSurface(d, b, bx, by, bz, force);
      }
      pair++;
    }
  }
  if (refresh) d.contactSetInitialized = true;
}

/** Elastic energy stored by the compliant normal springs at the current configuration. */
function contactPotentialEnergy(d: Dynamics): number {
  const { contactSpheres: spheres, contactPlanes: planes } = d.model;
  if (spheres.length === 0) return 0;
  updateContactPoints(d);
  let energy = 0;

  for (let i = 0; i < spheres.length; i++) {
    const sphere = spheres[i]!;
    const position = d.spherePosition[i]!;
    for (const plane of planes) {
      const distance = plane.normal[0]! * (position[0]! - plane.point[0]!)
        + plane.normal[1]! * (position[1]! - plane.point[1]!)
        + plane.normal[2]! * (position[2]! - plane.point[2]!);
      const penetration = Math.max(0, sphere.radius - distance);
      energy += 0.5 * Math.min(sphere.stiffness, plane.stiffness) * penetration * penetration;
    }
  }

  for (let i = 0; i < spheres.length; i++) {
    for (let j = i + 1; j < spheres.length; j++) {
      const a = spheres[i]!, b = spheres[j]!;
      if (a.link === b.link) continue;
      const pa = d.spherePosition[i]!, pb = d.spherePosition[j]!;
      const distance = Math.hypot(pb[0]! - pa[0]!, pb[1]! - pa[1]!, pb[2]! - pa[2]!);
      const penetration = Math.max(0, a.radius + b.radius - distance);
      energy += 0.5 * Math.min(a.stiffness, b.stiffness) * penetration * penetration;
    }
  }
  return energy;
}

/**
 * Joint-local generalized forces: springs, dampers, Coulomb friction and travel stops.
 *
 * Stops are penalty springs rather than hard constraints — a hard stop in reduced
 * coordinates would need an impact law and a event-driven integrator, which is well past
 * what a back-of-the-envelope tool should carry. The penalty is damped against the actual
 * diagonal of the mass matrix, so the stop is roughly critically damped whatever the body
 * masses are, instead of ringing or exploding depending on the model's scale.
 */
function jointForces(d: Dynamics, q: Float64Array, v: Float64Array): void {
  const { model, tau, H } = d;
  const nv = model.nv;
  tau.fill(0);

  for (let i = 0; i < nv; i++) {
    const binding = model.dofBindings[i]!;
    const p = binding.params;
    const rate = v[i]!;
    let force = 0;

    if (p.damping !== 0) force -= p.damping * rate;
    // Sliding friction is skipped on a stuck axis: there the friction force is not a known
    // quantity but the unknown the constraint solves for, and adding a sliding term as well
    // would double-count it and corrupt the breakaway test.
    if (p.friction !== 0 && !d.stuck[i]) {
      force -= p.friction * Math.tanh(rate / FRICTION_VELOCITY_SCALE);
    }

    if (binding.qIndex >= 0) {
      const value = q[binding.qIndex]!;
      if (p.stiffness !== 0) force -= p.stiffness * (value - p.rest);

      if (p.limitEnabled && p.limitStiffness > 0) {
        let penetration = 0;
        if (value < p.limitLo) penetration = value - p.limitLo;
        else if (value > p.limitHi) penetration = value - p.limitHi;
        if (penetration !== 0) {
          const effectiveMass = Math.max(H[i * nv + i]!, 1e-12);
          const damping = 2 * LIMIT_DAMPING_RATIO * Math.sqrt(p.limitStiffness * effectiveMass);
          force -= p.limitStiffness * penetration + damping * rate;
        }
      }
    }

    tau[i] = force;
  }
}

// ---------------------------------------------------------------------------
// Static friction
// ---------------------------------------------------------------------------

/**
 * Which axes are slow enough for static friction to arrest them this step.
 *
 * The threshold is derived rather than picked: `μ/H_ii · dt` is exactly the velocity change
 * the breakaway force can produce in one step, so an axis moving slower than that would be
 * stopped within the step anyway. That makes the criterion scale-free — it needs no absolute
 * velocity constant, and so it means the same thing whether the model is in millimetres or
 * kilometres, which matters in a tool that enforces no units.
 */
function stickCandidates(d: Dynamics, v: Float64Array, dt: number): void {
  const nv = d.model.nv;
  for (let i = 0; i < nv; i++) {
    if (d.stuck[i]) continue;
    const breakaway = d.model.dofBindings[i]!.params.stiction;
    if (breakaway <= 0) continue;
    const effectiveMass = Math.max(d.H[i * nv + i]!, 1e-30);
    if (Math.abs(v[i]!) <= (breakaway / effectiveMass) * dt) d.stuck[i] = 1;
  }
}

/**
 * Solve `H·q̈ = rhs` with the stuck axes pinned to zero acceleration.
 *
 * This is where reduced coordinates pay off twice. A stuck axis is simply dropped from the
 * system — the same thing a locked axis is — so holding a joint still costs *less* than
 * letting it move, not more. And the force it has to carry to stay still falls straight out
 * of its own row afterwards:
 *
 *     H·q̈ + C = τ + f_hold   ⟹   f_hold = (H·q̈) − rhs
 *
 * which is the quantity the breakaway test needs. No penalty spring, no extra stiffness, and
 * nothing that forces a smaller timestep.
 */
function solveConstrained(d: Dynamics, rhs: Float64Array, qdd: Float64Array): void {
  const nv = d.model.nv;

  let nf = 0;
  for (let i = 0; i < nv; i++) if (!d.stuck[i]) d.freeIndex[nf++] = i;

  if (nf === nv) {
    factorize(d.H, d.factorization);
    d.singular = d.factorization.failedAt >= 0;
    d.singularDof = d.factorization.failedAt;
    solveFactorized(d.factorization, rhs, qdd);
    return;
  }

  for (let a = 0; a < nf; a++) {
    const row = d.freeIndex[a]! * nv;
    for (let b = 0; b < nf; b++) d.reducedH[a * nf + b] = d.H[row + d.freeIndex[b]!]!;
    d.reducedRhs[a] = rhs[d.freeIndex[a]!]!;
  }

  // The factorization's buffers are sized for the full system; narrowing `n` reuses them
  // for the reduced one without allocating.
  d.reducedFactor.n = nf;
  factorize(d.reducedH, d.reducedFactor);
  d.singular = d.reducedFactor.failedAt >= 0;
  d.singularDof = d.singular ? (d.freeIndex[d.reducedFactor.failedAt]! ?? -1) : -1;
  solveFactorized(d.reducedFactor, d.reducedRhs, d.reducedAcc);

  qdd.fill(0);
  for (let a = 0; a < nf; a++) qdd[d.freeIndex[a]!] = d.reducedAcc[a]!;

  for (let i = 0; i < nv; i++) {
    if (!d.stuck[i]) {
      d.holdForce[i] = 0;
      continue;
    }
    let acc = 0;
    const row = i * nv;
    for (let a = 0; a < nf; a++) acc += d.H[row + d.freeIndex[a]!]! * d.reducedAcc[a]!;
    d.holdForce[i] = acc - rhs[i]!;
  }
}

/** Zero the velocity of every stuck axis, so a held joint holds exactly rather than creeping. */
export function zeroStuckVelocities(d: Dynamics, v: Float64Array): void {
  if (!d.hasStiction) return;
  for (let i = 0; i < d.model.nv; i++) if (d.stuck[i]) v[i] = 0;
}

/**
 * Evaluate `q̈` for a state.
 *
 * Order matters: kinematics and velocities first (both the mass matrix and the bias forces
 * read them), then H, then the joint forces — which need H's diagonal for the stop damping
 * — then the bias forces, then the solve.
 *
 * `settleDt` is the integrator's step when this evaluation is allowed to *change* which axes
 * are stuck, and zero when it must reuse the set it was given. Stick and slip are decided
 * once per step and then held for every stage of it: a set that changed between stages would
 * make the derivative function discontinuous mid-step, which is precisely the thing a
 * Runge-Kutta method assumes never happens. The cost is that a transition lands one step
 * late, which at this fidelity is not worth chasing.
 */
export function forwardDynamics(
  d: Dynamics,
  q: Float64Array,
  v: Float64Array,
  t: number,
  qdd: Float64Array,
  settleDt = 0,
): void {
  const { model } = d;
  const nv = model.nv;
  if (nv === 0) return;

  updateKinematics(model, q, v, d.kin);
  updateVelocities(model, d.kin);
  crba(model, d.H, d.crbaScratch);
  clearExternalForces(d);
  applyActuators(d, t);
  applyContacts(d, settleDt > 0);

  // Fast path: no breakaway forces anywhere, so none of the machinery below can apply.
  if (!d.hasStiction) {
    jointForces(d, q, v);
    rnea(model, null, d.fext, d.C, d.rneaScratch);
    const rhs = qdd;
    for (let i = 0; i < nv; i++) rhs[i] = d.tau[i]! - d.C[i]!;
    factorize(d.H, d.factorization);
    d.singular = d.factorization.failedAt >= 0;
    d.singularDof = d.factorization.failedAt;
    solveFactorized(d.factorization, rhs, qdd);
    return;
  }

  if (settleDt > 0) stickCandidates(d, v, settleDt);

  // The bias forces and actuator loads do not depend on which axes are stuck, so they are
  // computed once and reused across the breakaway passes; only the joint-local forces and
  // the solve are repeated.
  rnea(model, null, d.fext, d.C, d.rneaScratch);

  const rhs = d.rhsBuffer;
  for (let pass = 0; ; pass++) {
    jointForces(d, q, v);
    for (let i = 0; i < nv; i++) rhs[i] = d.tau[i]! - d.C[i]!;
    solveConstrained(d, rhs, qdd);

    if (settleDt <= 0) return;

    // Release the axis most over its breakaway force, then re-solve: letting one go changes
    // the load on the rest, so they cannot all be judged from a single solve.
    let worst = -1;
    let excess = 0;
    for (let i = 0; i < nv; i++) {
      if (!d.stuck[i]) continue;
      const over = Math.abs(d.holdForce[i]!) - d.model.dofBindings[i]!.params.stiction;
      if (over > excess) {
        excess = over;
        worst = i;
      }
    }
    if (worst < 0 || pass >= MAX_BREAKAWAY_PASSES) return;
    d.stuck[worst] = 0;
  }
}

// ---------------------------------------------------------------------------
// Energy
// ---------------------------------------------------------------------------

/**
 * Total mechanical energy.
 *
 * The zero of potential energy is arbitrary — it is measured against the world origin —
 * which is fine, because only its *drift* is meaningful. A passive model whose energy
 * wanders is a model whose timestep is too big, and that is the check this feeds.
 */
export function totalEnergy(d: Dynamics, q: Float64Array, v: Float64Array): { kinetic: number; potential: number; total: number } {
  const { model } = d;
  updateKinematics(model, q, v, d.kin);
  updateVelocities(model, d.kin);

  const kinetic = kineticEnergy(model, d.rneaScratch);

  let potential = 0;
  const g = model.gravity;
  for (const link of model.links) {
    if (link.mass === 0) continue;
    // The link's centre of mass in link coordinates is h/m; push it out to world.
    const cx = link.I.h[0]! / link.I.m;
    const cy = link.I.h[1]! / link.I.m;
    const cz = link.I.h[2]! / link.I.m;
    const e = link.Xworld.E;
    // Xworld maps world → link, so link → world is its transpose.
    const wx = e[0]! * cx + e[3]! * cy + e[6]! * cz + link.Xworld.r[0]!;
    const wy = e[1]! * cx + e[4]! * cy + e[7]! * cz + link.Xworld.r[1]!;
    const wz = e[2]! * cx + e[5]! * cy + e[8]! * cz + link.Xworld.r[2]!;
    potential -= link.I.m * (g[0]! * wx + g[1]! * wy + g[2]! * wz);
  }

  potential += contactPotentialEnergy(d);

  return { kinetic, potential, total: kinetic + potential };
}
