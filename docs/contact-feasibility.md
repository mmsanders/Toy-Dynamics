# Analytical contact feasibility and development plan

## Implementation status

Phase 1's headless, frictionless solver core is now implemented. Solver specifications can
define body-fixed spheres and any number of one-sided fixed world planes; sphere-plane and
sphere-sphere pairs generate compliant normal forces, multiple contacts accumulate, and the
active set is frozen across Runge-Kutta stages. Canonical model storage, repair, share-link
persistence, model adaptation, and worker-runner transport are also wired. The editor,
visualization, diagnostics, and friction remain Phase 2/3 work, so contact is not yet exposed
in the application UI.

## Decision

Simple contact is feasible without replacing the reduced-coordinate tree solver. The first
version should use **compliant (penalty) contact**: detect penetration from analytic geometry,
turn penetration and closing speed into a normal force, optionally add regularized friction,
and accumulate that force in the existing per-link external-wrench array before RNEA projects
it into generalized coordinates.

This is a good fit for Toy Dynamics because it is small, understandable, and uses the same
explicit-integration tradeoff as the existing compliant travel stops. It is not a rigid-contact
or collision engine: overlap is expected, stiffness is timestep-limited, impacts are resolved
over one or more steps, and stacked bodies may jitter or settle with a small residual
penetration. Those limitations should be stated in the UI and diagnostics.

The recommended first useful slice is **body spheres against fixed, one-sided or two-sided
world planes**, with any number of each. Sphere-sphere contact is a small enough extension to
include immediately. A point is the zero-radius limit of a sphere mathematically, but a tiny
positive radius should be the normal UI representation; an exact radius of zero is useful for
sampling a plane or terrain but has no finite volume and makes contact tolerances more visible.

## Why it fits the current architecture

- Bodies already expose named body-fixed nodes. A sphere can therefore be defined by a body
  and node plus a radius, without adding a second pose system.
- `updateKinematics` and `updateVelocities` already produce link world transforms and spatial
  velocities at every dynamics evaluation. Contact can use those results before inverse
  dynamics is evaluated.
- Actuators already demonstrate the required force path: a force at a point becomes a spatial
  wrench (`r × f`, `f`) in link coordinates, is accumulated per link, and is projected through
  RNEA. Contact should share a general `addForceAtPoint` helper rather than create another
  generalized-force implementation.
- Multiple planes do not create kinematic loops. They are independent force-producing world
  surfaces, so V-shapes, floors/walls, and open boxes are ordinary collections of planes.
- The worker owns a complete run, so contact state such as a warm-start or tangential history
  could be added later without crossing the UI/worker boundary on every step.

The main architectural change is that external forces can no longer be assembled only from
time and actuators. Contact assembly also needs the current kinematics and velocities. The
order in forward dynamics should become:

1. update poses and spatial velocities;
2. build the mass matrix;
3. clear external wrenches, apply actuators, detect contacts, and add contact wrenches;
4. compute joint-local forces and RNEA bias forces;
5. solve for acceleration.

No collision constraint rows need to be added to the mass-matrix solve for the proposed
penalty model.

## Proposed model

Keep collision geometry separate from visual body geometry and from mass properties. A
minimal persisted model is:

```ts
type ContactMaterial = {
  stiffness: number;       // force / length
  damping: number;         // force / speed
  friction: number;        // Coulomb coefficient
};

type ContactSphere = {
  id: string;
  name: string;
  bodyId: string;
  nodeId: string;
  radius: number;
  material: ContactMaterial;
  enabled: boolean;
};

type ContactPlane = {
  id: string;
  name: string;
  point: Vec3;             // world coordinates
  normal: Vec3;            // normalized during model compilation
  side: 'positive' | 'both';
  material: ContactMaterial;
  enabled: boolean;
};
```

For the initial UI, a plane may instead be attached to a Ground node, which gives users the
existing node/orientation editor and makes a plane normal easy to visualize. The compiled
solver representation should still be a world point and unit normal. Moving planes can be a
later feature; body-body sphere contact already covers moving geometry without complicating
plane ownership.

Material combination must be explicit and deterministic. A simple rule is
`k = min(kA, kB)`, `c = min(cA, cB)`, and `mu = min(muA, muB)`. Alternatively, putting one
material on each contact pair is simpler internally but makes authoring many planes tedious.
Whichever rule is chosen should appear next to the fields rather than being hidden.

### Sphere-plane

For sphere centre `x`, radius `r`, plane point `p`, and unit normal `n` pointing into the
allowed half-space:

```text
signed distance  d = n dot (x - p)
penetration      delta = r - d
contact point    xc = x - r n
normal speed     vn = n dot velocity(xc)
normal force     fn = max(0, k delta - c min(vn, 0))
```

The contact is active only when `delta > 0`. Clamping `fn` prevents damping from attracting a
separating body. For a two-sided plane, choose the normal from the sign of the centre distance
and use `abs(d)`; define the zero-distance tie-break deterministically.

### Sphere-sphere

For centres `xA`, `xB`, radii `rA`, `rB`, separation vector `s = xB - xA`, and distance
`L = |s|`, penetration is `rA + rB - L`. The normal is `s/L`; the equal and opposite forces
act at the two surface points. When `L` is effectively zero, reuse the previous normal if
contact state exists, otherwise use a documented fixed axis. Broad phase can initially be an
`O(n^2)` pair loop because this application targets small models. A sweep-and-prune or spatial
hash can be added only if measurement shows it is needed.

Sphere-sphere is therefore nearly free after sphere-plane: it reuses point velocity, wrench
accumulation, materials, and tests. It also makes an exact point primitive unnecessary.

### Friction

Start with regularized sliding friction, not static friction:

```text
vt = relative velocity - normal component
ft = -mu fn tanh(|vt| / vRegularization) normalize(vt)
```

The implementation must handle `|vt| = 0` without normalization. Because this project accepts
arbitrary unit systems, a fixed velocity regularization is undesirable. Expose it as an
advanced setting or derive it from a documented contact velocity scale. Frictionless contact
is a sensible first milestone; regularized friction can follow once normal contact is proven.

True static contact friction is substantially more work than joint stiction. Joint stiction
removes known generalized coordinates, while a contact tangent is a state-dependent direction
coupled to the normal inequality. Correct sticking would require contact constraint rows,
complementarity or an iterative impulse/constraint solve. Do not describe the regularized
penalty law as static friction.

### Impact and integration semantics

Contact changes discontinuously when a pair activates. As with stick/slip, the active set
should be selected once at the beginning of an RK step and held across its stages. Geometry,
penetration, velocity, and force magnitude may still be recomputed at each stage for those
pairs. This avoids different RK stages seeing unrelated force laws. A small hysteresis or
activation tolerance may be needed to stop boundary chatter.

Penalty stiffness introduces a contact natural frequency. A useful conservative diagnostic
is based on the smallest effective contact mass:

```text
omega = sqrt(k / mEffective)
warn when omega * dt > 0.2
```

For a sphere-plane pair, effective mass should ideally be computed along the contact Jacobian
(`1 / (J H^-1 J^T)`), because rotation about an offset contact point matters. The first version
may use a documented conservative body-mass estimate, but should not silently claim it is the
exact contact frequency. Restitution should be deferred: stiffness and damping already define
a compliant bounce, and a second coefficient risks an inconsistent impact model.

## DEM / heightfield contact

A DEM is feasible as a follow-on **if it means a single-valued heightfield**, not an arbitrary
triangle mesh. File loading is not required for the physics. The model can initially accept a
small pasted/generated grid with origin, cell spacing, dimensions, heights, and an up axis;
importing GeoTIFF or other geospatial formats can remain out of scope.

At a sphere centre, locate the grid cell and bilinearly interpolate height. The two analytic
height derivatives give the local surface normal. Sphere-heightfield contact can then reuse
the sphere-plane law using the interpolated point and normal. This is cheap and useful for
small spheres, but it is an approximation: sampling only under the centre can miss a nearby
ridge or cell boundary that intersects a large sphere. The initial feature should document
that the sphere radius should not greatly exceed the grid spacing.

Robust finite-radius sphere/heightfield contact would require searching overlapping cells and
finding the closest point on their triangles (or another bounded surface representation).
That is a separate medium-sized feature, as are holes/no-data values, coordinate reference
systems, huge raster streaming, and file parsing. None should block planes.

Recommended DEM scope:

1. Keep the core contact API expressed as a query returning separation, normal, surface point,
   and surface velocity. Planes and heightfields can then feed the same force law.
2. After primitive contact ships, add a small in-memory height grid with bilinear sampling,
   bounds handling (`no contact` outside the grid), and explicit length units inherited from
   the model.
3. Treat paste/JSON or procedural generation as the first authoring path. Consider file import
   only after the contact behavior is useful and the desired source format is known.

This adds moderate core complexity but potentially substantial UI and persistence work. It
should not be included in the first contact milestone.

## Development plan

### Phase 0 — lock the behavior with a short design decision

- Confirm that interpenetrating, compliant contact is acceptable and that perfectly rigid
  contact, static contact friction, and high-speed collision detection are explicitly out of
  scope.
- Choose one-sided plane semantics, material combination, default stiffness/damping, and a
  scale-aware activation tolerance.
- Decide whether contact geometry is stored in dedicated collections (recommended) or as body
  shape metadata. Reserve append-only fields in the share-link format and repair defaults so
  old saved models remain valid.

### Phase 1 — headless frictionless primitives

- Add canonical and compiled types for spheres, planes, and materials.
- Refactor actuator wrench assembly into allocation-free `clearExternalForces` and
  `addForceAtPoint` helpers.
- Add world position and point-velocity helpers with tests for translated and rotating links.
- Implement sphere-plane detection/normal force and multiple-plane accumulation.
- Implement sphere-sphere pairs, equal-and-opposite wrenches, self-pair filtering, and the
  coincident-centre fallback.
- Freeze the active pair set across RK stages. Ensure failed/diverged runs report the same way
  as today.

Acceptance tests should include: free fall onto a plane without tunnelling at the documented
timestep; static equilibrium penetration near `mg/k`; no tensile normal force while
separating; force and moment from an off-centre hit; equal and opposite sphere-pair forces;
two planes forming a V; energy loss with damping; and unchanged actuator/dynamics baselines
when no contact geometry exists.

### Phase 2 — persistence, diagnostics, and visualization

- Extend store repair, local persistence, share encoding/decoding, worker messages, model
  adapter, and passive-energy classification. Contact damping/friction makes a run non-passive;
  frictionless undamped compliant contact stores recoverable spring energy, which must either
  be included in total energy or conservatively mark the run non-passive.
- Add a Contact panel with sphere node/radius and plane point/normal/material fields. Permit
  duplicate or arbitrary planes so users can build a V or box.
- Render translucent plane patches and sphere wireframes independently of body visibility;
  optionally render active contact points/normals during playback.
- Add validation for negative radius, zero plane normal, negative material values, initial
  penetration, excessive `sqrt(k/mEffective) * dt`, and likely tunnelling (`speed * dt` large
  relative to radius/contact length scale).
- Update CSV/readouts only if contact force output is included; do not silently change the
  existing frame stride without updating worker transfer and consumers.

### Phase 3 — sliding friction and quality improvements

- Add regularized kinetic friction and tests for a sliding sphere/block surrogate slowing
  monotonically without injecting energy.
- Add activation hysteresis and optional per-pair state only if tests demonstrate chatter.
- Measure pair-loop cost and add broad phase only if necessary.
- Consider event subdivision or swept sphere-plane detection for fast impacts. Continuous
  sphere-plane time of impact is analytic, but integrating exactly to each event is a larger
  runner change and should not be implied by the initial feature.

### Phase 4 — optional heightfield

- Introduce the surface-query interface and an in-memory regular height grid.
- Add bilinear height/gradient sampling, edge/no-data behavior, visualization, repair, and
  compact persistence with practical grid-size limits.
- Test flat grids against planes, constant-slope grids against tilted planes, cell-boundary
  continuity, out-of-bounds behavior, and a sphere traversing a smooth procedural hill.
- Only then evaluate triangle-cell closest-point contact and external file formats.

## Effort and risk assessment

| Scope | Relative effort | Main risk |
|---|---:|---|
| Frictionless sphere-plane core | Small–medium | Explicit-step stability and RK activation semantics |
| Multiple planes | Small | UI clarity and one-/two-sided semantics |
| Sphere-sphere | Small after sphere-plane | Coincident centres and pair scaling |
| Visualization, persistence, diagnostics | Medium | Touches most application seams and share compatibility |
| Regularized sliding friction | Small–medium | Unit-independent smoothing and low-speed chatter |
| True sticking / rigid contact | Large | Requires a new constraint/impulse solve |
| Basic in-memory heightfield | Medium | Authoring/persistence and finite-radius approximation |
| Robust DEM and file import | Large | Raster formats, scale, no-data/CRS, and closest-surface search |

The proposal is therefore **go** for spheres, planes, sphere-sphere, and multiple-plane scenes,
provided the feature is presented as compliant contact. A basic programmatic or pasted-grid
heightfield is compatible with the same model later, but should be a separate milestone.
Rigid non-penetration, static friction, mesh contact, and production-grade DEM import would
change the character and complexity of the solver and are not recommended for the initial
implementation.
