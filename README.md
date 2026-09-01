# Toy Dynamics

A calculator-style multibody 6DOF simulator that runs entirely in the browser. Build a small
model out of rigid bodies, connect them with hinges, push on them with actuators, and read
the numbers back — so you can sanity-check a more serious simulation tool before you trust
what it told you.

Live at **https://mmsanders.github.io/Toy-Dynamics/**

Works on desktop and on a phone: the 3D view fills the screen, and the controls are a drag-up
sheet on mobile or a docked sidebar on a wide screen.

## What it is for

This is deliberately a back-of-the-envelope tool. It trades fidelity for speed and for being
*legible* — you should be able to see exactly what it did and why. That only works if you can
also see when it has stopped being trustworthy, so a large part of it is devoted to saying so.

Contact is deliberately simple: body-fixed **spheres** against fixed **world planes** and
against each other, with a compliant (penalty) normal force and regularized sliding friction.
Bodies have no other shape, so anything that is not a sphere passes through everything. There
is no rigid non-penetration, no static sticking and no swept collision detection; the
[contact development plan](docs/contact-feasibility.md) spells out what was chosen and why.

## The model

**Bodies** are nodes and mass properties, and nothing else. A body has no shape here; it is a
set of named points plus a mass and an inertia tensor. Two of those nodes are special: one
defines the body's frame origin, and one is the centre of mass. They need not be the same
point, and on most real bodies they are not.

**Inertia can be stated about either of them**, and the panel always shows both readings — the
one you are editing, and what the same body looks like from the other point. Two operations
are offered and they are not the same thing:

| | What changes | What stays |
|---|---|---|
| **Toggle** the reference | what the numbers *mean* | the numbers |
| **Convert** to the other reference | the numbers | the physical body |

**Hinges** connect a node on one body to a node on another and carry a mount orientation. Each
hinge has six axes — three translational, three rotational — and every one is independently
free or locked. That single control covers every joint type: one free rotation is a revolute,
one free translation is a telescoping pole, three rotations is a ball joint, all six is a free-
flying body, none is a weld.

Each free axis can also carry a spring, a damper, travel limits, and friction — both
**sliding** friction and a **breakaway (static)** force.

Static friction is a real stick/slip constraint rather than a stiff spring. Below the
breakaway force the axis is held perfectly still by dropping out of the equations, exactly
the way a locked axis does — so it holds *exactly*, with no creep, and costs less than
letting it move rather than more. Above it, the axis breaks free and sliding friction takes
over. The one cost is that stick and slip are decided once per step and held across the
integrator's stages, so a transition lands one step late; letting the set shift mid-step
would make the derivative discontinuous, which is the thing Runge-Kutta assumes never
happens.

**Actuators** attach to a node and apply either a pure force or a pure moment — two different
things, drawn differently, because a force away from the centre of mass also spins the body
and a moment does not. Each one is body-fixed or world-fixed: a thruster bolted to a tumbling
body sweeps its thrust around with it, and one pointing a fixed way in space does not.
Identical numbers, completely different motion.

Actuators run on a time profile — constant, step, ramp, sine, impulse — or on an arbitrary
`f(t)` expression when none of those fits.

**Spring-dampers** are passive two-node devices, edited beside actuators. They connect nodes
on different bodies (including Ground as a fixed anchor), apply equal-and-opposite axial
forces, and expose stiffness, damping, and rest length. Their elastic energy is included in
the run's energy readout; damping correctly removes energy from the system.

## The dynamics

Reduced coordinates over a tree, integrated with a hand-written spatial-vector solver. For a
tree this **is** Kane's method: the joint rates are the generalized speeds and the columns of
each joint's motion subspace are the partial velocities that project applied forces onto them.

The central consequence is worth stating plainly: **a locked axis is not a constraint the
solver has to satisfy — it simply is not a coordinate.** Locks are therefore exact, constraint
drift cannot happen, and a tree can never be over-constrained. That is why the topology is a
tree: closing a loop is refused rather than approximated.

Forward dynamics is CRBA for the mass matrix, RNEA for the bias forces, and an LDLᵀ solve.
The articulated-body algorithm is asymptotically faster but never materializes **H**, and H is
what the diagnostics are built on — a non-positive pivot names the exact coordinate at which
the model stopped being solvable. At the tens of DOF this tool is for, the two cost about the
same.

Three integrators: RK4 (the default), RK2, and semi-implicit Euler. None is implicit, so a very
stiff spring or a hard travel stop still needs a small step — which the diagnostics will tell
you before you waste a run finding out.

### Measured speed

Indicative, from `npm test`'s benchmark on one machine — expect these to move by up to about
1.5× either way with hardware and load. What matters is the shape: cost grows with DOF, RK4
costs roughly 4× semi-implicit Euler, and a model of this size recomputes in well under a
second.

| Model | RK4 | Euler | 10 s run at dt = 1 ms |
|---|---|---|---|
| 1 body, 1 DOF | 8.8 µs/step | 4.9 µs/step | 88 ms |
| 5 bodies, 5 DOF | 29 µs/step | 7.1 µs/step | 290 ms |
| 10 bodies, 10 DOF | 71 µs/step | 17.5 µs/step | 714 ms |
| 20 bodies, 20 DOF | 199 µs/step | 48 µs/step | 2.0 s |

Holding a joint with static friction is measured at ~98% the cost of letting it move: a
stuck axis leaves the system rather than joining it.

Everything runs in the browser. There is no server and nothing to install.

Trajectories integrate in a Web Worker and stream back as they are computed, so editing never
waits on simulating. The 3D view meanwhile draws the initial pose — pure kinematics,
microseconds — which is what lets dragging a slider update the view immediately with the
trajectory catching up behind it.

## Units are not enforced

The solver is pure arithmetic. It never converts anything, and keeping your numbers mutually
consistent is your job. Picking a unit system changes exactly three things:

1. the labels on the fields,
2. the gravity preset,
3. which plausibility checks are armed.

**It never rewrites a value.** The one exception is gravity, and only while it is still sitting
at the previous system's standard — meaning you never touched it.

Generic mode assumes nothing at all and labels fields with dimension symbols (`[M]`, `[L]`,
`[F]`). Imperial labels mass as **slug**, not pound: with force in lbf, length in ft and time
in s, mass must be in slugs, and entering pounds-mass is wrong by a factor of 32.174. There is
a one-tap converter on the mass field rather than a heuristic that guesses.

## When things get nasty

Every check is **advisory and never blocking**. A tool for sanity-checking has to let you run
an absurd model and see what happens; sometimes that *is* the question. Where there is an
obvious correction it is offered as one tap.

Most checks are structural or built from dimensionless ratios, so they work in any unit system:

- **A degenerate mass matrix**, naming the exact coordinate the LDLᵀ factorization failed at,
  and a separate warning when it is merely close.
- **Non-physical inertia** — a negative principal moment, or principal moments violating the
  triangle inequality `I₁ + I₂ ≥ I₃`, which no real mass distribution can do.
- **Radius of gyration against body size.** `√(I/m)` compared to how far the body's nodes
  actually spread. Off by orders of magnitude is the fingerprint of an inertia entered in the
  wrong units — g·mm² for kg·m², lbm·in² for slug·ft².
- **Timestep against the stiffest spring.** `ωₙ·dt` above ~0.2 and the motion is not being
  resolved; the warning names a step that would work. Travel stops are the usual culprit.
- **Energy drift.** When nothing in the model can add or remove energy, total energy must be
  constant — so any drift is integration error, and the Run tab reports how much.
- **Divergence.** A non-finite state stops the run and says when, keeping everything computed
  up to that point.
- Gimbal lock on a two-rotation hinge, a fully locked model, and springs set on a ball joint
  (which has no single angle for one to act against).

Two checks need a named unit system and are skipped entirely in Generic:

- **Gravity in the wrong system.** Deliberately not a range check — Moon and Mars gravity are
  legitimate values. It looks for the *other* system's constant: 32.17 while you are in SI is
  almost certainly ft/s². You are offered both fixes, rescaling the value or switching systems,
  because only you know which half of the model is wrong.
- Order-of-magnitude typos in masses, lengths and inertias.

## Reading the results

- **Time-history plots** of joint angles, rates and energy, with a checkbox per series. One
  unit per chart and never two y-scales — angles and offsets get separate charts, because
  radians and metres do not share an axis.
- **A live numeric readout** at any node of any body: position, velocity, angular velocity,
  energy and total momentum, tap-to-copy. With no external forces both momenta are conserved
  exactly, so watching them wander is the most direct check that a run is still honest.
- **CSV export**, one button, everything in it: time, every generalized coordinate and speed,
  every body pose and velocity, every node position, energy and momentum. No configuration —
  the column you need is already there.
- **A playback scrubber** over the precomputed frames.

## Sharing

The Setup tab copies a link containing the whole model, so a setup moves between phone and
desktop. Importing never destroys what you had: the replaced model sits behind an Undo, and
the hash is cleared so a refresh cannot re-import over later edits. Malformed links degrade to
a working model rather than a blank screen.

## How it is checked

Being checkable is the whole point of the tool, so the solver is pinned against results that
have an answer independent of it — a closed form, a conservation law, or a second algorithm
computing the same quantity a different way:

- a pendulum's period against small-angle theory, and its lengthening at large amplitude;
- energy conservation under RK4, and a chaotic double pendulum matched against a reference run
  at a 40× smaller step;
- angular momentum and energy conservation for a torque-free body, and the **Dzhanibekov
  intermediate-axis flip** — which appears on its own, from the equations, rather than being
  put there;
- closed-form motion for a prismatic joint under constant force, and `√(k/m)` for a spring;
- a stuck joint holding a sub-breakaway load with *bit-exact* zero motion, breaking free at
  the threshold, and a sliding joint coming to rest at the closed-form `v₀²/(2a)`;
- the same body entered CoM-relative and origin-relative producing identical motion;
- the mass matrix from CRBA against inverse dynamics, column by column;
- `F = ma` and `τ = Iα` for actuators, and a body-fixed thruster's cycloid against its closed
  form;
- **the joint motion subspace and its apparent derivative against finite differences of the
  joint transform, for all 64 free/locked masks** — the check that catches the errors which
  are invisible on a single-DOF joint and silently wrong the moment two axes move together.

Run them with `npm test`. There is also a benchmark that prints the numbers in the table above,
so the performance claims stay honest.

## Development

```bash
npm install
npm run dev          # dev server
npm run typecheck    # tsc
npm run lint         # eslint
npm test             # vitest — physics, model, simulation, share links
npm run test:e2e     # playwright, desktop + phone viewports
npm run build        # production build
```

## Provenance

The UI kit — the design system, `NumberField`, `Segmented`, the bottom sheet, the scene
primitives, and the rotation-convention handling — is copied from
[Rotation Wizard](https://github.com/mmsanders/Rotation-wizard), which is a separate app and is
not a dependency. The two are independent; nothing here changes anything there.

All rotation maths at the presentation boundary is delegated to three.js. The solver's spatial
algebra is its own, because it needs to be allocation-free in an integrator's inner loop.
