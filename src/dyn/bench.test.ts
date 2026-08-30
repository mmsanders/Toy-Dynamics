import { describe, expect, it } from 'vitest';
import { buildModel, NEUTRAL_DOF_PARAMS, type ModelSpec } from './model';
import { makeDynamics } from './forward';
import { makeStepScratch, step, type State } from './integrate';
import { bodySpec, hingeSpec, MASK } from './fixtures';

/**
 * Performance floor for the solver.
 *
 * The tool's promise is that a trajectory recomputes fast enough to feel live, so the cost
 * per step is a real requirement rather than a curiosity. This measures it and prints it,
 * and asserts only a loose ceiling — a CI runner under load is not a benchmark rig, and a
 * tight threshold here would just be a flaky test. The printed number is the useful part.
 */

/** A chain of `n` bodies, each on a revolute joint — the standard scaling shape. */
function chain(n: number, mask: readonly boolean[] = MASK.hingeY): ModelSpec {
  const spec: ModelSpec = { bodies: [], hinges: [], gravity: [0, 0, -9.80665] };
  for (let i = 0; i < n; i++) {
    spec.bodies.push(
      bodySpec({
        name: `L${i}`,
        mass: 1 + i * 0.1,
        com: [0, 0, -0.5],
        inertia: { ixx: 0.1, iyy: 0.12, izz: 0.08, ixy: 0, ixz: 0, iyz: 0 },
      }),
    );
    spec.hinges.push(
      hingeSpec({
        name: `J${i}`,
        parent: i - 1,
        child: i,
        parentNodePos: i === 0 ? [0, 0, 0] : [0, 0, -1],
        free: [...mask],
        values: [0, 0, 0, 0, 0.2, 0],
      }),
    );
  }
  return spec;
}

function timePerStep(spec: ModelSpec, integrator: 'euler' | 'rk4', steps: number): number {
  const model = buildModel(spec);
  const d = makeDynamics(model);
  const scratch = makeStepScratch(model);
  const state: State = { q: Float64Array.from(model.q0), v: Float64Array.from(model.v0) };
  const dt = 1e-3;

  // Warm up, so the measurement is of optimized code rather than the interpreter.
  for (let i = 0; i < 500; i++) step(d, state, i * dt, dt, integrator, scratch);

  const start = performance.now();
  for (let i = 0; i < steps; i++) step(d, state, i * dt, dt, integrator, scratch);
  const elapsed = performance.now() - start;
  return (elapsed / steps) * 1000; // microseconds
}

describe('solver performance', () => {
  it('reports cost per step across model sizes', () => {
    const rows: string[] = [];
    for (const n of [1, 5, 10, 20]) {
      const rk4 = timePerStep(chain(n), 'rk4', 4000);
      const euler = timePerStep(chain(n), 'euler', 4000);
      rows.push(
        `  ${String(n).padStart(2)} bodies (${String(n).padStart(2)} DOF): ` +
          `RK4 ${rk4.toFixed(1).padStart(6)} µs/step   ` +
          `Euler ${euler.toFixed(1).padStart(6)} µs/step   ` +
          `→ 10 s @ dt=1ms: ${((rk4 * 10000) / 1000).toFixed(0)} ms`,
      );
      expect(rk4).toBeGreaterThan(0);
    }

    const freeBodies = timePerStep(chain(5, MASK.free), 'rk4', 2000);
    rows.push(`   5 bodies (30 DOF, all free): RK4 ${freeBodies.toFixed(1)} µs/step`);

    // Static friction, held. A stuck axis is dropped from the system exactly as a locked one
    // is, so holding a joint should cost *less* than letting it move — the claim the README
    // makes, measured rather than asserted.
    const held = chain(10);
    for (const hinge of held.hinges) {
      hinge.params = [0, 1, 2, 3, 4, 5].map((i) =>
        i === 4 ? { ...NEUTRAL_DOF_PARAMS, stiction: 1e6 } : NEUTRAL_DOF_PARAMS,
      );
    }
    const stuck = timePerStep(held, 'rk4', 4000);
    const moving = timePerStep(chain(10), 'rk4', 4000);
    rows.push(
      `  10 bodies, all axes stuck:   RK4 ${stuck.toFixed(1)} µs/step ` +
        `(vs ${moving.toFixed(1)} moving — ${((stuck / moving) * 100).toFixed(0)}%)`,
    );
    // Holding must not be the expensive case.
    expect(stuck).toBeLessThan(moving * 1.1);

    console.log(`\nSolver throughput:\n${rows.join('\n')}\n`);
  });

  it('stays fast enough for a live trajectory at the size this tool targets', () => {
    // 10 bodies is comfortably past a back-of-the-envelope model. A 10 s run at dt = 1 ms
    // must land well inside a second of worker time for the recompute to feel live.
    const perStep = timePerStep(chain(10), 'rk4', 4000);
    const tenSecondRunMs = (perStep * 10_000) / 1000;
    expect(tenSecondRunMs).toBeLessThan(3000);
  });
});
