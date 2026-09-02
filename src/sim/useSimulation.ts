import { useEffect, useMemo, useRef, useState } from 'react';
import { useModelStore } from '../store/useModelStore';
import type { RunProgress, TrajectoryMeta } from './runner';
import type { WorkerRequest, WorkerResponse } from '../worker/sim.worker';

/**
 * Driving the trajectory worker from React.
 *
 * The shape of this hook follows one requirement: **editing must never wait on
 * simulating.** So the trajectory is always a background result that arrives late, and
 * nothing in the UI blocks on it. The 3D view keeps drawing the previous trajectory, or the
 * initial pose, while a new one is computed.
 *
 * Edits are debounced, because dragging a slider fires a change per pixel and each one
 * would otherwise start and abandon a run. The debounce is short enough that letting go of
 * a slider feels immediate.
 */

/** Long enough to swallow a slider drag, short enough to feel instant on release. */
const DEBOUNCE_MS = 120;

export type Trajectory = {
  meta: TrajectoryMeta;
  data: Float64Array;
  /** Frames actually filled. The tail of `data` is zeroed until the run reaches it. */
  count: number;
  progress: RunProgress;
};

export type SimulationState = {
  trajectory: Trajectory | null;
  /** True from the moment an edit lands until the run finishes. */
  computing: boolean;
  /** Set when the model could not be assembled at all. */
  error: string | null;
};

/**
 * Everything a run depends on.
 *
 * Deliberately excludes selection and visibility: highlighting a body or hiding it must not
 * throw away a trajectory that is still perfectly valid.
 */
function runInput(state: ReturnType<typeof useModelStore.getState>) {
  return {
    bodies: state.bodies,
    hinges: state.hinges,
    actuators: state.actuators,
    springDampers: state.springDampers,
    contactSpheres: state.contactSpheres,
    contactPlanes: state.contactPlanes,
    contactHeightfields: state.contactHeightfields,
    settings: state.settings,
  };
}

export function useSimulation(): SimulationState {
  const bodies = useModelStore((s) => s.bodies);
  const hinges = useModelStore((s) => s.hinges);
  const actuators = useModelStore((s) => s.actuators);
  const springDampers = useModelStore((s) => s.springDampers);
  const contactSpheres = useModelStore((s) => s.contactSpheres);
  const contactPlanes = useModelStore((s) => s.contactPlanes);
  const contactHeightfields = useModelStore((s) => s.contactHeightfields);
  const settings = useModelStore((s) => s.settings);

  const [state, setState] = useState<SimulationState>({
    trajectory: null,
    computing: true,
    error: null,
  });

  const workerRef = useRef<Worker | null>(null);
  const generationRef = useRef(0);
  const pendingRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  /** The run being assembled, kept out of React state so a chunk does not force a render. */
  const buildingRef = useRef<{ generation: number; meta: TrajectoryMeta; data: Float64Array } | null>(null);

  useEffect(() => {
    const worker = new Worker(new URL('../worker/sim.worker.ts', import.meta.url), {
      type: 'module',
    });
    workerRef.current = worker;

    worker.onmessage = (event: MessageEvent<WorkerResponse>) => {
      const message = event.data;
      // Anything from a superseded run is ignored rather than merged.
      if (message.generation !== generationRef.current) return;

      switch (message.kind) {
        case 'started':
          buildingRef.current = {
            generation: message.generation,
            meta: message.meta,
            data: new Float64Array(message.meta.stride * message.meta.frameCount),
          };
          break;

        case 'chunk': {
          const building = buildingRef.current;
          if (!building) break;
          building.data.set(message.data, message.from * building.meta.stride);
          break;
        }

        case 'progress': {
          const building = buildingRef.current;
          if (!building) break;
          setState({
            trajectory: {
              meta: building.meta,
              data: building.data,
              count: message.progress.written,
              progress: message.progress,
            },
            computing: message.progress.status === 'running',
            error: null,
          });
          break;
        }

        case 'failed':
          buildingRef.current = null;
          setState({ trajectory: null, computing: false, error: message.error });
          break;
      }
    };

    return () => {
      worker.terminate();
      workerRef.current = null;
    };
  }, []);

  useEffect(() => {
    const worker = workerRef.current;
    if (!worker) return;

    setState((prev) => ({ ...prev, computing: true }));
    if (pendingRef.current) clearTimeout(pendingRef.current);

    pendingRef.current = setTimeout(() => {
      const generation = ++generationRef.current;
      const request: WorkerRequest = {
        kind: 'run',
        generation,
        // Structured-cloned across the boundary, so this must stay plain data — which is
        // why the worker rebuilds the solver model itself rather than being handed one
        // carrying closures.
        input: runInput(useModelStore.getState()),
      };
      worker.postMessage(request);
    }, DEBOUNCE_MS);

    return () => {
      if (pendingRef.current) clearTimeout(pendingRef.current);
    };
  }, [bodies, hinges, actuators, springDampers, contactSpheres, contactPlanes, contactHeightfields, settings]);

  return state;
}

// ---------------------------------------------------------------------------
// Reading frames
// ---------------------------------------------------------------------------

/** The simulated time of a frame. */
export const frameTime = (t: Trajectory, index: number): number =>
  t.data[index * t.meta.stride] ?? 0;

/** A view onto a frame's generalized coordinates. No copy. */
export function frameQ(t: Trajectory, index: number): Float64Array {
  const base = index * t.meta.stride + 1;
  return t.data.subarray(base, base + t.meta.nq);
}

/** A view onto a frame's generalized speeds. No copy. */
export function frameV(t: Trajectory, index: number): Float64Array {
  const base = index * t.meta.stride + 1 + t.meta.nq;
  return t.data.subarray(base, base + t.meta.nv);
}

export function frameEnergy(t: Trajectory, index: number): { kinetic: number; potential: number; total: number } {
  const base = index * t.meta.stride + 1 + t.meta.nq + t.meta.nv;
  const kinetic = t.data[base] ?? 0;
  const potential = t.data[base + 1] ?? 0;
  return { kinetic, potential, total: kinetic + potential };
}

/** The frame nearest a given time, clamped to what has actually been computed. */
export function frameAtTime(t: Trajectory, time: number): number {
  if (t.count === 0) return 0;
  const index = Math.round(time / t.meta.sampleInterval);
  return Math.min(t.count - 1, Math.max(0, index));
}

/** The last simulated time available, which grows as the run streams in. */
export const trajectoryDuration = (t: Trajectory): number =>
  t.count === 0 ? 0 : frameTime(t, t.count - 1);

/**
 * Total simulated time the run will eventually cover.
 *
 * Distinct from `trajectoryDuration`: the scrubber is sized from this so it does not
 * visibly grow while frames stream in, which would make the handle drift under the cursor.
 */
export const trajectorySpan = (t: Trajectory): number =>
  (t.meta.frameCount - 1) * t.meta.sampleInterval;

/** Memoized helper for components that only need the current frame index. */
export function useFrameIndex(trajectory: Trajectory | null, time: number): number {
  return useMemo(() => (trajectory ? frameAtTime(trajectory, time) : 0), [trajectory, time]);
}
