import { createRun, type Run, type RunInput, type RunProgress, type TrajectoryMeta } from '../sim/runner';

/**
 * The trajectory worker.
 *
 * Integration happens here so that editing a value never blocks the UI. Two details make
 * that actually true rather than nominally true:
 *
 *  - **The run is chunked.** A worker is single-threaded, so a straight loop would not
 *    process incoming messages until it finished — a cancel would arrive only after the
 *    work it was cancelling had completed. Instead each batch integrates for a few
 *    milliseconds, posts what it has, and yields through `setTimeout` so any queued message
 *    is handled before the next batch.
 *  - **Generations, not flags.** Every request carries a generation number, and a batch
 *    that finds a newer one abandons its run immediately. Results from a superseded run are
 *    never posted, so a fast typist cannot get an older trajectory landing on top of a
 *    newer one.
 *
 * Frames stream back as they are produced, so plots and the scrubber fill in progressively
 * rather than appearing all at once at the end.
 */

/** Per batch. Short enough that a cancel is acted on within a frame or two. */
const BATCH_MS = 8;

export type WorkerRequest =
  | { kind: 'run'; generation: number; input: RunInput }
  | { kind: 'cancel'; generation: number };

export type WorkerResponse =
  | { kind: 'started'; generation: number; meta: TrajectoryMeta }
  | { kind: 'chunk'; generation: number; from: number; to: number; data: Float64Array }
  | { kind: 'progress'; generation: number; progress: RunProgress }
  | { kind: 'failed'; generation: number; error: string };

let generation = -1;
let run: Run | null = null;
let sent = 0;
let scheduled = false;

/**
 * The worker global, typed.
 *
 * The project builds against the DOM lib rather than the WebWorker one — the app is
 * overwhelmingly DOM code — so `self` is typed as a Window here and its `postMessage`
 * signature is the wrong one. This narrows it to what a dedicated worker actually provides.
 */
type WorkerScope = {
  postMessage(message: WorkerResponse, transfer?: Transferable[]): void;
  onmessage: ((event: MessageEvent<WorkerRequest>) => void) | null;
};
const ctx = self as unknown as WorkerScope;

const post = (message: WorkerResponse, transfer?: Transferable[]): void => {
  if (transfer) ctx.postMessage(message, transfer);
  else ctx.postMessage(message);
};

/** Send whatever frames have been produced since the last chunk. */
function flush(current: number): void {
  if (!run || sent >= run.progress.written) return;
  const from = sent;
  const to = run.progress.written;
  const slice = run.data.slice(from * run.meta.stride, to * run.meta.stride);
  sent = to;
  // Transferred, not copied: these are the largest messages the app sends.
  post({ kind: 'chunk', generation: current, from, to, data: slice }, [slice.buffer]);
}

function pump(): void {
  scheduled = false;
  const current = generation;
  if (!run) return;

  const progress = run.advance(BATCH_MS);
  // A newer request arrived while we were integrating; drop this run on the floor.
  if (current !== generation) return;

  flush(current);
  post({ kind: 'progress', generation: current, progress: { ...progress } });

  if (progress.status === 'running') {
    scheduled = true;
    // setTimeout rather than a microtask: a microtask would run before the message queue is
    // drained, which is precisely the starvation this is avoiding.
    setTimeout(pump, 0);
  } else {
    run = null;
  }
}

ctx.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const message = event.data;

  if (message.kind === 'cancel') {
    if (message.generation >= generation) {
      generation = message.generation;
      run = null;
    }
    return;
  }

  if (message.kind !== 'run') return;
  // Out-of-order delivery would otherwise let a stale request restart a superseded run.
  if (message.generation < generation) return;

  generation = message.generation;
  sent = 0;
  const created = createRun(message.input);

  if ('error' in created) {
    run = null;
    post({ kind: 'failed', generation, error: created.error });
    return;
  }

  run = created;
  post({ kind: 'started', generation, meta: created.meta });
  flush(generation);

  if (!scheduled) {
    scheduled = true;
    setTimeout(pump, 0);
  }
};
