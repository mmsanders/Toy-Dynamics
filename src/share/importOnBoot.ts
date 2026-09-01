import { useModelStore } from '../store/useModelStore';
import type { ModelPersisted } from '../store/modelRepair';
import { MODEL_HASH_PREFIX, decodeModel } from './modelLink';

/**
 * Importing a shared model from the URL.
 *
 * Three things this gets right that a naive version does not:
 *
 *  - **The hash is cleared once consumed.** Otherwise a refresh re-imports over whatever you
 *    have edited since, silently discarding your work every time you reload.
 *  - **The replaced model is kept.** An import is destructive, so the previous model is held
 *    for an Undo rather than being gone. Anyone who opens a link on a tab where they were
 *    mid-edit gets their work back with one tap.
 *  - **A hash arriving later still imports.** Pasting a link into a tab that already has the
 *    app open is a *same-document* navigation: the page does not reload and module code
 *    never re-runs. Without the `hashchange` listener that link would appear to do nothing.
 */

let replaced: ModelPersisted | null = null;
const listeners = new Set<() => void>();

const notify = (): void => {
  for (const listener of listeners) listener();
};

/**
 * Subscribe to import events.
 *
 * Shaped for `useSyncExternalStore`: `displacedModel` is a stable snapshot between imports,
 * so a component can read it without tearing.
 */
export function subscribeToImports(listener: () => void): () => void {
  listeners.add(listener);
  return () => {
    listeners.delete(listener);
  };
}

/** The model an import displaced, or null if there is nothing to undo. */
export const displacedModel = (): ModelPersisted | null => replaced;

/** Drop the undo, which is also how the banner dismisses itself. */
export function clearDisplacedModel(): void {
  if (replaced === null) return;
  replaced = null;
  notify();
}

function snapshotCurrent(): ModelPersisted {
  const store = useModelStore.getState();
  return {
    bodies: store.bodies,
    bodyOrder: store.bodyOrder,
    hinges: store.hinges,
    hingeOrder: store.hingeOrder,
    actuators: store.actuators,
    actuatorOrder: store.actuatorOrder,
    contactSpheres: store.contactSpheres,
    contactSphereOrder: store.contactSphereOrder,
    contactPlanes: store.contactPlanes,
    contactPlaneOrder: store.contactPlaneOrder,
    settings: store.settings,
    conventions: store.conventions,
    selectedBodyId: store.selectedBodyId,
    selectedHingeId: store.selectedHingeId,
    selectedActuatorId: store.selectedActuatorId,
  };
}

/**
 * Read a model out of the URL hash and load it.
 *
 * Called before the first render, so a shared model is what actually paints rather than a
 * flash of the previous one — and again on every `hashchange`. Returns whether anything was
 * imported.
 */
export function importModelFromHash(): boolean {
  if (typeof window === 'undefined') return false;
  const hash = window.location.hash;
  if (!hash.startsWith(MODEL_HASH_PREFIX)) return false;

  const decoded = decodeModel(hash.slice(MODEL_HASH_PREFIX.length));

  // Clear the hash either way: a payload we could not read is not going to become readable
  // on the next reload, and leaving it there would re-fail forever.
  window.history.replaceState(null, '', window.location.pathname + window.location.search);
  if (!decoded) return false;

  replaced = snapshotCurrent();
  useModelStore.getState().loadModel(decoded);
  notify();
  return true;
}

/** Watch for links pasted into an already-open tab. Returns an unsubscribe. */
export function watchHashImports(): () => void {
  if (typeof window === 'undefined') return () => {};
  const onHashChange = () => void importModelFromHash();
  window.addEventListener('hashchange', onHashChange);
  return () => window.removeEventListener('hashchange', onHashChange);
}
