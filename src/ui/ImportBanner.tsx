import { useEffect, useSyncExternalStore } from 'react';
import { useModelStore } from '../store/useModelStore';
import {
  clearDisplacedModel,
  displacedModel,
  subscribeToImports,
  watchHashImports,
} from '../share/importOnBoot';

/**
 * "A shared model replaced yours" — with an Undo.
 *
 * An import is destructive, and someone opening a link on a tab where they were mid-edit
 * should not lose that work to a tap. The displaced model is held in memory and this is the
 * way back to it.
 *
 * All the state lives in the import module rather than here, so a second link arriving after
 * this banner was dismissed brings it back — which local `dismissed` state would not.
 */
export function ImportBanner() {
  const loadModel = useModelStore((s) => s.loadModel);
  const previous = useSyncExternalStore(subscribeToImports, displacedModel, () => null);

  // A link pasted into an already-open tab changes only the hash, which is a same-document
  // navigation — no reload, so the boot-time import never runs. This catches those.
  useEffect(() => watchHashImports(), []);

  if (!previous) return null;

  return (
    <div className="toast" role="status">
      <span className="toast__text">Loaded a shared model.</span>
      <button
        type="button"
        className="toast__action"
        onClick={() => {
          loadModel(previous);
          clearDisplacedModel();
        }}
      >
        Undo
      </button>
      <button type="button" className="toast__close" aria-label="Dismiss" onClick={clearDisplacedModel}>
        ×
      </button>
    </div>
  );
}
