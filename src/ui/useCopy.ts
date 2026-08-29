import { useCallback, useState } from 'react';

/**
 * Tap-to-copy with brief per-value feedback.
 *
 * Returns the key of the most recently copied value so a caller can swap that one label
 * for a confirmation without tracking it itself.
 */
export type CopyStatus = { key: string; ok: boolean } | null;

export function useCopy(): [CopyStatus, (key: string, text: string) => void] {
  const [status, setStatus] = useState<CopyStatus>(null);

  const copy = useCallback((key: string, text: string) => {
    const settle = (ok: boolean) => {
      setStatus({ key, ok });
      window.setTimeout(() => setStatus((s) => (s?.key === key ? null : s)), 1300);
    };

    /**
     * The Clipboard API needs a secure context, so it is simply absent over plain http —
     * which is exactly how the dev server is reached from a phone on the LAN. Optional
     * chaining alone would short-circuit the whole chain and make copying a silent no-op:
     * no copy, no confirmation, no error. Report the failure instead.
     */
    const clipboard = navigator.clipboard;
    if (!clipboard) {
      settle(false);
      return;
    }
    clipboard.writeText(text).then(
      () => settle(true),
      () => settle(false),
    );
  }, []);

  return [status, copy];
}
