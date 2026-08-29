import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react';

/** Fractions of viewport height the sheet settles at. */
const DETENTS = [0.16, 0.5, 0.92] as const;

/** Movement/duration under which a drag counts as a tap instead. */
const TAP_SLOP_PX = 6;
const TAP_MS = 400;

type Props = {
  children: ReactNode;
  /** Rendered below the drag strip, with normal event handling. */
  header?: ReactNode;
};

/**
 * A drag-to-resize bottom sheet.
 *
 * Hand-rolled rather than pulled from a library because the gesture handling has to be
 * exact: the drag must be owned by the grip strip alone, or it steals from the scrolling
 * content below and from the camera orbit behind.
 *
 * Two details matter more than they look:
 *
 *  - Pointer capture on the strip is what makes a drag survive the finger wandering off
 *    it. But capture also retargets the events that would have become a `click`, so
 *    nothing interactive may live inside the strip — the header sits outside it, and the
 *    grip's own tap-to-cycle is detected from the pointer events directly.
 *  - The strip carries `touch-action: none` so the browser never claims the gesture for
 *    scrolling first.
 */
export function BottomSheet({ children, header }: Props) {
  const [detent, setDetent] = useState(1);
  const [dragHeight, setDragHeight] = useState<number | null>(null);
  /**
   * Viewport height is state, not a `window.innerHeight` read during render.
   *
   * Reading it at render time silently breaks on rotate: the resize handler used to only
   * clear `dragHeight`, and when that was already null React's eager-state bailout skipped
   * the re-render entirely, leaving the sheet sized from the *previous* viewport. Turning a
   * phone to landscape could leave it covering the whole screen.
   */
  const [viewportHeight, setViewportHeight] = useState(() =>
    typeof window === 'undefined' ? 800 : window.innerHeight,
  );
  const dragRef = useRef<{ startY: number; startHeight: number; startedAt: number } | null>(null);
  const sheetRef = useRef<HTMLDivElement>(null);

  const cycle = useCallback(() => setDetent((d) => (d + 1) % DETENTS.length), []);

  const onPointerDown = useCallback((event: React.PointerEvent<HTMLDivElement>) => {
    event.currentTarget.setPointerCapture(event.pointerId);
    const current =
      sheetRef.current?.getBoundingClientRect().height ?? DETENTS[1] * viewportHeight;
    dragRef.current = {
      startY: event.clientY,
      startHeight: current,
      startedAt: performance.now(),
    };
    setDragHeight(current);
  }, [viewportHeight]);

  const onPointerMove = useCallback((event: React.PointerEvent<HTMLDivElement>) => {
    const drag = dragRef.current;
    if (!drag) return;
    // Dragging up grows the sheet, hence the inverted delta.
    const next = drag.startHeight + (drag.startY - event.clientY);
    const max = DETENTS[DETENTS.length - 1]! * viewportHeight;
    setDragHeight(Math.min(max, Math.max(64, next)));
  }, [viewportHeight]);

  const onPointerUp = useCallback(
    (event: React.PointerEvent<HTMLDivElement>) => {
      const drag = dragRef.current;
      dragRef.current = null;
      if (!drag) return;

      if (event.currentTarget.hasPointerCapture(event.pointerId)) {
        event.currentTarget.releasePointerCapture(event.pointerId);
      }

      const travelled = Math.abs(event.clientY - drag.startY);
      const elapsed = performance.now() - drag.startedAt;
      setDragHeight(null);

      if (travelled < TAP_SLOP_PX && elapsed < TAP_MS) {
        // A tap, not a drag: step to the next size.
        cycle();
        return;
      }

      // Snap to whichever detent the finger ended up nearest.
      const fraction = (drag.startHeight + (drag.startY - event.clientY)) / viewportHeight;
      let nearest = 0;
      for (let i = 1; i < DETENTS.length; i++) {
        if (Math.abs(DETENTS[i]! - fraction) < Math.abs(DETENTS[nearest]! - fraction)) {
          nearest = i;
        }
      }
      setDetent(nearest);
    },
    [cycle, viewportHeight],
  );

  // Re-snap on rotate/resize so the sheet keeps its proportion of the new viewport.
  useEffect(() => {
    const onResize = () => {
      setViewportHeight(window.innerHeight);
      setDragHeight(null);
    };
    window.addEventListener('resize', onResize);
    window.addEventListener('orientationchange', onResize);
    return () => {
      window.removeEventListener('resize', onResize);
      window.removeEventListener('orientationchange', onResize);
    };
  }, []);

  const height = dragHeight ?? (DETENTS[detent] ?? DETENTS[1]) * viewportHeight;

  return (
    <div
      ref={sheetRef}
      className="sheet"
      style={{
        height: `${height}px`,
        transition: dragHeight === null ? 'height 220ms ease' : 'none',
      }}
    >
      <div
        className="sheet__handle"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        onKeyDown={(event) => {
          if (event.key === 'Enter' || event.key === ' ') {
            event.preventDefault();
            cycle();
          }
        }}
        role="button"
        tabIndex={0}
        aria-label="Resize panel"
      >
        <span className="sheet__grip" />
      </div>

      {header && <div className="sheet__header">{header}</div>}
      <div className="sheet__body">{children}</div>
    </div>
  );
}
