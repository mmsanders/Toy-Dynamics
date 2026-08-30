import { useEffect, useRef } from 'react';
import { trajectorySpan, type Trajectory } from './useSimulation';

/**
 * Advance the playhead in real time while playing.
 *
 * Driven by `requestAnimationFrame` and wall-clock deltas rather than by counting frames,
 * so playback runs at the speed it claims to whatever the display refresh rate is and
 * whatever the trajectory's sample rate is — the two are unrelated, and tying them together
 * would make a 120 Hz laptop play everything at double speed.
 *
 * Playback stops at the end of the *whole* run rather than at the last computed frame, so a
 * trajectory still streaming in does not repeatedly stop and restart as frames arrive.
 */
export function usePlayback(
  trajectory: Trajectory | null,
  playing: boolean,
  speed: number,
  onTime: (updater: (time: number) => number) => void,
  onStop: () => void,
): void {
  const lastRef = useRef<number | null>(null);

  useEffect(() => {
    if (!playing || !trajectory) {
      lastRef.current = null;
      return;
    }

    let raf = 0;
    const tick = (now: number) => {
      const previous = lastRef.current;
      lastRef.current = now;
      // The first tick after starting has no previous timestamp to measure against, so it
      // only establishes the baseline — otherwise resuming would jump by however long the
      // pause lasted.
      if (previous !== null) {
        const delta = ((now - previous) / 1000) * speed;
        onTime((time) => {
          const next = time + delta;
          const limit = trajectorySpan(trajectory);
          if (next >= limit) {
            onStop();
            return limit;
          }
          return next;
        });
      }
      raf = requestAnimationFrame(tick);
    };

    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing, trajectory, speed, onTime, onStop]);
}
