import { useCallback, useMemo, useState } from 'react';
import { SceneCanvas } from './scene/SceneCanvas';
import { Panel } from './ui/Panel';
import { WarningBanner } from './ui/WarningBanner';
import { DESKTOP_QUERY, useMediaQuery } from './ui/useMediaQuery';
import { useSimulation } from './sim/useSimulation';
import { usePlayback } from './sim/usePlayback';
import { useModelStore } from './store/useModelStore';

/**
 * Layout shell: the 3D view fills the viewport and the control panel sits over it.
 *
 * The canvas is never resized by the panel — on a phone the sheet slides over the scene
 * rather than squeezing it, so orbiting stays usable at any sheet height.
 *
 * Playback state lives here rather than in the Run tab, because the 3D view is driven by it
 * too. Switching tabs must not stop the animation or lose your place in the run.
 */
export function App() {
  const isDesktop = useMediaQuery(DESKTOP_QUERY);
  const simulation = useSimulation();
  const settings = useModelStore((s) => s.settings);

  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);

  const stop = useCallback(() => setPlaying(false), []);
  usePlayback(simulation.trajectory, playing, speed, setTime, stop);

  const frameIndex = useMemo(() => {
    const trajectory = simulation.trajectory;
    if (!trajectory || trajectory.count === 0) return 0;
    const index = Math.round(time / trajectory.meta.sampleInterval);
    return Math.min(trajectory.count - 1, Math.max(0, index));
  }, [simulation.trajectory, time]);

  return (
    <div className={`app${isDesktop ? ' app--desktop' : ''}`}>
      <div className="app__scene">
        <SceneCanvas trajectory={simulation.trajectory} frameIndex={frameIndex} />
      </div>

      {!isDesktop && (
        <header className="topbar">
          <h1 className="brand">
            Toy <span>Dynamics</span>
          </h1>
          <span className="topbar__status">
            t = {time.toFixed(2)}s · {settings.integrator.toUpperCase()}
            {simulation.computing ? ' · computing' : ''}
          </span>
        </header>
      )}

      <WarningBanner />

      <Panel
        trajectory={simulation.trajectory}
        computing={simulation.computing}
        error={simulation.error}
        time={time}
        onTime={setTime}
        playing={playing}
        onPlaying={setPlaying}
        speed={speed}
        onSpeed={setSpeed}
      />
    </div>
  );
}
