import { useState } from 'react';
import { BottomSheet } from './BottomSheet';
import { BodiesPanel } from './BodiesPanel';
import { HingesPanel } from './HingesPanel';
import { ActuatorsPanel } from './ActuatorsPanel';
import { RunPanel } from './RunPanel';
import { SetupPanel } from './SetupPanel';
import { DESKTOP_QUERY, useMediaQuery } from './useMediaQuery';
import type { Trajectory } from '../sim/useSimulation';

const TABS = [
  { id: 'bodies', label: 'Bodies' },
  { id: 'hinges', label: 'Hinges' },
  { id: 'actuators', label: 'Actuators' },
  { id: 'run', label: 'Run' },
  { id: 'setup', label: 'Setup' },
] as const;

type TabId = (typeof TABS)[number]['id'];

type Props = {
  trajectory: Trajectory | null;
  computing: boolean;
  error: string | null;
  time: number;
  onTime: (time: number) => void;
  playing: boolean;
  onPlaying: (playing: boolean) => void;
  speed: number;
  onSpeed: (speed: number) => void;
};

/**
 * The control surface: a drag-up sheet on phones, a docked sidebar on desktop.
 *
 * Both layouts render exactly the same tab contents — only the container differs — so there
 * is one implementation of every control rather than a mobile and a desktop copy that drift
 * apart.
 */
export function Panel(props: Props) {
  const isDesktop = useMediaQuery(DESKTOP_QUERY);
  const [tab, setTab] = useState<TabId>('bodies');

  const tabBar = (
    <nav className="tabs" role="tablist" aria-label="Panel sections">
      {TABS.map((entry) => (
        <button
          key={entry.id}
          type="button"
          role="tab"
          aria-selected={tab === entry.id}
          className={`tabs__tab${tab === entry.id ? ' is-active' : ''}`}
          onClick={() => setTab(entry.id)}
        >
          {entry.label}
        </button>
      ))}
    </nav>
  );

  const content = (
    <div className="panel__content" role="tabpanel">
      {tab === 'bodies' && <BodiesPanel />}
      {tab === 'hinges' && <HingesPanel />}
      {tab === 'actuators' && <ActuatorsPanel />}
      {tab === 'run' && <RunPanel {...props} />}
      {tab === 'setup' && <SetupPanel />}
    </div>
  );

  if (isDesktop) {
    return (
      <aside className="sidebar">
        <header className="sidebar__head">
          <h1 className="brand">
            Toy <span>Dynamics</span>
          </h1>
        </header>
        {tabBar}
        {content}
      </aside>
    );
  }

  return <BottomSheet header={tabBar}>{content}</BottomSheet>;
}
