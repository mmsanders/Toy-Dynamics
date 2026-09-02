import { useMemo, useState } from 'react';
import { useModelStore } from '../store/useModelStore';
import {
  frameEnergy,
  frameQ,
  frameTime,
  frameV,
  trajectorySpan,
  type Trajectory,
} from '../sim/useSimulation';
import {
  buildSolverModel,
  bodyPoses,
  makeBodyMotionEvaluator,
  nodeWorldPosition,
  totalMomentum,
  type MotionFrame,
  type SolverModel,
} from '../sim/kinematics';
import { buildCsv, downloadCsv } from '../sim/csv';
import { unitLabel } from '../units';
import { toDisplayAngle } from '../math/conventions';
import { AXIS_COLORS, MAX_SERIES, SERIES_COLORS } from '../theme';
import { Legend, Plot, type Series } from './Plot';
import { CopyableRow } from './CopyableRow';
import { EmptyState, Note, Picker, Section } from './Bits';
import { Segmented } from './Segmented';

/**
 * The Run tab: play the trajectory back, read numbers off it, and export it.
 *
 * Everything here reads a trajectory that was computed in the background. Nothing in this
 * panel blocks on it, and nothing in it triggers a recompute — scrubbing, toggling a series
 * and hovering a chart are all free.
 */

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
 * Series grouped by unit.
 *
 * The grouping is the reason there is more than one chart: an angle and an offset do not
 * belong on the same axis, and putting them there would be a dual-axis chart wearing a
 * disguise.
 */
type Group = {
  id: string;
  title: string;
  unit: string;
  series: Series[];
};

export function RunPanel({
  trajectory,
  computing,
  error,
  time,
  onTime,
  playing,
  onPlaying,
  speed,
  onSpeed,
}: Props) {
  const bodies = useModelStore((s) => s.bodies);
  const hinges = useModelStore((s) => s.hinges);
  const actuators = useModelStore((s) => s.actuators);
  const springDampers = useModelStore((s) => s.springDampers);
  const contactSpheres = useModelStore((s) => s.contactSpheres);
  const contactPlanes = useModelStore((s) => s.contactPlanes);
  const settings = useModelStore((s) => s.settings);
  const conventions = useModelStore((s) => s.conventions);
  const selectedBodyId = useModelStore((s) => s.selectedBodyId);
  const selectBody = useModelStore((s) => s.selectBody);

  const [hidden, setHidden] = useState<Set<string>>(() => new Set());
  const [readoutNodeId, setReadoutNodeId] = useState<string | null>(null);
  const [motionFrame, setMotionFrame] = useState<MotionFrame>('world');

  const solver = useMemo(
    () => buildSolverModel(bodies, hinges, actuators, settings, springDampers, contactSpheres, contactPlanes),
    [bodies, hinges, actuators, settings, contactSpheres, contactPlanes, springDampers],
  );

  const frame = useMemo(() => {
    if (!trajectory || trajectory.count === 0) return 0;
    const index = Math.round(time / trajectory.meta.sampleInterval);
    return Math.min(trajectory.count - 1, Math.max(0, index));
  }, [trajectory, time]);

  const jointGroups = useMemo(
    () => (trajectory ? buildGroups(trajectory, settings, conventions) : []),
    [trajectory, settings, conventions],
  );
  const motionGroups = useMemo(() => {
    const body = bodies[selectedBodyId];
    if (!trajectory || !('model' in solver) || !body || body.isGround) return [];
    return buildBodyMotionGroups(trajectory, solver, body, motionFrame, settings);
  }, [trajectory, solver, bodies, selectedBodyId, motionFrame, settings]);
  const groups = [...motionGroups, ...jointGroups];
  const selectedBody = bodies[selectedBodyId];

  const visible = (group: Group): Series[] => group.series.filter((s) => !hidden.has(s.id));

  const toggleSeries = (id: string) =>
    setHidden((prev) => {
      const next = new Set(prev);
      if (next.has(id)) next.delete(id);
      else next.add(id);
      return next;
    });

  if (error) {
    return (
      <div className="stack">
        <Section title="Run">
          <Note tone="warn">{error}</Note>
        </Section>
      </div>
    );
  }

  const span = trajectory ? trajectorySpan(trajectory) : settings.duration;
  const diverged = trajectory?.progress.status === 'diverged';

  return (
    <div className="stack">
      <Section title="Playback">
        <Scrubber
          time={time}
          span={span}
          available={trajectory ? frameTime(trajectory, Math.max(0, trajectory.count - 1)) : 0}
          playing={playing}
          onPlaying={onPlaying}
          onTime={onTime}
          speed={speed}
          onSpeed={onSpeed}
        />

        <div className="status">
          {computing && <span className="status__chip">computing…</span>}
          {trajectory && (
            <span className="status__chip">
              {trajectory.count} / {trajectory.meta.frameCount} frames
            </span>
          )}
          {trajectory && <span className="status__chip">dt {trajectory.meta.dt.toPrecision(3)}</span>}
        </div>

        {diverged && (
          <Note tone="warn">
            The state stopped being finite at t ={' '}
            {trajectory?.progress.divergedAt?.toPrecision(4)}. Everything up to that point is still
            here to look at. Usually a timestep too large for a stiff spring or travel stop — the
            Setup tab has the step.
          </Note>
        )}

        {trajectory && trajectory.meta.passive && trajectory.progress.energyDrift > 1e-3 && (
          <Note tone="warn">
            Nothing in this model adds or removes energy, so total energy should be constant — but
            it has moved by {(trajectory.progress.energyDrift * 100).toPrecision(3)}%. That is
            integration error, not physics. Reduce the timestep or use RK4.
          </Note>
        )}

        {trajectory && trajectory.meta.passive && trajectory.progress.energyDrift <= 1e-3 && (
          <Note>
            Energy is conserved to {(trajectory.progress.energyDrift * 100).toExponential(1)}% over
            the run, so the timestep is resolving this model.
          </Note>
        )}
      </Section>

      {trajectory && trajectory.count > 0 && selectedBody && !selectedBody.isGround && (
        <Section title="Motion vectors">
          <Segmented
            label="Express components in"
            value={motionFrame}
            options={[
              { value: 'world', label: 'World', title: 'Fixed inertial world axes' },
              { value: 'body', label: 'Body', title: 'The selected body’s rotating axes' },
            ]}
            onChange={setMotionFrame}
          />
          <Note>
            This changes the linear and angular velocity and acceleration readout and plots for
            {` ${selectedBody.name}`}. Joint coordinates and energy keep their own natural bases.
          </Note>
        </Section>
      )}

      {trajectory && trajectory.count > 0 && 'model' in solver && (
        <Readout
          trajectory={trajectory}
          frame={frame}
          solver={solver}
          bodies={bodies}
          bodyId={selectedBodyId}
          onBody={selectBody}
          nodeId={readoutNodeId}
          onNode={setReadoutNodeId}
          units={settings.units}
          motionFrame={motionFrame}
        />
      )}

      {groups.length > 0 && (
        <Section title="Plots">
          <Note>
            Each chart holds one unit. Uncheck a series to hide it — the others keep their colours.
          </Note>
          {groups.map((group) => {
            const shown = visible(group);
            const overflow = shown.length > MAX_SERIES;
            return (
              <div key={group.id} className="plot-group">
                <div className="plot-group__toggles">
                  {group.series.map((s) => (
                    <label key={s.id} className="chip">
                      <input
                        type="checkbox"
                        checked={!hidden.has(s.id)}
                        onChange={() => toggleSeries(s.id)}
                      />
                      <span className="chip__swatch" style={{ background: s.color }} />
                      {s.label}
                    </label>
                  ))}
                </div>
                {overflow ? (
                  <Note tone="warn">
                    {shown.length} series selected. Past {MAX_SERIES} the colours stop being
                    reliably distinguishable — uncheck a few.
                  </Note>
                ) : (
                  <>
                    <Plot
                      title={group.title}
                      unit={group.unit}
                      series={shown}
                      count={trajectory?.count ?? 0}
                      timeAt={(i) => (trajectory ? frameTime(trajectory, i) : 0)}
                      playhead={frame}
                    />
                    <Legend series={shown} />
                  </>
                )}
              </div>
            );
          })}
        </Section>
      )}

      <Section title="Export">
        <button
          type="button"
          className="primary-button"
          disabled={!trajectory || trajectory.count === 0 || !('model' in solver)}
          onClick={() => {
            if (!trajectory || !('model' in solver)) return;
            downloadCsv(`toy-dynamics-${Date.now()}.csv`, buildCsv(trajectory, solver, bodies));
          }}
        >
          Download CSV
        </button>
        <Note>
          Everything: time, every generalized coordinate and speed, every body pose and velocity,
          every node position, energy and momentum. No configuration — the column you need is
          already in there.
        </Note>
      </Section>
    </div>
  );
}

/** Play/pause, scrub, and a speed control over the frames that exist so far. */
function Scrubber({
  time,
  span,
  available,
  playing,
  onPlaying,
  onTime,
  speed,
  onSpeed,
}: {
  time: number;
  span: number;
  available: number;
  playing: boolean;
  onPlaying: (playing: boolean) => void;
  onTime: (time: number) => void;
  speed: number;
  onSpeed: (speed: number) => void;
}) {
  return (
    <div className="scrubber">
      <div className="scrubber__row">
        <button
          type="button"
          className="scrubber__play"
          onClick={() => onPlaying(!playing)}
          aria-label={playing ? 'Pause' : 'Play'}
        >
          {playing ? '❙❙' : '▶'}
        </button>
        <input
          className="scrubber__track"
          type="range"
          min={0}
          // Sized from the whole run rather than what has streamed in, so the handle does
          // not drift under the cursor while frames arrive.
          max={span || 1}
          step={span / 500 || 0.01}
          value={Math.min(time, span)}
          onChange={(event) => {
            onPlaying(false);
            onTime(Number(event.target.value));
          }}
          aria-label="Time"
        />
        <span className="scrubber__time">{time.toFixed(2)}s</span>
      </div>
      <div className="scrubber__row">
        <button type="button" className="ghost-button" onClick={() => { onPlaying(false); onTime(0); }}>
          Reset
        </button>
        {[0.25, 0.5, 1, 2].map((option) => (
          <button
            key={option}
            type="button"
            className={`ghost-button${speed === option ? ' is-active' : ''}`}
            onClick={() => onSpeed(option)}
          >
            {option}×
          </button>
        ))}
      </div>
      {available < span && (
        <div className="scrubber__buffer" style={{ width: `${(available / (span || 1)) * 100}%` }} />
      )}
    </div>
  );
}

/**
 * The numbers at the current instant.
 *
 * Doubles as the table view the charts need for accessibility: every plotted quantity is
 * legible here as text at the scrubbed time, not only as a line on a chart.
 */
function Readout({
  trajectory,
  frame,
  solver,
  bodies,
  bodyId,
  onBody,
  nodeId,
  onNode,
  units,
  motionFrame,
}: {
  trajectory: Trajectory;
  frame: number;
  solver: Extract<ReturnType<typeof buildSolverModel>, { model: unknown }>;
  bodies: ReturnType<typeof useModelStore.getState>['bodies'];
  bodyId: string;
  onBody: (id: string) => void;
  nodeId: string | null;
  onNode: (id: string | null) => void;
  units: ReturnType<typeof useModelStore.getState>['settings']['units'];
  motionFrame: MotionFrame;
}) {
  const q = frameQ(trajectory, frame);
  const v = frameV(trajectory, frame);
  const energy = frameEnergy(trajectory, frame);

  const poses = useMemo(() => bodyPoses(solver, q), [solver, q]);
  const motion = useMemo(
    () => makeBodyMotionEvaluator(solver).at(q, v, frameTime(trajectory, frame), motionFrame),
    [solver, q, v, trajectory, frame, motionFrame],
  );
  const momentum = useMemo(() => totalMomentum(solver, q, v), [solver, q, v]);

  const body = bodies[bodyId];
  const movable = Object.values(bodies).filter((b) => !b.isGround);
  const pose = poses.get(bodyId);
  const bodyMotion = motion.get(bodyId);

  const node = body && nodeId ? body.nodes[nodeId] : undefined;
  const point = pose && node ? nodeWorldPosition(pose, node.position) : pose?.position;

  const fmt = (value: number): string => {
    if (!Number.isFinite(value)) return '—';
    const abs = Math.abs(value);
    if (abs !== 0 && (abs >= 1e5 || abs < 1e-4)) return value.toExponential(3);
    return (Math.round(value * 1e6) / 1e6).toString();
  };
  const frameName = motionFrame === 'world' ? 'world axes' : 'body axes';

  return (
    <Section title={`At t = ${frameTime(trajectory, frame).toFixed(3)}`}>
      {movable.length === 0 ? (
        <EmptyState>No bodies to report on.</EmptyState>
      ) : (
        <>
          <Picker
            label="Body"
            value={bodyId}
            options={movable.map((b) => ({ value: b.id, label: b.name }))}
            onChange={onBody}
          />
          {body && (
            <Picker
              label="Point"
              value={nodeId ?? body.originNodeId}
              options={body.nodeOrder.map((id) => ({ value: id, label: body.nodes[id]?.name ?? id }))}
              onChange={onNode}
            />
          )}

          {point && (
            <CopyableRow
              heading={`Position · ${unitLabel(units, 'length')}`}
              values={[
                { name: 'X', value: fmt(point[0]), color: AXIS_COLORS.X },
                { name: 'Y', value: fmt(point[1]), color: AXIS_COLORS.Y },
                { name: 'Z', value: fmt(point[2]), color: AXIS_COLORS.Z },
              ]}
            />
          )}

          {bodyMotion && (
            <>
              <CopyableRow
                heading={`Linear velocity · ${unitLabel(units, 'velocity')} · ${frameName}`}
                values={[
                  { name: 'X', value: fmt(bodyMotion.velocity.linear[0]), color: AXIS_COLORS.X },
                  { name: 'Y', value: fmt(bodyMotion.velocity.linear[1]), color: AXIS_COLORS.Y },
                  { name: 'Z', value: fmt(bodyMotion.velocity.linear[2]), color: AXIS_COLORS.Z },
                ]}
              />
              <CopyableRow
                heading={`Angular velocity · ${unitLabel(units, 'angularVelocity')} · ${frameName}`}
                values={[
                  { name: 'X', value: fmt(bodyMotion.velocity.angular[0]), color: AXIS_COLORS.X },
                  { name: 'Y', value: fmt(bodyMotion.velocity.angular[1]), color: AXIS_COLORS.Y },
                  { name: 'Z', value: fmt(bodyMotion.velocity.angular[2]), color: AXIS_COLORS.Z },
                ]}
              />
              <CopyableRow
                heading={`Linear acceleration · ${unitLabel(units, 'acceleration')} · ${frameName}`}
                values={[
                  { name: 'X', value: fmt(bodyMotion.acceleration.linear[0]), color: AXIS_COLORS.X },
                  { name: 'Y', value: fmt(bodyMotion.acceleration.linear[1]), color: AXIS_COLORS.Y },
                  { name: 'Z', value: fmt(bodyMotion.acceleration.linear[2]), color: AXIS_COLORS.Z },
                ]}
              />
              <CopyableRow
                heading={`Angular acceleration · ${unitLabel(units, 'angularAcceleration')} · ${frameName}`}
                values={[
                  { name: 'X', value: fmt(bodyMotion.acceleration.angular[0]), color: AXIS_COLORS.X },
                  { name: 'Y', value: fmt(bodyMotion.acceleration.angular[1]), color: AXIS_COLORS.Y },
                  { name: 'Z', value: fmt(bodyMotion.acceleration.angular[2]), color: AXIS_COLORS.Z },
                ]}
              />
            </>
          )}

          <CopyableRow
            heading={`Energy · ${unitLabel(units, 'energy')}`}
            values={[
              { name: 'Kinetic', value: fmt(energy.kinetic) },
              { name: 'Potential', value: fmt(energy.potential) },
              { name: 'Total', value: fmt(energy.total) },
            ]}
          />
          <CopyableRow
            heading="Total momentum"
            values={[
              { name: '|p|', value: fmt(Math.hypot(...momentum.linear)) },
              { name: '|L|', value: fmt(Math.hypot(...momentum.angular)) },
            ]}
          />
          <Note>
            With no external forces both momenta are conserved exactly, so watching them wander is
            the most direct check that the run is still trustworthy.
          </Note>
        </>
      )}
    </Section>
  );
}

/**
 * Build the plot groups from a trajectory.
 *
 * Colours are assigned by position in the *full* list of coordinates, not by position among
 * the visible ones, so hiding a series never repaints the rest.
 */
function buildGroups(
  trajectory: Trajectory,
  settings: ReturnType<typeof useModelStore.getState>['settings'],
  conventions: ReturnType<typeof useModelStore.getState>['conventions'],
): Group[] {
  const { meta } = trajectory;
  const angleUnit = conventions.angleUnit === 'deg' ? '°' : 'rad';

  const rotational: Series[] = [];
  const translational: Series[] = [];
  const angularRates: Series[] = [];
  const linearRates: Series[] = [];

  meta.dofNames.forEach((name, i) => {
    const color = SERIES_COLORS[i % SERIES_COLORS.length]!;
    const isRotational = /\.r[xyz]$/.test(name);
    // A rotational coordinate is stored in radians; everything the user reads is in their
    // chosen angle unit.
    const scale = isRotational ? (value: number) => toDisplayAngle(value, conventions.angleUnit) : (value: number) => value;

    // Only joints with a scalar coordinate appear here; a quaternion-parametrized rotation
    // has no single angle to plot, and inventing one would be a lie.
    const hasScalar = meta.nq === meta.nv;

    const position: Series = {
      id: `q:${name}`,
      label: name,
      color,
      at: (index) => (hasScalar ? scale(frameQ(trajectory, index)[i] ?? 0) : Number.NaN),
    };
    const rate: Series = {
      id: `u:${name}`,
      label: name,
      color,
      at: (index) => scale(frameV(trajectory, index)[i] ?? 0),
    };

    if (isRotational) {
      if (hasScalar) rotational.push(position);
      angularRates.push(rate);
    } else {
      if (hasScalar) translational.push(position);
      linearRates.push(rate);
    }
  });

  const energySeries: Series[] = [
    { id: 'e:kinetic', label: 'Kinetic', color: SERIES_COLORS[0]!, at: (i) => frameEnergy(trajectory, i).kinetic },
    { id: 'e:potential', label: 'Potential', color: SERIES_COLORS[1]!, at: (i) => frameEnergy(trajectory, i).potential },
    { id: 'e:total', label: 'Total', color: SERIES_COLORS[2]!, at: (i) => frameEnergy(trajectory, i).total },
  ];

  const groups: Group[] = [];
  if (rotational.length > 0) groups.push({ id: 'angles', title: 'Joint angles', unit: angleUnit, series: rotational });
  if (translational.length > 0)
    groups.push({ id: 'offsets', title: 'Joint offsets', unit: unitLabel(settings.units, 'length'), series: translational });
  if (angularRates.length > 0)
    groups.push({ id: 'angular-rates', title: 'Angular joint rates', unit: `${angleUnit}/s`, series: angularRates });
  if (linearRates.length > 0)
    groups.push({ id: 'linear-rates', title: 'Linear joint rates', unit: unitLabel(settings.units, 'velocity'), series: linearRates });
  groups.push({ id: 'energy', title: 'Energy', unit: unitLabel(settings.units, 'energy'), series: energySeries });

  return groups;
}

/**
 * Physical motion of the currently selected body. Generalized joint rates already have a
 * natural joint basis, so these are deliberately separate charts rather than pretending the
 * frame selector can re-express a coordinate or an energy scalar.
 */
function buildBodyMotionGroups(
  trajectory: Trajectory,
  solver: SolverModel,
  body: { id: string; name: string },
  frame: MotionFrame,
  settings: ReturnType<typeof useModelStore.getState>['settings'],
): Group[] {
  const size = trajectory.count * 3;
  const linearVelocity = new Float64Array(size).fill(Number.NaN);
  const angularVelocity = new Float64Array(size).fill(Number.NaN);
  const linearAcceleration = new Float64Array(size).fill(Number.NaN);
  const angularAcceleration = new Float64Array(size).fill(Number.NaN);
  const evaluator = makeBodyMotionEvaluator(solver);

  for (let i = 0; i < trajectory.count; i++) {
    const value = evaluator.at(frameQ(trajectory, i), frameV(trajectory, i), frameTime(trajectory, i), frame).get(body.id);
    if (!value) continue;
    for (let axis = 0; axis < 3; axis++) {
      const at = 3 * i + axis;
      linearVelocity[at] = value.velocity.linear[axis]!;
      angularVelocity[at] = value.velocity.angular[axis]!;
      linearAcceleration[at] = value.acceleration.linear[axis]!;
      angularAcceleration[at] = value.acceleration.angular[axis]!;
    }
  }

  const axes = [
    { label: 'X', color: AXIS_COLORS.X },
    { label: 'Y', color: AXIS_COLORS.Y },
    { label: 'Z', color: AXIS_COLORS.Z },
  ];
  const series = (kind: string, values: Float64Array): Series[] =>
    axes.map((axis, index) => ({
      id: `motion:${body.id}:${frame}:${kind}:${axis.label}`,
      label: axis.label,
      color: axis.color,
      at: (sample) => values[3 * sample + index] ?? Number.NaN,
    }));

  return [
    {
      id: `motion:${body.id}:${frame}:linear-velocity`,
      title: `Linear velocity · ${body.name}`,
      unit: unitLabel(settings.units, 'velocity'),
      series: series('linear-velocity', linearVelocity),
    },
    {
      id: `motion:${body.id}:${frame}:angular-velocity`,
      title: `Angular velocity · ${body.name}`,
      unit: unitLabel(settings.units, 'angularVelocity'),
      series: series('angular-velocity', angularVelocity),
    },
    {
      id: `motion:${body.id}:${frame}:linear-acceleration`,
      title: `Linear acceleration · ${body.name}`,
      unit: unitLabel(settings.units, 'acceleration'),
      series: series('linear-acceleration', linearAcceleration),
    },
    {
      id: `motion:${body.id}:${frame}:angular-acceleration`,
      title: `Angular acceleration · ${body.name}`,
      unit: unitLabel(settings.units, 'angularAcceleration'),
      series: series('angular-acceleration', angularAcceleration),
    },
  ];
}
