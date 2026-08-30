import { useMemo } from 'react';
import type { Profile } from '../types';
import { useModelStore } from '../store/useModelStore';
import { compileExpr, EXPR_FUNCTION_NAMES } from '../dyn/expr';
import { unitLabel } from '../units';
import { AXIS_COLORS } from '../theme';
import { NumberField } from './NumberField';
import { Segmented } from './Segmented';
import { EmptyState, IconButton, ListRow, Note, Picker, Section, TextField } from './Bits';

/**
 * Actuators: a force or a moment at a node.
 *
 * Two choices here matter more than they look, and both are surfaced rather than buried.
 *
 * **Force versus moment.** A force applied away from the centre of mass also spins the body;
 * a moment is a free vector and only spins it. They are different objects, not two settings
 * of one.
 *
 * **Body-fixed versus world-fixed.** A thruster bolted to a tumbling body sweeps its thrust
 * around with it; one pointing a fixed way in space does not. Identical numbers, completely
 * different motion. The 3D view shows the difference as the model moves.
 */

const PROFILE_KINDS: { value: Profile['kind']; label: string; title: string }[] = [
  { value: 'constant', label: 'Constant', title: 'On for the whole run' },
  { value: 'step', label: 'Step', title: 'On between two times' },
  { value: 'ramp', label: 'Ramp', title: 'Linear between two values' },
  { value: 'sine', label: 'Sine', title: 'Sinusoidal, with an offset' },
  { value: 'impulse', label: 'Impulse', title: 'A short burst of fixed total impulse' },
  { value: 'expr', label: 'f(t)', title: 'An expression in t' },
];

function defaultProfile(kind: Profile['kind']): Profile {
  switch (kind) {
    case 'constant':
      return { kind: 'constant' };
    case 'step':
      return { kind: 'step', tOn: 0, tOff: 1 };
    case 'ramp':
      return { kind: 'ramp', t0: 0, t1: 1, from: 0, to: 1 };
    case 'sine':
      return { kind: 'sine', frequency: 1, phase: 0, offset: 0 };
    case 'impulse':
      return { kind: 'impulse', t0: 0, width: 0.05 };
    case 'expr':
      return { kind: 'expr', source: 'pulse(t, 0, 1)' };
  }
}

export function ActuatorsPanel() {
  const bodies = useModelStore((s) => s.bodies);
  const actuators = useModelStore((s) => s.actuators);
  const actuatorOrder = useModelStore((s) => s.actuatorOrder);
  const settings = useModelStore((s) => s.settings);
  const selectedId = useModelStore((s) => s.selectedActuatorId);

  const selectActuator = useModelStore((s) => s.selectActuator);
  const addActuator = useModelStore((s) => s.addActuator);
  const removeActuator = useModelStore((s) => s.removeActuator);
  const renameActuator = useModelStore((s) => s.renameActuator);
  const setActuatorTarget = useModelStore((s) => s.setActuatorTarget);
  const setActuatorNode = useModelStore((s) => s.setActuatorNode);
  const setActuatorKind = useModelStore((s) => s.setActuatorKind);
  const setActuatorFrame = useModelStore((s) => s.setActuatorFrame);
  const setActuatorVector = useModelStore((s) => s.setActuatorVector);
  const setActuatorProfile = useModelStore((s) => s.setActuatorProfile);
  const toggleActuator = useModelStore((s) => s.toggleActuator);

  const actuator = selectedId ? actuators[selectedId] : undefined;
  const body = actuator ? bodies[actuator.bodyId] : undefined;
  const movable = Object.values(bodies).filter((b) => !b.isGround);
  const magnitudeUnit = actuator?.kind === 'moment' ? unitLabel(settings.units, 'moment') : unitLabel(settings.units, 'force');

  return (
    <div className="stack">
      <Section
        title="Actuators"
        action={
          <IconButton label="Add an actuator" onClick={() => addActuator()} disabled={movable.length === 0}>
            +
          </IconButton>
        }
      >
        {actuatorOrder.length === 0 && (
          <EmptyState>No actuators. The model will move only under gravity and its initial rates.</EmptyState>
        )}
        {actuatorOrder.map((id) => {
          const entry = actuators[id];
          if (!entry) return null;
          const magnitude = Math.hypot(...entry.vector);
          return (
            <ListRow
              key={id}
              label={entry.name}
              detail={`${entry.kind} · ${magnitude.toPrecision(3)} · ${bodies[entry.bodyId]?.name ?? '?'}`}
              color={entry.color}
              active={id === selectedId}
              onSelect={() => selectActuator(id)}
              actions={
                <>
                  <IconButton
                    label={entry.enabled ? 'Disable' : 'Enable'}
                    active={!entry.enabled}
                    onClick={() => toggleActuator(id)}
                  >
                    {entry.enabled ? '◉' : '○'}
                  </IconButton>
                  <IconButton label="Delete" danger onClick={() => removeActuator(id)}>
                    ×
                  </IconButton>
                </>
              }
            />
          );
        })}
      </Section>

      {!actuator && <EmptyState>Select an actuator to edit it.</EmptyState>}

      {actuator && body && (
        <>
          <Section title="What and where">
            <TextField
              label="Name"
              value={actuator.name}
              onChange={(name) => renameActuator(actuator.id, name)}
            />
            <Segmented
              label="Kind"
              value={actuator.kind}
              options={[
                { value: 'force', label: 'Force', title: 'A push, applied at the node' },
                { value: 'moment', label: 'Moment', title: 'A pure twist, applied to the body' },
              ]}
              onChange={(kind) => setActuatorKind(actuator.id, kind)}
            />
            <Picker
              label="On body"
              value={actuator.bodyId}
              options={movable.map((b) => ({ value: b.id, label: b.name }))}
              onChange={(id) => setActuatorTarget(actuator.id, id)}
            />
            <Picker
              label="At node"
              value={actuator.nodeId}
              options={body.nodeOrder.map((id) => ({ value: id, label: body.nodes[id]?.name ?? id }))}
              onChange={(id) => setActuatorNode(actuator.id, id)}
            />
            {actuator.kind === 'moment' && (
              <Note>
                A moment is a free vector, so the node sets only where the glyph is drawn and which
                axes a body-fixed direction is measured in — not where the twist acts.
              </Note>
            )}
          </Section>

          <Section title="Direction and magnitude">
            <Segmented
              label="Direction fixed to"
              value={actuator.frame}
              options={[
                { value: 'body', label: 'Body', title: "Turns with the body, like a bolted-on thruster" },
                { value: 'world', label: 'World', title: 'Stays pointing the same way in space' },
              ]}
              onChange={(frame) => setActuatorFrame(actuator.id, frame)}
            />
            <Note>
              {actuator.frame === 'body'
                ? 'Measured in the node’s axes, so it sweeps around as the body turns.'
                : 'Measured in world axes, so it keeps pointing the same way however the body turns.'}
            </Note>
            {(['X', 'Y', 'Z'] as const).map((axis, i) => (
              <NumberField
                key={axis}
                label={axis}
                value={actuator.vector[i]!}
                onChange={(value) => {
                  const next: [number, number, number] = [...actuator.vector];
                  next[i] = value;
                  setActuatorVector(actuator.id, next);
                }}
                min={-50}
                max={50}
                step={0.5}
                unit={magnitudeUnit}
                color={AXIS_COLORS[axis]}
              />
            ))}
          </Section>

          <Section title="Over time">
            <Segmented
              label="Profile"
              wrap
              value={actuator.profile.kind}
              options={PROFILE_KINDS}
              onChange={(kind) => setActuatorProfile(actuator.id, defaultProfile(kind))}
            />
            <ProfileEditor
              profile={actuator.profile}
              onChange={(profile) => setActuatorProfile(actuator.id, profile)}
            />
            <ProfilePreview profile={actuator.profile} duration={settings.duration} />
          </Section>
        </>
      )}
    </div>
  );
}

function ProfileEditor({ profile, onChange }: { profile: Profile; onChange: (p: Profile) => void }) {
  switch (profile.kind) {
    case 'constant':
      return <Note>Full magnitude for the whole run.</Note>;

    case 'step':
      return (
        <>
          <NumberField label="On at" value={profile.tOn} onChange={(tOn) => onChange({ ...profile, tOn })} min={0} max={20} step={0.1} unit="s" />
          <NumberField label="Off at" value={profile.tOff} onChange={(tOff) => onChange({ ...profile, tOff })} min={0} max={20} step={0.1} unit="s" />
        </>
      );

    case 'ramp':
      return (
        <>
          <NumberField label="From value" value={profile.from} onChange={(from) => onChange({ ...profile, from })} min={-2} max={2} step={0.05} />
          <NumberField label="To value" value={profile.to} onChange={(to) => onChange({ ...profile, to })} min={-2} max={2} step={0.05} />
          <NumberField label="Start at" value={profile.t0} onChange={(t0) => onChange({ ...profile, t0 })} min={0} max={20} step={0.1} unit="s" />
          <NumberField label="End at" value={profile.t1} onChange={(t1) => onChange({ ...profile, t1 })} min={0} max={20} step={0.1} unit="s" />
        </>
      );

    case 'sine':
      return (
        <>
          <NumberField label="Frequency" value={profile.frequency} onChange={(frequency) => onChange({ ...profile, frequency })} min={0} max={20} step={0.1} unit="Hz" />
          <NumberField label="Phase" value={profile.phase} onChange={(phase) => onChange({ ...profile, phase })} min={-Math.PI} max={Math.PI} step={0.05} unit="rad" />
          <NumberField label="Offset" value={profile.offset} onChange={(offset) => onChange({ ...profile, offset })} min={-2} max={2} step={0.05} />
        </>
      );

    case 'impulse':
      return (
        <>
          <NumberField label="At" value={profile.t0} onChange={(t0) => onChange({ ...profile, t0 })} min={0} max={20} step={0.1} unit="s" />
          <NumberField label="Width" value={profile.width} onChange={(width) => onChange({ ...profile, width })} min={0.001} max={1} step={0.005} unit="s" />
          <Note>
            Scaled by 1/width, so the total impulse delivered stays the same however narrow you
            make it. A narrow impulse needs a small timestep to land properly.
          </Note>
        </>
      );

    case 'expr':
      return <ExpressionEditor source={profile.source} onChange={(source) => onChange({ kind: 'expr', source })} />;
  }
}

/**
 * The general `f(t)` escape hatch.
 *
 * Validated as you type, with the error placed under a caret at the offending character.
 * The expression is parsed rather than evaluated as JavaScript, so only `t`, the constants
 * and the listed functions are in scope.
 */
function ExpressionEditor({ source, onChange }: { source: string; onChange: (s: string) => void }) {
  const compiled = useMemo(() => compileExpr(source), [source]);

  return (
    <div className="expr">
      <label className="text-field">
        <span className="text-field__label">f(t)</span>
        <input
          className={`text-field__input expr__input${compiled.ok ? '' : ' is-invalid'}`}
          type="text"
          spellCheck={false}
          autoCapitalize="off"
          autoCorrect="off"
          value={source}
          onChange={(event) => onChange(event.target.value)}
        />
      </label>

      {!compiled.ok && (
        <div className="expr__error">
          <pre className="expr__caret">{`${' '.repeat(Math.max(0, compiled.at))}^`}</pre>
          <span>{compiled.error}</span>
        </div>
      )}

      <details className="expr__help">
        <summary>What you can write</summary>
        <p>
          The variable <code>t</code>, the constants <code>pi</code>, <code>e</code>,{' '}
          <code>tau</code>, and <code>+ - * / ^ %</code> with parentheses.
        </p>
        <p className="expr__functions">{EXPR_FUNCTION_NAMES.join(' · ')}</p>
        <p>
          <code>step(t - 2)</code> switches on at 2. <code>pulse(t, 1, 3)</code> is a burn from 1 to
          3. Multiply them together to build a schedule.
        </p>
      </details>
    </div>
  );
}

/**
 * A sparkline of the profile over the run.
 *
 * Small, but it turns "did I get the times right" from a question you answer by running the
 * simulation into one you answer by looking.
 */
function ProfilePreview({ profile, duration }: { profile: Profile; duration: number }) {
  const path = useMemo(() => {
    const compiled =
      profile.kind === 'expr' ? compileExpr(profile.source) : null;
    if (compiled && !compiled.ok) return null;

    const evaluate = (t: number): number => {
      switch (profile.kind) {
        case 'constant':
          return 1;
        case 'step':
          return t >= profile.tOn && t < profile.tOff ? 1 : 0;
        case 'ramp': {
          const span = profile.t1 - profile.t0;
          if (t <= profile.t0) return profile.from;
          if (t >= profile.t1 || span === 0) return profile.to;
          return profile.from + ((t - profile.t0) / span) * (profile.to - profile.from);
        }
        case 'sine':
          return profile.offset + Math.sin(2 * Math.PI * profile.frequency * t + profile.phase);
        case 'impulse':
          return t >= profile.t0 && t < profile.t0 + Math.max(profile.width, 1e-9) ? 1 / Math.max(profile.width, 1e-9) : 0;
        case 'expr':
          return compiled && compiled.ok ? compiled.fn(t) : 0;
      }
    };

    const samples = 160;
    const values: number[] = [];
    for (let i = 0; i <= samples; i++) {
      const value = evaluate((i / samples) * duration);
      values.push(Number.isFinite(value) ? value : 0);
    }
    let lo = Math.min(0, ...values);
    let hi = Math.max(0, ...values);
    if (hi - lo < 1e-9) {
      lo -= 0.5;
      hi += 0.5;
    }
    const pad = (hi - lo) * 0.08;
    lo -= pad;
    hi += pad;

    const points = values.map((value, i) => {
      const x = (i / samples) * 100;
      const y = 30 - ((value - lo) / (hi - lo)) * 30;
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    });
    const zeroY = 30 - ((0 - lo) / (hi - lo)) * 30;
    return { line: `M${points.join(' L')}`, zeroY, lo, hi };
  }, [profile, duration]);

  if (!path) return null;

  return (
    <figure className="sparkline">
      <svg viewBox="0 0 100 30" preserveAspectRatio="none" role="img" aria-label="Actuator profile over time">
        <line x1="0" y1={path.zeroY} x2="100" y2={path.zeroY} className="sparkline__zero" />
        <path d={path.line} className="sparkline__line" vectorEffect="non-scaling-stroke" />
      </svg>
      <figcaption>
        0 → {duration}s · multiplier {path.lo.toPrecision(2)} to {path.hi.toPrecision(2)}
      </figcaption>
    </figure>
  );
}
