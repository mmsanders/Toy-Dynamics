import type { Integrator, UnitSystem, Vec3 } from '../types';
import { modelSnapshot, useModelStore } from '../store/useModelStore';
import { modelLink } from '../share/modelLink';
import { useCopy } from './useCopy';
import { EULER_ORDERS, describeSequence } from '../math/conventions';
import { UNIT_SYSTEMS, standardGravityVector, unitLabel } from '../units';
import { AXIS_COLORS } from '../theme';
import { NumberField } from './NumberField';
import { Segmented } from './Segmented';
import { Note, Section } from './Bits';

/**
 * Setup: unit system, gravity, the integrator, and the display conventions.
 *
 * Two guarantees are made visible here rather than only being true:
 *
 *  - Changing the **unit system** never rewrites a number. It sets labels, offers a gravity
 *    preset, and arms the plausibility checks. Nothing else moves.
 *  - Changing a **display convention** — up-axis, Euler order, degrees or radians — never
 *    changes the model either. Orientations are stored as quaternions and these are applied
 *    only when reading and writing numbers.
 */

const INTEGRATORS: { value: Integrator; label: string; title: string }[] = [
  { value: 'rk4', label: 'RK4', title: 'Four evaluations per step, fourth order. The accurate default.' },
  { value: 'rk2', label: 'RK2', title: 'Two evaluations, second order. A middle setting.' },
  { value: 'euler', label: 'Euler', title: 'One evaluation, first order, symplectic. Fastest and roughest.' },
];

export function SetupPanel() {
  const settings = useModelStore((s) => s.settings);
  const conventions = useModelStore((s) => s.conventions);
  const setSettings = useModelStore((s) => s.setSettings);
  const setUnits = useModelStore((s) => s.setUnits);
  const setConventions = useModelStore((s) => s.setConventions);
  const resetModel = useModelStore((s) => s.resetModel);
  const [status, copy] = useCopy();

  const system = UNIT_SYSTEMS[settings.units];
  const accelerationUnit = unitLabel(settings.units, 'acceleration');
  const gravityMagnitude = Math.hypot(...settings.gravity);

  const setGravityComponent = (index: number, value: number) => {
    const next: Vec3 = [...settings.gravity];
    next[index] = value;
    setSettings({ gravity: next });
  };

  return (
    <div className="stack">
      <Section title="Units">
        <Segmented
          label="System"
          value={settings.units}
          options={(Object.keys(UNIT_SYSTEMS) as UnitSystem[]).map((id) => ({
            value: id,
            label: UNIT_SYSTEMS[id].name,
            title: UNIT_SYSTEMS[id].note,
          }))}
          onChange={setUnits}
        />
        <Note>{system.note}</Note>
        <Note>
          Nothing is enforced or converted — the solver does arithmetic and keeping the numbers
          mutually consistent is yours to do. Switching systems relabels the fields and never
          rewrites a value.
        </Note>
      </Section>

      <Section title="Gravity">
        {(['X', 'Y', 'Z'] as const).map((axis, i) => (
          <NumberField
            key={axis}
            label={axis}
            value={settings.gravity[i]!}
            onChange={(value) => setGravityComponent(i, value)}
            min={-40}
            max={40}
            step={0.1}
            unit={accelerationUnit}
            color={AXIS_COLORS[axis]}
          />
        ))}
        <div className="inline-actions">
          {system.standardGravity !== null && (
            <button
              type="button"
              className="ghost-button"
              onClick={() => setSettings({ gravity: standardGravityVector(settings.units) })}
            >
              Standard ({system.standardGravity.toFixed(3)} {accelerationUnit})
            </button>
          )}
          <button type="button" className="ghost-button" onClick={() => setSettings({ gravity: [0, 0, 0] })}>
            Zero g
          </button>
          <span className="hint">|g| = {gravityMagnitude.toPrecision(5)}</span>
        </div>
      </Section>

      <Section title="Integration">
        <Segmented label="Method" value={settings.integrator} options={INTEGRATORS} onChange={(integrator) => setSettings({ integrator })} />
        <Note>
          None of these is implicit, so a very stiff spring or a hard travel stop still needs a
          small step. Euler is symplectic, so it stays bounded on oscillatory problems where it is
          merely inaccurate rather than divergent.
        </Note>
        <NumberField
          label="Timestep"
          value={settings.dt}
          onChange={(dt) => setSettings({ dt: Math.max(1e-6, dt) })}
          min={0.0001}
          max={0.05}
          step={0.0005}
          unit="s"
        />
        <NumberField
          label="Duration"
          value={settings.duration}
          onChange={(duration) => setSettings({ duration: Math.max(0.01, duration) })}
          min={0.5}
          max={60}
          step={0.5}
          unit="s"
        />
        <NumberField
          label="Sample rate"
          hint="frames stored"
          value={settings.sampleRate}
          onChange={(sampleRate) => setSettings({ sampleRate: Math.min(1000, Math.max(1, sampleRate)) })}
          min={10}
          max={240}
          step={10}
          unit="Hz"
        />
        <Note>
          The step is snapped so a whole number of them lands on each sample, keeping the stored
          times on an exact grid — which matters when you diff this against another tool.
        </Note>
      </Section>

      <Section title="Display conventions">
        <Note>
          These change how orientations are read and written, never what is stored. Switching one
          re-reads the same rotation; it cannot alter the model.
        </Note>
        <Segmented
          label="Up axis"
          value={conventions.upAxis}
          options={[
            { value: 'Z', label: 'Z up', title: 'Aerospace and robotics' },
            { value: 'Y', label: 'Y up', title: 'Graphics' },
          ]}
          onChange={(upAxis) => setConventions({ upAxis })}
        />
        <Segmented
          label="Angles"
          value={conventions.angleUnit}
          options={[
            { value: 'deg', label: 'Degrees' },
            { value: 'rad', label: 'Radians' },
          ]}
          onChange={(angleUnit) => setConventions({ angleUnit })}
        />
        <Segmented
          label="Euler order"
          wrap
          value={conventions.eulerOrder}
          options={EULER_ORDERS.map((order) => ({ value: order, label: order.split('').join('-') }))}
          onChange={(eulerOrder) => setConventions({ eulerOrder })}
        />
        <Segmented
          label="Applied"
          value={conventions.rotationMode}
          options={[
            { value: 'intrinsic', label: 'Intrinsic', title: 'About the new, already-rotated axes' },
            { value: 'extrinsic', label: 'Extrinsic', title: 'About the fixed world axes' },
          ]}
          onChange={(rotationMode) => setConventions({ rotationMode })}
        />
        <Note>Currently reading orientations as {describeSequence(conventions)}.</Note>
      </Section>

      <Section title="Share">
        <button
          type="button"
          className="primary-button"
          onClick={() => copy('link', modelLink(modelSnapshot(useModelStore.getState())))}
        >
          {status?.key === 'link' ? (status.ok ? 'Link copied' : 'No clipboard') : 'Copy a link to this model'}
        </button>
        <Note>
          The whole model travels in the link — bodies, hinges, actuators and settings — so a
          setup moves between a phone and a desktop. Opening one never destroys what you had:
          the replaced model sits behind an Undo.
        </Note>
      </Section>

      <Section title="Model">
        <button
          type="button"
          className="ghost-button is-danger"
          onClick={() => {
            if (confirm('Replace the current model with the starting example?')) resetModel();
          }}
        >
          Reset to the example model
        </button>
      </Section>
    </div>
  );
}
