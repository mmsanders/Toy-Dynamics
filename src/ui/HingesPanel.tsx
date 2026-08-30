import { DOF_LABELS, GROUND_ID, type Conventions, type DofSpec, type Hinge } from '../types';
import { useModelStore } from '../store/useModelStore';
import { wouldCreateCycle } from '../model/topology';
import { fromDisplayAngle, toDisplayAngle } from '../math/conventions';
import { unitLabel, type Quantity } from '../units';
import { AXIS_COLORS, HINGE_COLOR } from '../theme';
import { NumberField } from './NumberField';
import { RotationEditor } from './RotationEditor';
import { EmptyState, ListRow, Note, Picker, Section, TextField, Toggle } from './Bits';

/**
 * Hinges: which of the six axes between two bodies are free.
 *
 * This panel is the heart of the model. Everything else describes what the bodies *are*;
 * this describes what they can do relative to each other, and a hinge here is one general
 * six-axis joint rather than a menu of named types. A revolute is one free rotation, a
 * telescoping pole is one free translation, a ball joint is three, a free-flying body is all
 * six, and a weld is none — all the same control.
 *
 * Angles are stored in radians and shown in whatever unit the Setup tab is set to, so a
 * hinge angle reads as the number you would write down.
 */

const AXIS_OF_DOF = ['X', 'Y', 'Z', 'X', 'Y', 'Z'] as const;

const DOF_DESCRIPTION = [
  'Slide along the joint X axis',
  'Slide along the joint Y axis',
  'Slide along the joint Z axis',
  'Rotate about the joint X axis',
  'Rotate about the joint Y axis',
  'Rotate about the joint Z axis',
];

const isRotational = (axis: number): boolean => axis >= 3;

export function HingesPanel() {
  const bodies = useModelStore((s) => s.bodies);
  const hinges = useModelStore((s) => s.hinges);
  const hingeOrder = useModelStore((s) => s.hingeOrder);
  const conventions = useModelStore((s) => s.conventions);
  const settings = useModelStore((s) => s.settings);
  const selectedId = useModelStore((s) => s.selectedHingeId);

  const selectHinge = useModelStore((s) => s.selectHinge);
  const renameHinge = useModelStore((s) => s.renameHinge);
  const setHingeParent = useModelStore((s) => s.setHingeParent);
  const setHingeParentNode = useModelStore((s) => s.setHingeParentNode);
  const setHingeChildNode = useModelStore((s) => s.setHingeChildNode);
  const setHingeMount = useModelStore((s) => s.setHingeMount);
  const resetHingeDof = useModelStore((s) => s.resetHingeDof);

  const hinge = selectedId ? hinges[selectedId] : undefined;
  const parent = hinge ? bodies[hinge.parentBodyId] : undefined;
  const child = hinge ? bodies[hinge.childBodyId] : undefined;

  const freeCount = (h: Hinge): number => h.dof.filter((d) => d.free).length;

  return (
    <div className="stack">
      <Section title="Hinges">
        {hingeOrder.length === 0 && <EmptyState>Add a body and it will bring a hinge with it.</EmptyState>}
        {hingeOrder.map((id) => {
          const entry = hinges[id];
          if (!entry) return null;
          const count = freeCount(entry);
          return (
            <ListRow
              key={id}
              label={entry.name}
              detail={`${bodies[entry.parentBodyId]?.name ?? '?'} → ${bodies[entry.childBodyId]?.name ?? '?'} · ${
                count === 0 ? 'welded' : `${count} DOF`
              }`}
              color={HINGE_COLOR}
              active={id === selectedId}
              onSelect={() => selectHinge(id)}
            />
          );
        })}
      </Section>

      {!hinge && <EmptyState>Select a hinge to edit it.</EmptyState>}

      {hinge && parent && child && (
        <>
          <Section title="Connection">
            <TextField label="Name" value={hinge.name} onChange={(name) => renameHinge(hinge.id, name)} />
            <Picker
              label="Parent body"
              value={hinge.parentBodyId}
              options={Object.values(bodies).map((b) => ({
                value: b.id,
                label: b.name,
                // Anything that would close a loop is disabled rather than allowed and then
                // rejected — reduced coordinates describe trees only.
                disabled: b.id === hinge.childBodyId || wouldCreateCycle(hinges, hinge.childBodyId, b.id),
              }))}
              onChange={(id) => setHingeParent(hinge.id, id)}
            />
            <Picker
              label="Attach at"
              value={hinge.parentNodeId}
              options={parent.nodeOrder.map((id) => ({
                value: id,
                label: parent.nodes[id]?.name ?? id,
              }))}
              onChange={(id) => setHingeParentNode(hinge.id, id)}
            />
            <Picker
              label={`${child.name} attaches by`}
              value={hinge.childNodeId}
              options={child.nodeOrder.map((id) => ({
                value: id,
                label: child.nodes[id]?.name ?? id,
              }))}
              onChange={(id) => setHingeChildNode(hinge.id, id)}
            />
            {hinge.parentBodyId === GROUND_ID && (
              <Note>Attached to ground, so the free axes below are this body's motion in the world.</Note>
            )}
          </Section>

          <Section title="Joint axes">
            <RotationEditor
              label="Axis orientation"
              value={hinge.mount}
              onChange={(mount) => setHingeMount(hinge.id, mount)}
              conventions={conventions}
            />
            <Note>
              Relative to the attachment node's axes. The joint translates along these axes first,
              then rotates about them — so a slide axis stays fixed in the parent while the child
              turns on the end of it.
            </Note>
          </Section>

          <Section
            title="Degrees of freedom"
            action={
              <button type="button" className="ghost-button" onClick={() => resetHingeDof(hinge.id)}>
                Lock all
              </button>
            }
          >
            <Note>
              A locked axis is not a constraint the solver has to satisfy — it simply is not a
              coordinate. Locks are exact and cost nothing.
            </Note>
            {[0, 1, 2, 3, 4, 5].map((axis) => (
              <DofRow
                key={axis}
                hingeId={hinge.id}
                axis={axis}
                dof={hinge.dof[axis]!}
                conventions={conventions}
                units={settings.units}
                allRotationsFree={hinge.dof[3]!.free && hinge.dof[4]!.free && hinge.dof[5]!.free}
              />
            ))}
          </Section>
        </>
      )}
    </div>
  );
}

/**
 * One axis: free or locked, its initial state, and what resists it.
 *
 * Collapsed by default unless the axis is free, because a locked axis has exactly one
 * number worth seeing — the offset it is held at — and six expanded panels of springs and
 * stops would bury the thing this tab is actually for.
 */
function DofRow({
  hingeId,
  axis,
  dof,
  conventions,
  units,
  allRotationsFree,
}: {
  hingeId: string;
  axis: number;
  dof: DofSpec;
  conventions: Conventions;
  units: ReturnType<typeof useModelStore.getState>['settings']['units'];
  allRotationsFree: boolean;
}) {
  const setDof = useModelStore((s) => s.setDof);
  const setDofLimit = useModelStore((s) => s.setDofLimit);

  const rotational = isRotational(axis);
  const angleUnit = conventions.angleUnit === 'deg' ? '°' : 'rad';
  const unitFor = (quantity: Quantity): string => unitLabel(units, quantity);

  // Rotational values live in radians and are shown in the chosen angle unit; translational
  // ones are unit-free magnitudes and pass straight through.
  const toDisplay = (value: number): number =>
    rotational ? toDisplayAngle(value, conventions.angleUnit) : value;
  const fromDisplay = (value: number): number =>
    rotational ? fromDisplayAngle(value, conventions.angleUnit) : value;

  const valueUnit = rotational ? angleUnit : unitFor('length');
  const rateUnit = rotational ? `${angleUnit}/s` : unitFor('velocity');
  const stiffnessUnit = rotational ? unitFor('angularStiffness') : unitFor('linearStiffness');
  const dampingUnit = rotational ? unitFor('angularDamping') : unitFor('linearDamping');
  const effortUnit = rotational ? unitFor('moment') : unitFor('force');

  const range = rotational
    ? conventions.angleUnit === 'deg'
      ? { min: -180, max: 180, step: 1 }
      : { min: -Math.PI, max: Math.PI, step: 0.01 }
    : { min: -3, max: 3, step: 0.05 };

  // A ball or free joint stores its rotation as a quaternion, which has no single angle for
  // a spring or a stop to work against. Said here, at the control, rather than only in a
  // warning after the fact.
  const quaternionRotation = rotational && allRotationsFree;

  return (
    <details className={`dof${dof.free ? ' is-free' : ''}`} open={dof.free}>
      <summary className="dof__summary">
        <Toggle
          label={DOF_LABELS[axis]!}
          checked={dof.free}
          onChange={(free) => setDof(hingeId, axis, { free })}
          hint={dof.free ? 'free' : 'locked'}
        />
        <span className="dof__axis" style={{ color: AXIS_COLORS[AXIS_OF_DOF[axis]!] }}>
          {DOF_DESCRIPTION[axis]}
        </span>
      </summary>

      <div className="dof__body">
        <NumberField
          label={dof.free ? 'Initial value' : 'Held at'}
          value={toDisplay(dof.q0)}
          onChange={(value) => setDof(hingeId, axis, { q0: fromDisplay(value) })}
          min={range.min}
          max={range.max}
          step={range.step}
          unit={valueUnit}
          color={AXIS_COLORS[AXIS_OF_DOF[axis]!]}
        />

        {dof.free && (
          <>
            <NumberField
              label="Initial rate"
              value={toDisplay(dof.u0)}
              onChange={(value) => setDof(hingeId, axis, { u0: fromDisplay(value) })}
              min={rotational ? (conventions.angleUnit === 'deg' ? -720 : -12) : -10}
              max={rotational ? (conventions.angleUnit === 'deg' ? 720 : 12) : 10}
              step={rotational ? (conventions.angleUnit === 'deg' ? 5 : 0.1) : 0.1}
              unit={rateUnit}
            />

            {!quaternionRotation && (
              <>
                <NumberField
                  label="Spring"
                  hint="stiffness"
                  value={dof.stiffness}
                  onChange={(stiffness) => setDof(hingeId, axis, { stiffness })}
                  min={0}
                  max={200}
                  step={1}
                  unit={stiffnessUnit}
                />
                {dof.stiffness !== 0 && (
                  <NumberField
                    label="Rest"
                    value={toDisplay(dof.rest)}
                    onChange={(value) => setDof(hingeId, axis, { rest: fromDisplay(value) })}
                    min={range.min}
                    max={range.max}
                    step={range.step}
                    unit={valueUnit}
                  />
                )}
              </>
            )}

            <NumberField
              label="Damping"
              value={dof.damping}
              onChange={(damping) => setDof(hingeId, axis, { damping })}
              min={0}
              max={20}
              step={0.05}
              unit={dampingUnit}
            />
            <NumberField
              label="Friction"
              hint="constant"
              value={dof.friction}
              onChange={(friction) => setDof(hingeId, axis, { friction })}
              min={0}
              max={20}
              step={0.05}
              unit={effortUnit}
            />
            {dof.friction !== 0 && (
              <Note>
                Regularized over a small velocity so it does not chatter, which means it models
                sliding friction rather than stiction — a joint under a light load will creep
                instead of holding.
              </Note>
            )}

            {!quaternionRotation && (
              <>
                <Toggle
                  label="Travel limits"
                  checked={dof.limit.enabled}
                  onChange={(enabled) => setDofLimit(hingeId, axis, { enabled })}
                />
                {dof.limit.enabled && (
                  <>
                    <NumberField
                      label="Lower stop"
                      value={toDisplay(dof.limit.lo)}
                      onChange={(value) => setDofLimit(hingeId, axis, { lo: fromDisplay(value) })}
                      min={range.min}
                      max={range.max}
                      step={range.step}
                      unit={valueUnit}
                    />
                    <NumberField
                      label="Upper stop"
                      value={toDisplay(dof.limit.hi)}
                      onChange={(value) => setDofLimit(hingeId, axis, { hi: fromDisplay(value) })}
                      min={range.min}
                      max={range.max}
                      step={range.step}
                      unit={valueUnit}
                    />
                    <NumberField
                      label="Stop stiffness"
                      value={dof.limit.stiffness}
                      onChange={(stiffness) => setDofLimit(hingeId, axis, { stiffness })}
                      min={0}
                      max={100000}
                      step={100}
                      unit={stiffnessUnit}
                    />
                    <Note>
                      Stops are penalty springs, not hard impacts, so expect a little overshoot.
                      Stiffer stops need a smaller timestep — the Run tab will say so if yours is
                      too large.
                    </Note>
                  </>
                )}
              </>
            )}

            {quaternionRotation && (
              <Note tone="warn">
                All three rotations are free, so this hinge stores its orientation as a quaternion
                and there is no single angle for a spring or a stop to act on. Damping and friction
                still work. Lock one rotation to get the angles back.
              </Note>
            )}
          </>
        )}
      </div>
    </details>
  );
}
