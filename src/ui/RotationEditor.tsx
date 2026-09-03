import { useState } from 'react';
import type { Conventions, Quat, Vec3 } from '../types';
import {
  angleRange,
  describeSequence,
  eulerFromQuat,
  eulerSequence,
  isNearGimbalLock,
  quatFromEuler,
} from '../math/conventions';
import { AXIS_COLORS } from '../theme';
import { NumberField } from './NumberField';
import { Segmented } from './Segmented';
import { CopyableRow } from './CopyableRow';

/**
 * Editing an orientation as either Euler angles or a quaternion, with each shown live in
 * terms of the other.
 *
 * Euler edits update the stored orientation when each field is committed. Quaternion edits
 * deliberately use a four-component draft instead: normalizing after every component would
 * rewrite the remaining fields while the user was still entering them.
 *
 * Two things this makes visible that a single-representation editor hides:
 *
 *  - **Gimbal lock.** Where the Euler triple stops being unique, the panel says so. The
 *    angles are still editable; they simply are no longer the trustworthy reading, and the
 *    quaternion is.
 *  - **Normalization.** A quaternion draft is normalized only when Apply is pressed, so a
 *    hand-entered `[1, 1, 0, 0]` becomes a unit quaternion without fighting data entry.
 */

type Props = {
  value: Quat;
  onChange: (value: Quat) => void;
  conventions: Conventions;
  label?: string;
};

const fmt = (value: number, digits = 5): string => {
  const fixed = value.toFixed(digits);
  return fixed === `-${(0).toFixed(digits)}` ? (0).toFixed(digits) : fixed;
};

function normalize(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-9) return [0, 0, 0, 1];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function RotationEditor({ value, onChange, conventions, label = 'Orientation' }: Props) {
  const [mode, setMode] = useState<'euler' | 'quaternion'>('euler');
  // `null` follows the stored value. A tuple means the user has started a transaction and
  // must remain independent of store updates until they apply it.
  const [quaternionDraft, setQuaternionDraft] = useState<Quat | null>(null);

  const euler = eulerFromQuat(value, conventions);
  const slots = eulerSequence(conventions);
  const range = angleRange(conventions.angleUnit);
  const unit = conventions.angleUnit === 'deg' ? '°' : 'rad';
  const locked = isNearGimbalLock(value, conventions);

  const setEulerComponent = (index: 0 | 1 | 2, next: number) => {
    const angles: Vec3 = [...euler];
    angles[index] = next;
    onChange(quatFromEuler(angles, conventions));
  };

  const setQuatComponent = (index: 0 | 1 | 2 | 3, next: number) => {
    const q: Quat = [...(quaternionDraft ?? value)];
    q[index] = next;
    setQuaternionDraft(q);
  };

  const applyQuaternion = () => {
    if (!quaternionDraft) return;
    const next = normalize(quaternionDraft);
    setQuaternionDraft(null);
    onChange(next);
  };

  return (
    <div className="rotation">
      <div className="rotation__head">
        <span className="rotation__label">{label}</span>
        <Segmented
          label="Edit as"
          value={mode}
          options={[
            { value: 'euler', label: 'Euler' },
            { value: 'quaternion', label: 'Quat' },
          ]}
          onChange={setMode}
        />
      </div>

      {mode === 'euler' ? (
        <>
          {slots.map((slot) => (
            <NumberField
              key={slot.axis}
              label={slot.alias ? `${slot.alias} (${slot.axis})` : slot.axis}
              hint={`${slot.step} of 3`}
              value={euler[slot.index]}
              onChange={(next) => setEulerComponent(slot.index, next)}
              min={range.min}
              max={range.max}
              step={conventions.angleUnit === 'deg' ? 1 : 0.01}
              unit={unit}
              color={AXIS_COLORS[slot.axis]}
            />
          ))}
          <CopyableRow
            heading="As a quaternion"
            values={[
              { name: 'w', value: fmt(value[3]) },
              { name: 'x', value: fmt(value[0]) },
              { name: 'y', value: fmt(value[1]) },
              { name: 'z', value: fmt(value[2]) },
            ]}
          />
        </>
      ) : (
        <>
          {/* w first: it is the one people read for "how much rotation", and putting the
              vector part after it matches how a quaternion is spoken aloud. */}
          {([3, 0, 1, 2] as const).map((index) => (
            <NumberField
              key={index}
              label={(['x', 'y', 'z', 'w'] as const)[index]}
              value={(quaternionDraft ?? value)[index]}
              onChange={(next) => setQuatComponent(index, next)}
              min={-1}
              max={1}
              step={0.01}
              {...(index === 3 ? {} : { color: AXIS_COLORS[(['X', 'Y', 'Z'] as const)[index]] })}
            />
          ))}
          <div className="inline-actions">
            <button
              type="button"
              className="ghost-button"
              disabled={!quaternionDraft}
              onClick={applyQuaternion}
            >
              Apply quaternion
            </button>
          </div>
          <p className="hint">Components stay exactly as entered until you apply them; the result is then normalized.</p>
          <CopyableRow
            heading={`As Euler · ${describeSequence(conventions)}`}
            values={slots.map((slot) => ({
              name: slot.alias ?? slot.axis,
              value: `${fmt(euler[slot.index], conventions.angleUnit === 'deg' ? 3 : 5)}${unit}`,
              color: AXIS_COLORS[slot.axis],
            }))}
          />
        </>
      )}

      {locked && (
        <p className="note note--warn">
          Near gimbal lock: the {describeSequence(conventions)} triple is no longer unique here, so
          two very different-looking angle sets describe this same rotation. The quaternion is
          unaffected.
        </p>
      )}
    </div>
  );
}
