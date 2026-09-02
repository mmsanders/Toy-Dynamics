import { useRef, useState } from 'react';
import type { ContactHeightfield, ContactMaterial, Vec3 } from '../types';
import { MAX_HEIGHTFIELD_AXIS, MAX_HEIGHTFIELD_SAMPLES } from '../dyn/contact';
import { useModelStore } from '../store/useModelStore';
import { unitLabel } from '../units';
import { AXIS_COLORS } from '../theme';
import { NumberField } from './NumberField';
import { EmptyState, IconButton, ListRow, Note, Picker, Section, TextField, Toggle } from './Bits';

type Selection = { kind: 'sphere' | 'plane' | 'heightfield'; id: string } | null;

/** Authoring controls for the analytical compliant contact primitives. */
export function ContactPanel() {
  const bodies = useModelStore((s) => s.bodies);
  const spheres = useModelStore((s) => s.contactSpheres);
  const sphereOrder = useModelStore((s) => s.contactSphereOrder);
  const planes = useModelStore((s) => s.contactPlanes);
  const planeOrder = useModelStore((s) => s.contactPlaneOrder);
  const heightfields = useModelStore((s) => s.contactHeightfields);
  const heightfieldOrder = useModelStore((s) => s.contactHeightfieldOrder);
  const settings = useModelStore((s) => s.settings);
  const addSphere = useModelStore((s) => s.addContactSphere);
  const removeSphere = useModelStore((s) => s.removeContactSphere);
  const setSphere = useModelStore((s) => s.setContactSphere);
  const addPlane = useModelStore((s) => s.addContactPlane);
  const removePlane = useModelStore((s) => s.removeContactPlane);
  const setPlane = useModelStore((s) => s.setContactPlane);
  const setMaterial = useModelStore((s) => s.setContactMaterial);
  const addHeightfield = useModelStore((s) => s.addContactHeightfield);
  const removeHeightfield = useModelStore((s) => s.removeContactHeightfield);
  const setHeightfield = useModelStore((s) => s.setContactHeightfield);
  const setHeightfieldMaterial = useModelStore((s) => s.setContactHeightfieldMaterial);
  const [selected, setSelected] = useState<Selection>(null);

  const movable = Object.values(bodies).filter((body) => !body.isGround);
  const sphere = selected?.kind === 'sphere' ? spheres[selected.id] : undefined;
  const plane = selected?.kind === 'plane' ? planes[selected.id] : undefined;
  const heightfield = selected?.kind === 'heightfield' ? heightfields[selected.id] : undefined;
  const lengthUnit = unitLabel(settings.units, 'length');

  return (
    <div className="stack">
      <Section title="Contact spheres" action={<IconButton label="Add a contact sphere" disabled={!movable.length} onClick={() => { const id = addSphere(); if (id) setSelected({ kind: 'sphere', id }); }}>+</IconButton>}>
        {!sphereOrder.length && <EmptyState>No spheres. Add one at a body node to collide with planes or other spheres.</EmptyState>}
        {sphereOrder.map((id) => {
          const entry = spheres[id];
          if (!entry) return null;
          return <ListRow key={id} label={entry.name} detail={`r ${entry.radius} · ${bodies[entry.bodyId]?.name ?? '?'}`} active={selected?.kind === 'sphere' && selected.id === id} onSelect={() => setSelected({ kind: 'sphere', id })} actions={<><IconButton label={entry.enabled ? 'Disable' : 'Enable'} active={!entry.enabled} onClick={() => setSphere(id, { enabled: !entry.enabled })}>{entry.enabled ? '◉' : '○'}</IconButton><IconButton label="Delete" danger onClick={() => removeSphere(id)}>×</IconButton></>} />;
        })}
      </Section>

      <Section title="World planes" action={<IconButton label="Add a contact plane" onClick={() => { const id = addPlane(); setSelected({ kind: 'plane', id }); }}>+</IconButton>}>
        {!planeOrder.length && <EmptyState>No planes. Multiple one-sided planes can form floors, ramps, boxes, or a V.</EmptyState>}
        {planeOrder.map((id) => {
          const entry = planes[id];
          if (!entry) return null;
          return <ListRow key={id} label={entry.name} detail={`n [${entry.normal.join(', ')}] · ${entry.bounded ? `${entry.size} wide` : 'unbounded'}`} active={selected?.kind === 'plane' && selected.id === id} onSelect={() => setSelected({ kind: 'plane', id })} actions={<><IconButton label={entry.enabled ? 'Disable' : 'Enable'} active={!entry.enabled} onClick={() => setPlane(id, { enabled: !entry.enabled })}>{entry.enabled ? '◉' : '○'}</IconButton><IconButton label="Delete" danger onClick={() => removePlane(id)}>×</IconButton></>} />;
        })}
      </Section>

      <Section title="Heightfields" action={<IconButton label="Add a heightfield" onClick={() => { const id = addHeightfield(); setSelected({ kind: 'heightfield', id }); }}>+</IconButton>}>
        {!heightfieldOrder.length && <EmptyState>No heightfields. Add a regular grid for terrain that varies over world X/Y.</EmptyState>}
        {heightfieldOrder.map((id) => {
          const entry = heightfields[id];
          if (!entry) return null;
          return <ListRow key={id} label={entry.name} detail={`${entry.columns} × ${entry.rows} · Δ ${entry.spacing}`} active={selected?.kind === 'heightfield' && selected.id === id} onSelect={() => setSelected({ kind: 'heightfield', id })} actions={<><IconButton label={entry.enabled ? 'Disable' : 'Enable'} active={!entry.enabled} onClick={() => setHeightfield(id, { enabled: !entry.enabled })}>{entry.enabled ? '◉' : '○'}</IconButton><IconButton label="Delete" danger onClick={() => removeHeightfield(id)}>×</IconButton></>} />;
        })}
      </Section>

      {!sphere && !plane && !heightfield && <EmptyState>Select a sphere, plane, or heightfield to edit it.</EmptyState>}
      {sphere && <>
        <Section title="Sphere geometry">
          <TextField label="Name" value={sphere.name} onChange={(name) => setSphere(sphere.id, { name })} />
          <Picker label="On body" value={sphere.bodyId} options={movable.map((body) => ({ value: body.id, label: body.name }))} onChange={(bodyId) => setSphere(sphere.id, { bodyId })} />
          <Picker label="At node" value={sphere.nodeId} options={(bodies[sphere.bodyId]?.nodeOrder ?? []).map((id) => ({ value: id, label: bodies[sphere.bodyId]?.nodes[id]?.name ?? id }))} onChange={(nodeId) => setSphere(sphere.id, { nodeId })} />
          <NumberField label="Radius" value={sphere.radius} onChange={(radius) => setSphere(sphere.id, { radius })} min={0} max={10} step={0.01} unit={lengthUnit} />
          <Note>A zero radius is an exact point contact. Small positive radii are usually more forgiving.</Note>
        </Section>
        <MaterialEditor material={sphere.material} onChange={(patch) => setMaterial('sphere', sphere.id, patch)} />
      </>}
      {plane && <>
        <Section title="Plane geometry">
          <TextField label="Name" value={plane.name} onChange={(name) => setPlane(plane.id, { name })} />
          <VectorEditor label="Point" value={plane.point} unit={lengthUnit} onChange={(point) => setPlane(plane.id, { point })} />
          <VectorEditor label="Allowed-side normal" value={plane.normal} onChange={(normal) => setPlane(plane.id, { normal })} />
          <Toggle
            label="Finite plate"
            checked={plane.bounded}
            onChange={(bounded) => setPlane(plane.id, { bounded })}
            hint={plane.bounded ? 'Contact stops at the drawn edge' : 'Contact extends forever; the square is only what is drawn'}
          />
          <NumberField label="Size" value={plane.size} onChange={(size) => setPlane(plane.id, { size: Math.max(0.01, size) })} min={0.1} max={20} step={0.1} unit={lengthUnit} />
          <Note>The arrow in the scene points to the allowed side; the solver normalizes the normal for you. A finite plate is solid out to its edge and nothing past it, so a sphere rolling off one tips over the rim and falls. An unbounded plane ignores the size entirely except for drawing.</Note>
        </Section>
        <MaterialEditor material={plane.material} onChange={(patch) => setMaterial('plane', plane.id, patch)} />
      </>}
      {heightfield && <>
        <Section title="Heightfield geometry">
          <TextField label="Name" value={heightfield.name} onChange={(name) => setHeightfield(heightfield.id, { name })} />
          <VectorEditor label="Grid origin" value={heightfield.origin} unit={lengthUnit} onChange={(origin) => setHeightfield(heightfield.id, { origin })} />
          <NumberField label="Sample spacing" value={heightfield.spacing} onChange={(spacing) => setHeightfield(heightfield.id, { spacing })} min={0.000001} max={1000} step={0.1} unit={lengthUnit} />
          <NumberField label="Columns" value={heightfield.columns} onChange={(columns) => setHeightfield(heightfield.id, { columns })} min={2} max={MAX_HEIGHTFIELD_AXIS} step={1} />
          <NumberField label="Rows" value={heightfield.rows} onChange={(rows) => setHeightfield(heightfield.id, { rows })} min={2} max={MAX_HEIGHTFIELD_AXIS} step={1} />
          <HeightGridEditor field={heightfield} onChange={(patch) => setHeightfield(heightfield.id, patch)} />
          <Note>World Z is the height axis. Contact is bilinear and continuous across cells; outside the grid or across a no-data cell there is no contact. Keep sphere radii near or below the sample spacing.</Note>
        </Section>
        <MaterialEditor material={heightfield.material} onChange={(patch) => setHeightfieldMaterial(heightfield.id, patch)} />
      </>}
    </div>
  );
}

const gridText = (field: ContactHeightfield): string =>
  Array.from({ length: field.rows }, (_, row) =>
    field.heights.slice(row * field.columns, (row + 1) * field.columns)
      .map((height) => height === null ? 'x' : String(height)).join(', '),
  ).join('\n');

function HeightGridEditor({ field, onChange }: { field: ContactHeightfield; onChange: (patch: Partial<ContactHeightfield>) => void }) {
  const textarea = useRef<HTMLTextAreaElement>(null);
  const [error, setError] = useState<string | null>(null);

  const applyText = (text = textarea.current?.value ?? '') => {
    const lines = text.trim().split(/\n+/).map((line) => line.trim()).filter(Boolean);
    const rows = lines.map((line) => line.split(/[\s,]+/).filter(Boolean));
    const columns = rows[0]?.length ?? 0;
    if (rows.length < 2 || columns < 2 || rows.some((row) => row.length !== columns)
        || rows.length > MAX_HEIGHTFIELD_AXIS || columns > MAX_HEIGHTFIELD_AXIS
        || rows.length * columns > MAX_HEIGHTFIELD_SAMPLES) {
      setError(`Use a rectangular 2 × 2 to ${MAX_HEIGHTFIELD_AXIS} × ${MAX_HEIGHTFIELD_AXIS} grid (at most ${MAX_HEIGHTFIELD_SAMPLES} samples).`);
      return;
    }
    const heights = rows.flatMap((row) => row.map((token) => {
      if (/^(x|null|nodata|-)$/i.test(token)) return null;
      const value = Number(token);
      return Number.isFinite(value) ? value : null;
    }));
    setError(null);
    onChange({ columns, rows: rows.length, heights });
  };

  const preset = (kind: 'flat' | 'slope' | 'hill') => {
    const columns = 9;
    const rows = 9;
    const heights = Array.from({ length: columns * rows }, (_, index) => {
      const x = (index % columns) - 4;
      const y = Math.floor(index / columns) - 4;
      if (kind === 'slope') return 0.1 * x;
      if (kind === 'hill') return Math.exp(-(x * x + y * y) / 8);
      return 0;
    });
    setError(null);
    onChange({ columns, rows, heights });
  };

  return <div className="height-grid">
    <span className="text-field__label">Height samples · rows in Y</span>
    <textarea ref={textarea} key={`${field.columns}:${field.rows}:${field.heights.join(',')}`} aria-label="Height samples" defaultValue={gridText(field)} onBlur={(event) => applyText(event.currentTarget.value)} rows={Math.min(9, field.rows)} spellCheck={false} />
    <div className="inline-actions">
      <button type="button" className="ghost-button" onClick={() => preset('flat')}>Flat</button>
      <button type="button" className="ghost-button" onClick={() => preset('slope')}>Slope</button>
      <button type="button" className="ghost-button" onClick={() => preset('hill')}>Hill</button>
      <button type="button" className="ghost-button" onClick={() => applyText()}>Apply grid</button>
    </div>
    {error && <Note tone="warn">{error}</Note>}
    <Note>Paste comma- or space-separated rows. Use x, null, noData, or - for a hole.</Note>
  </div>;
}

function VectorEditor({ label, value, unit, onChange }: { label: string; value: Vec3; unit?: string; onChange: (value: Vec3) => void }) {
  return <div><p className="hint">{label}</p>{(['X', 'Y', 'Z'] as const).map((axis, index) => <NumberField key={axis} label={axis} value={value[index]!} onChange={(number) => { const next: Vec3 = [...value]; next[index] = number; onChange(next); }} min={-100} max={100} step={0.1} {...(unit ? { unit } : {})} color={AXIS_COLORS[axis]} />)}</div>;
}

function MaterialEditor({ material, onChange }: { material: ContactMaterial; onChange: (patch: Partial<ContactMaterial>) => void }) {
  return <Section title="Compliant material">
    <NumberField label="Stiffness" value={material.stiffness} onChange={(stiffness) => onChange({ stiffness })} min={0} max={1e7} step={100} />
    <NumberField label="Damping" value={material.damping} onChange={(damping) => onChange({ damping })} min={0} max={1e5} step={10} />
    <NumberField label="Sliding friction" value={material.friction} onChange={(friction) => onChange({ friction })} min={0} max={10} step={0.05} />
    <NumberField label="Friction velocity" value={material.frictionVelocity} onChange={(frictionVelocity) => onChange({ frictionVelocity })} min={0.000001} max={100} step={0.001} />
    <Note>Pair properties use the lower value. Friction is regularized kinetic friction, not static sticking; the velocity scale controls smoothing near zero slip.</Note>
  </Section>;
}
