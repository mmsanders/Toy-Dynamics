import { useState } from 'react';
import type { ContactMaterial, Vec3 } from '../types';
import { useModelStore } from '../store/useModelStore';
import { unitLabel } from '../units';
import { AXIS_COLORS } from '../theme';
import { NumberField } from './NumberField';
import { EmptyState, IconButton, ListRow, Note, Picker, Section, TextField, Toggle } from './Bits';

type Selection = { kind: 'sphere' | 'plane'; id: string } | null;

/** Authoring controls for the analytical compliant contact primitives. */
export function ContactPanel() {
  const bodies = useModelStore((s) => s.bodies);
  const spheres = useModelStore((s) => s.contactSpheres);
  const sphereOrder = useModelStore((s) => s.contactSphereOrder);
  const planes = useModelStore((s) => s.contactPlanes);
  const planeOrder = useModelStore((s) => s.contactPlaneOrder);
  const settings = useModelStore((s) => s.settings);
  const addSphere = useModelStore((s) => s.addContactSphere);
  const removeSphere = useModelStore((s) => s.removeContactSphere);
  const setSphere = useModelStore((s) => s.setContactSphere);
  const addPlane = useModelStore((s) => s.addContactPlane);
  const removePlane = useModelStore((s) => s.removeContactPlane);
  const setPlane = useModelStore((s) => s.setContactPlane);
  const setMaterial = useModelStore((s) => s.setContactMaterial);
  const [selected, setSelected] = useState<Selection>(null);

  const movable = Object.values(bodies).filter((body) => !body.isGround);
  const sphere = selected?.kind === 'sphere' ? spheres[selected.id] : undefined;
  const plane = selected?.kind === 'plane' ? planes[selected.id] : undefined;
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

      {!sphere && !plane && <EmptyState>Select a sphere or plane to edit it.</EmptyState>}
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
    </div>
  );
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
