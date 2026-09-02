import { useModelStore } from '../store/useModelStore';
import { unitLabel } from '../units';
import { NumberField } from './NumberField';
import { EmptyState, IconButton, ListRow, Note, Picker, Section, TextField } from './Bits';

/** Editing surface for passive devices, deliberately separate from time-profiled actuators. */
export function SpringDampersPanel() {
  const bodies = useModelStore((s) => s.bodies);
  const devices = useModelStore((s) => s.springDampers);
  const order = useModelStore((s) => s.springDamperOrder);
  const selectedId = useModelStore((s) => s.selectedSpringDamperId);
  const settings = useModelStore((s) => s.settings);
  const select = useModelStore((s) => s.selectSpringDamper);
  const add = useModelStore((s) => s.addSpringDamper);
  const remove = useModelStore((s) => s.removeSpringDamper);
  const rename = useModelStore((s) => s.renameSpringDamper);
  const setEndpoint = useModelStore((s) => s.setSpringDamperEndpoint);
  const setDevice = useModelStore((s) => s.setSpringDamper);
  const toggle = useModelStore((s) => s.toggleSpringDamper);

  const device = selectedId ? devices[selectedId] : undefined;
  const bodyA = device ? bodies[device.bodyAId] : undefined;
  const bodyB = device ? bodies[device.bodyBId] : undefined;
  const allBodies = Object.values(bodies);
  const canAdd = allBodies.some((body) => !body.isGround);

  const endpointEditor = (
    end: 'a' | 'b',
    bodyId: string,
    nodeId: string,
    otherBodyId: string,
  ) => {
    const body = bodies[bodyId];
    if (!body || !device) return null;
    const title = end === 'a' ? 'End A' : 'End B';
    return (
      <div className="stack stack--tight">
        <Picker
          label={`${title} body`}
          value={bodyId}
          options={allBodies
            .filter((candidate) => candidate.id !== otherBodyId)
            .map((candidate) => ({ value: candidate.id, label: candidate.name }))}
          onChange={(nextBodyId) => setEndpoint(device.id, end, nextBodyId)}
        />
        <Picker
          label={`${title} node`}
          value={nodeId}
          options={body.nodeOrder.map((id) => ({ value: id, label: body.nodes[id]?.name ?? id }))}
          onChange={(nextNodeId) => setEndpoint(device.id, end, bodyId, nextNodeId)}
        />
      </div>
    );
  };

  return (
    <div className="stack">
      <Section
        title="Passive"
        action={
          <IconButton label="Add a passive device" onClick={() => add()} disabled={!canAdd}>
            +
          </IconButton>
        }
      >
        {order.length === 0 && (
          <EmptyState>No passive devices. Add one to connect a pair of nodes on different bodies.</EmptyState>
        )}
        {order.map((id) => {
          const entry = devices[id];
          if (!entry) return null;
          const a = bodies[entry.bodyAId]?.name ?? '?';
          const b = bodies[entry.bodyBId]?.name ?? '?';
          return (
            <ListRow
              key={id}
              label={entry.name}
              detail={`${a} ↔ ${b} · k ${entry.stiffness.toPrecision(3)}`}
              color={entry.color}
              active={id === selectedId}
              onSelect={() => select(id)}
              actions={
                <>
                  <IconButton label={entry.enabled ? 'Disable' : 'Enable'} active={!entry.enabled} onClick={() => toggle(id)}>
                    {entry.enabled ? '◉' : '○'}
                  </IconButton>
                  <IconButton label="Delete" danger onClick={() => remove(id)}>×</IconButton>
                </>
              }
            />
          );
        })}
      </Section>

      {!device && <EmptyState>Select a passive device to edit it.</EmptyState>}

      {device && bodyA && bodyB && (
        <>
          <Section title="Endpoints">
            <TextField label="Name" value={device.name} onChange={(name) => rename(device.id, name)} />
            {endpointEditor('a', device.bodyAId, device.nodeAId, device.bodyBId)}
            {endpointEditor('b', device.bodyBId, device.nodeBId, device.bodyAId)}
            <Note>
              A spring-damper always connects two different bodies. Ground is available as a fixed anchor.
            </Note>
          </Section>

          <Section title="Properties">
            <NumberField
              label="Stiffness"
              value={device.stiffness}
              onChange={(stiffness) => setDevice(device.id, { stiffness })}
              min={0}
              max={1e7}
              step={10}
              unit={unitLabel(settings.units, 'linearStiffness')}
            />
            <NumberField
              label="Damping"
              value={device.damping}
              onChange={(damping) => setDevice(device.id, { damping })}
              min={0}
              max={1e7}
              step={0.1}
              unit={unitLabel(settings.units, 'linearDamping')}
            />
            <NumberField
              label="Rest length"
              value={device.restLength}
              onChange={(restLength) => setDevice(device.id, { restLength })}
              min={0}
              max={1e7}
              step={0.01}
              unit={unitLabel(settings.units, 'length')}
            />
            <Note>
              The spring pulls when longer than its rest length and pushes when shorter. Damping resists the endpoints moving apart or together.
            </Note>
          </Section>
        </>
      )}
    </div>
  );
}
