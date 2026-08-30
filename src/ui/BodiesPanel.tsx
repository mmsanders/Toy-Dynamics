import { GROUND_ID, type Vec3 } from '../types';
import { useModelStore } from '../store/useModelStore';
import { depthOf } from '../model/topology';
import { moveInertiaReference, checkInertia, tensorOf } from '../dyn/inertia';
import { v3 } from '../dyn/spatial';
import { unitLabel, lbmToSlug } from '../units';
import { AXIS_COLORS, COM_COLOR } from '../theme';
import { NumberField } from './NumberField';
import { Segmented } from './Segmented';
import { CopyableRow } from './CopyableRow';
import { RotationEditor } from './RotationEditor';
import { EmptyState, IconButton, ListRow, Note, Section, TextField } from './Bits';

/**
 * Bodies: mass properties and nodes.
 *
 * The inertia editor is the substantial part. A body's tensor can be stated about the
 * centre of mass or about the frame origin, and the panel always shows **both readings** —
 * the one being edited, and what the same body looks like from the other point. That turns
 * a setting you have to remember into a number you can read, which is the whole idea behind
 * this being a calculator.
 */

const AXIS_LABELS = ['X', 'Y', 'Z'] as const;

export function BodiesPanel() {
  const bodies = useModelStore((s) => s.bodies);
  const bodyOrder = useModelStore((s) => s.bodyOrder);
  const hinges = useModelStore((s) => s.hinges);
  const settings = useModelStore((s) => s.settings);
  const conventions = useModelStore((s) => s.conventions);
  const selectedId = useModelStore((s) => s.selectedBodyId);

  const selectBody = useModelStore((s) => s.selectBody);
  const addBody = useModelStore((s) => s.addBody);
  const removeBody = useModelStore((s) => s.removeBody);
  const renameBody = useModelStore((s) => s.renameBody);
  const setMass = useModelStore((s) => s.setMass);
  const setInertia = useModelStore((s) => s.setInertia);
  const setInertiaReference = useModelStore((s) => s.setInertiaReference);
  const convertInertiaReference = useModelStore((s) => s.convertInertiaReference);
  const toggleBodyVisible = useModelStore((s) => s.toggleBodyVisible);
  const addNode = useModelStore((s) => s.addNode);
  const removeNode = useModelStore((s) => s.removeNode);
  const renameNode = useModelStore((s) => s.renameNode);
  const setNodePosition = useModelStore((s) => s.setNodePosition);
  const setNodeOrientation = useModelStore((s) => s.setNodeOrientation);
  const setOriginNode = useModelStore((s) => s.setOriginNode);
  const setComNode = useModelStore((s) => s.setComNode);

  const body = bodies[selectedId];
  const massUnit = unitLabel(settings.units, 'mass');
  const lengthUnit = unitLabel(settings.units, 'length');
  const inertiaUnit = unitLabel(settings.units, 'inertia');

  return (
    <div className="stack">
      <Section
        title="Bodies"
        action={
          <IconButton label="Add a body" onClick={() => addBody()}>
            +
          </IconButton>
        }
      >
        {bodyOrder.map((id) => {
          const entry = bodies[id];
          if (!entry) return null;
          return (
            <ListRow
              key={id}
              label={entry.name}
              detail={entry.isGround ? 'inertial frame' : `${entry.nodeOrder.length} nodes`}
              color={entry.color}
              active={id === selectedId}
              indent={depthOf(hinges, id)}
              onSelect={() => selectBody(id)}
              actions={
                entry.isGround ? null : (
                  <>
                    <IconButton
                      label={entry.visible ? 'Hide' : 'Show'}
                      active={!entry.visible}
                      onClick={() => toggleBodyVisible(id)}
                    >
                      {entry.visible ? '◉' : '○'}
                    </IconButton>
                    <IconButton label="Delete" danger onClick={() => removeBody(id)}>
                      ×
                    </IconButton>
                  </>
                )
              }
            />
          );
        })}
      </Section>

      {!body && <EmptyState>Select a body to edit it.</EmptyState>}

      {body && body.id === GROUND_ID && (
        <Section title="Ground">
          <Note>
            Ground is the inertial frame. It has no mass and never moves; its nodes exist so hinges
            have somewhere to attach.
          </Note>
          <NodeList
            bodyId={body.id}
            body={body}
            lengthUnit={lengthUnit}
            conventions={conventions}
            onAdd={() => addNode(body.id)}
            onRemove={(nodeId) => removeNode(body.id, nodeId)}
            onRename={(nodeId, name) => renameNode(body.id, nodeId, name)}
            onPosition={(nodeId, position) => setNodePosition(body.id, nodeId, position)}
            onOrientation={(nodeId, q) => setNodeOrientation(body.id, nodeId, q)}
            onOrigin={(nodeId) => setOriginNode(body.id, nodeId)}
            onCom={(nodeId) => setComNode(body.id, nodeId)}
            allowCom={false}
          />
        </Section>
      )}

      {body && body.id !== GROUND_ID && (
        <>
          <Section title="Mass properties">
            <TextField label="Name" value={body.name} onChange={(name) => renameBody(body.id, name)} />
            <NumberField
              label="Mass"
              value={body.mass}
              onChange={(mass) => setMass(body.id, mass)}
              min={0}
              max={100}
              step={0.1}
              unit={massUnit}
            />
            {settings.units === 'imperial' && (
              <div className="inline-actions">
                <button
                  type="button"
                  className="ghost-button"
                  onClick={() => setMass(body.id, lbmToSlug(body.mass))}
                >
                  Convert {body.mass} lbm → slug
                </button>
                <span className="hint">
                  Imperial mass is slugs. Entering pounds-mass is off by 32.174.
                </span>
              </div>
            )}
          </Section>

          <Section title="Inertia">
            <Segmented
              label="Taken about"
              value={body.inertia.about}
              options={[
                { value: 'com', label: 'Centre of mass', title: 'Moments measured about the CoM' },
                { value: 'origin', label: 'Body origin', title: 'Moments measured about the frame origin' },
              ]}
              onChange={(about) => setInertiaReference(body.id, about)}
            />
            <Note>
              Switching this changes what the numbers <em>mean</em>, not the numbers. To keep the
              same physical body and rewrite the numbers instead, use Convert.
            </Note>

            {(['ixx', 'iyy', 'izz'] as const).map((key, i) => (
              <NumberField
                key={key}
                label={`I${AXIS_LABELS[i]!.toLowerCase()}${AXIS_LABELS[i]!.toLowerCase()}`}
                value={body.inertia[key]}
                onChange={(value) => setInertia(body.id, { [key]: value })}
                min={0}
                max={10}
                step={0.01}
                unit={inertiaUnit}
                color={AXIS_COLORS[AXIS_LABELS[i]!]}
              />
            ))}
            {(['ixy', 'ixz', 'iyz'] as const).map((key) => (
              <NumberField
                key={key}
                label={`I${key.slice(1)}`}
                hint="product"
                value={body.inertia[key]}
                onChange={(value) => setInertia(body.id, { [key]: value })}
                min={-2}
                max={2}
                step={0.01}
                unit={inertiaUnit}
              />
            ))}
            <Note>
              Products of inertia are stored as ∫xy dm, so the tensor carries them negated. The
              opposite sign convention is equally common — if the off-diagonals came from another
              tool, check which one it uses.
            </Note>

            <InertiaReadout body={body} inertiaUnit={inertiaUnit} lengthUnit={lengthUnit} />

            <div className="inline-actions">
              <button
                type="button"
                className="ghost-button"
                onClick={() =>
                  convertInertiaReference(body.id, body.inertia.about === 'com' ? 'origin' : 'com')
                }
              >
                Convert to {body.inertia.about === 'com' ? 'body origin' : 'centre of mass'}
              </button>
              <span className="hint">Rewrites the numbers; the physical body is unchanged.</span>
            </div>
          </Section>

          <Section title="Nodes">
            <NodeList
              bodyId={body.id}
              body={body}
              lengthUnit={lengthUnit}
              conventions={conventions}
              onAdd={() => addNode(body.id)}
              onRemove={(nodeId) => removeNode(body.id, nodeId)}
              onRename={(nodeId, name) => renameNode(body.id, nodeId, name)}
              onPosition={(nodeId, position) => setNodePosition(body.id, nodeId, position)}
              onOrientation={(nodeId, q) => setNodeOrientation(body.id, nodeId, q)}
              onOrigin={(nodeId) => setOriginNode(body.id, nodeId)}
              onCom={(nodeId) => setComNode(body.id, nodeId)}
              allowCom
            />
          </Section>
        </>
      )}
    </div>
  );
}

/**
 * The same body seen from the other reference point, plus its principal moments.
 *
 * The principal moments are worth showing on their own: they are what physicality is judged
 * on, and seeing them go negative is a much clearer signal than a warning about a tensor.
 */
function InertiaReadout({
  body,
  inertiaUnit,
  lengthUnit,
}: {
  body: NonNullable<ReturnType<typeof useModelStore.getState>['bodies'][string]>;
  inertiaUnit: string;
  lengthUnit: string;
}) {
  const com = body.nodes[body.comNodeId]?.position ?? [0, 0, 0];
  const offset = v3(com[0], com[1], com[2]);
  const zero = v3();
  const other =
    body.inertia.about === 'com'
      ? moveInertiaReference(body.mass, body.inertia, zero, offset)
      : moveInertiaReference(body.mass, body.inertia, offset, zero);
  const otherName = body.inertia.about === 'com' ? 'body origin' : 'centre of mass';

  const check = checkInertia(body.mass, tensorOf(body.inertia));
  const fmt = (value: number): string => {
    if (!Number.isFinite(value)) return '—';
    const rounded = Math.round(value * 1e6) / 1e6;
    return Object.is(rounded, -0) ? '0' : String(rounded);
  };

  return (
    <>
      <CopyableRow
        heading={`Same body about the ${otherName}`}
        values={[
          { name: 'Ixx', value: fmt(other.ixx), color: AXIS_COLORS.X },
          { name: 'Iyy', value: fmt(other.iyy), color: AXIS_COLORS.Y },
          { name: 'Izz', value: fmt(other.izz), color: AXIS_COLORS.Z },
        ]}
      />
      <CopyableRow
        heading={`Principal moments · ${inertiaUnit}`}
        values={[
          { name: 'I₁', value: fmt(check.principal[0]) },
          { name: 'I₂', value: fmt(check.principal[1]) },
          { name: 'I₃', value: fmt(check.principal[2]) },
          { name: 'k', value: `${fmt(check.radiusOfGyration)} ${lengthUnit}` },
        ]}
      />
      {!check.triangleInequality && (
        <Note tone="warn">
          I₁ + I₂ &lt; I₃. No physical mass distribution can do that — usually a sign error on a
          product of inertia.
        </Note>
      )}
    </>
  );
}

/** Node list with position, orientation, and the origin / centre-of-mass designations. */
function NodeList({
  body,
  lengthUnit,
  conventions,
  onAdd,
  onRemove,
  onRename,
  onPosition,
  onOrientation,
  onOrigin,
  onCom,
  allowCom,
}: {
  bodyId: string;
  body: NonNullable<ReturnType<typeof useModelStore.getState>['bodies'][string]>;
  lengthUnit: string;
  conventions: ReturnType<typeof useModelStore.getState>['conventions'];
  onAdd: () => void;
  onRemove: (nodeId: string) => void;
  onRename: (nodeId: string, name: string) => void;
  onPosition: (nodeId: string, position: Vec3) => void;
  onOrientation: (nodeId: string, q: [number, number, number, number]) => void;
  onOrigin: (nodeId: string) => void;
  onCom: (nodeId: string) => void;
  allowCom: boolean;
}) {
  const selectedNode = body.nodes[body.originNodeId];
  void selectedNode;

  return (
    <>
      <div className="inline-actions">
        <button type="button" className="ghost-button" onClick={onAdd}>
          Add node
        </button>
      </div>

      {body.nodeOrder.map((nodeId) => {
        const node = body.nodes[nodeId];
        if (!node) return null;
        const isOrigin = nodeId === body.originNodeId;
        const isCom = nodeId === body.comNodeId;

        return (
          <details key={nodeId} className="node" open={isOrigin || isCom}>
            <summary className="node__summary">
              <span className="node__name">{node.name}</span>
              {isOrigin && <span className="tag">origin</span>}
              {isCom && (
                <span className="tag" style={{ color: COM_COLOR, borderColor: COM_COLOR }}>
                  CoM
                </span>
              )}
            </summary>

            <div className="node__body">
              <TextField label="Name" value={node.name} onChange={(name) => onRename(nodeId, name)} />

              {isOrigin ? (
                <Note>
                  This node defines the body frame, so it sits at the origin by definition.
                  Designating a different node shifts every other node to match, leaving the body
                  exactly where it is in space.
                </Note>
              ) : (
                (['X', 'Y', 'Z'] as const).map((axis, i) => (
                  <NumberField
                    key={axis}
                    label={axis}
                    value={node.position[i]!}
                    onChange={(value) => {
                      const next: Vec3 = [...node.position];
                      next[i] = value;
                      onPosition(nodeId, next);
                    }}
                    min={-3}
                    max={3}
                    step={0.05}
                    unit={lengthUnit}
                    color={AXIS_COLORS[axis]}
                  />
                ))
              )}

              <RotationEditor
                label="Node axes"
                value={node.orientation}
                onChange={(q) => onOrientation(nodeId, q)}
                conventions={conventions}
              />

              <div className="inline-actions">
                {!isOrigin && (
                  <button type="button" className="ghost-button" onClick={() => onOrigin(nodeId)}>
                    Make origin
                  </button>
                )}
                {allowCom && !isCom && (
                  <button type="button" className="ghost-button" onClick={() => onCom(nodeId)}>
                    Make centre of mass
                  </button>
                )}
                {!isOrigin && !isCom && (
                  <button type="button" className="ghost-button is-danger" onClick={() => onRemove(nodeId)}>
                    Delete node
                  </button>
                )}
              </div>
            </div>
          </details>
        );
      })}
    </>
  );
}
