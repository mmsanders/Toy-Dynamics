import { create } from 'zustand';
import { createJSONStorage, persist } from 'zustand/middleware';
import {
  GROUND_ID,
  type Actuator,
  type ActuatorFrame,
  type ActuatorKind,
  type Body,
  type Conventions,
  type ContactMaterial,
  type ContactPlane,
  type ContactSphere,
  type DofSpec,
  type Hinge,
  type Inertia,
  type InertiaReference,
  type Node,
  type Profile,
  type Quat,
  type SimSettings,
  type UnitSystem,
  type Vec3,
} from '../types';
import {
  ACTUATOR_COLORS,
  BODY_COLORS,
  IDENTITY_QUAT,
  initialModel,
  makeNode,
  neutralDof,
  neutralDofSet,
  type ModelSlice,
} from './defaults';
import { inboundHinge, wouldCreateCycle } from '../model/topology';
import { moveInertiaReference } from '../dyn/inertia';
import { chainPoses, invertPose, jointPose, type Pose } from '../math/transforms';
import { gravityOnSystemChange } from '../units';
import type { DiagnosticFix } from '../model/diagnostics';
import { repairModel, type ModelPersisted } from './modelRepair';
import { v3 } from '../dyn/spatial';

/**
 * The edited model.
 *
 * Everything derived — world poses, the solver model, diagnostics, the trajectory — is
 * computed from this rather than stored alongside it, so there is exactly one source of
 * truth and no way for a cached copy to drift.
 *
 * Two invariants the rest of the app relies on:
 *
 *  - **Every non-ground body has exactly one inbound hinge.** That hinge is its parent
 *    pointer; there is no separate field to fall out of sync, and no way to express a
 *    closed loop.
 *  - **A body's origin node sits at [0, 0, 0] by construction.** Designating a different
 *    node as the origin re-expresses every other node so the body does not move in space.
 */

export type ModelState = ModelSlice & {
  selectBody: (id: string) => void;
  addBody: (parentBodyId?: string) => string;
  removeBody: (id: string) => void;
  renameBody: (id: string, name: string) => void;
  setMass: (id: string, mass: number) => void;
  setInertia: (id: string, patch: Partial<Inertia>) => void;
  /** Reinterpret the same numbers about the other point. The physics changes. */
  setInertiaReference: (id: string, about: InertiaReference) => void;
  /** Re-express the numbers about the other point. The physics does not change. */
  convertInertiaReference: (id: string, about: InertiaReference) => void;
  toggleBodyVisible: (id: string) => void;

  addNode: (bodyId: string) => string | null;
  removeNode: (bodyId: string, nodeId: string) => void;
  renameNode: (bodyId: string, nodeId: string, name: string) => void;
  setNodePosition: (bodyId: string, nodeId: string, position: Vec3) => void;
  setNodeOrientation: (bodyId: string, nodeId: string, orientation: Quat) => void;
  setOriginNode: (bodyId: string, nodeId: string) => void;
  setComNode: (bodyId: string, nodeId: string) => void;

  selectHinge: (id: string | null) => void;
  renameHinge: (id: string, name: string) => void;
  setHingeParent: (id: string, parentBodyId: string, parentNodeId?: string) => void;
  setHingeParentNode: (id: string, nodeId: string) => void;
  setHingeChildNode: (id: string, nodeId: string) => void;
  setHingeMount: (id: string, mount: Quat) => void;
  setDof: (hingeId: string, axis: number, patch: Partial<DofSpec>) => void;
  setDofLimit: (hingeId: string, axis: number, patch: Partial<DofSpec['limit']>) => void;
  resetHingeDof: (hingeId: string) => void;

  selectActuator: (id: string | null) => void;
  addActuator: (bodyId?: string) => string | null;
  removeActuator: (id: string) => void;
  renameActuator: (id: string, name: string) => void;
  setActuatorTarget: (id: string, bodyId: string, nodeId?: string) => void;
  setActuatorNode: (id: string, nodeId: string) => void;
  setActuatorKind: (id: string, kind: ActuatorKind) => void;
  setActuatorFrame: (id: string, frame: ActuatorFrame) => void;
  setActuatorVector: (id: string, vector: Vec3) => void;
  setActuatorProfile: (id: string, profile: Profile) => void;
  toggleActuator: (id: string) => void;

  addContactSphere: (bodyId?: string) => string | null;
  removeContactSphere: (id: string) => void;
  setContactSphere: (id: string, patch: Partial<ContactSphere>) => void;
  addContactPlane: () => string;
  removeContactPlane: (id: string) => void;
  setContactPlane: (id: string, patch: Partial<ContactPlane>) => void;
  setContactMaterial: (kind: 'sphere' | 'plane', id: string, patch: Partial<ContactMaterial>) => void;

  setSettings: (patch: Partial<SimSettings>) => void;
  setUnits: (units: UnitSystem) => void;
  setConventions: (patch: Partial<Conventions>) => void;
  applyFix: (fix: DiagnosticFix) => void;
  resetModel: () => void;
  loadModel: (model: ModelPersisted) => void;
};

/** A `Storage`-shaped stub, for environments without one. */
function memoryStorage(): Storage {
  const map = new Map<string, string>();
  return {
    get length() {
      return map.size;
    },
    clear: () => map.clear(),
    getItem: (key) => map.get(key) ?? null,
    key: (index) => [...map.keys()][index] ?? null,
    removeItem: (key) => void map.delete(key),
    setItem: (key, value) => void map.set(key, value),
  };
}

let counter = 0;
const nextId = (prefix: string): string =>
  `${prefix}-${Date.now().toString(36)}-${(counter++).toString(36)}`;

function uniqueName(existing: { name: string }[], base: string): string {
  const taken = new Set(existing.map((item) => item.name));
  if (!taken.has(base)) return base;
  for (let i = 2; ; i++) {
    const candidate = `${base} ${i}`;
    if (!taken.has(candidate)) return candidate;
  }
}

function pickColor(existing: { color: string }[], palette: string[]): string {
  const used = new Set(existing.map((item) => item.color));
  return palette.find((c) => !used.has(c)) ?? palette[0]!;
}

const subtract = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];

/** Shorthand for a shallow body update that leaves the rest of the state alone. */
const patchBody = (state: ModelSlice, id: string, next: Body): Partial<ModelSlice> => ({
  bodies: { ...state.bodies, [id]: next },
});

/**
 * The pose of a node on a removed body, expressed in its grandparent's body frame.
 *
 * Deleting a body must not teleport whatever hung off it. The chain from the grandparent's
 * attachment node down to the node the child was using is a fixed sequence of poses once
 * the removed body's joint is evaluated at its initial configuration, so it can be folded
 * into a single node planted on the grandparent. The child then hangs off that node in
 * exactly the place it already occupied.
 *
 * This preserves the *initial* frame exactly. If the removed body's own joint was free, the
 * child obviously stops following it — there is nothing left to follow.
 */
function foldThroughRemovedBody(
  removed: Body,
  removedHinge: Hinge,
  parentNode: Node,
  attachmentNode: Node,
): Pose {
  const childNode = removed.nodes[removedHinge.childNodeId];
  const nodeToBody: Pose = childNode
    ? invertPose({ position: childNode.position, orientation: childNode.orientation })
    : { position: [0, 0, 0], orientation: [...IDENTITY_QUAT] as Quat };

  return chainPoses([
    { position: parentNode.position, orientation: parentNode.orientation },
    { position: [0, 0, 0], orientation: removedHinge.mount },
    jointPose(removedHinge.dof),
    nodeToBody,
    { position: attachmentNode.position, orientation: attachmentNode.orientation },
  ]);
}

export function modelSnapshot(state: ModelState): ModelPersisted {
  return {
    bodies: state.bodies,
    bodyOrder: state.bodyOrder,
    hinges: state.hinges,
    hingeOrder: state.hingeOrder,
    actuators: state.actuators,
    actuatorOrder: state.actuatorOrder,
    contactSpheres: state.contactSpheres,
    contactSphereOrder: state.contactSphereOrder,
    contactPlanes: state.contactPlanes,
    contactPlaneOrder: state.contactPlaneOrder,
    settings: state.settings,
    conventions: state.conventions,
    selectedBodyId: state.selectedBodyId,
    selectedHingeId: state.selectedHingeId,
    selectedActuatorId: state.selectedActuatorId,
  };
}

export const useModelStore = create<ModelState>()(
  persist(
    (set, get) => ({
      ...initialModel(),

      // --- bodies ---------------------------------------------------------------------

      selectBody: (id) =>
        set((state) => ({
          selectedBodyId: id,
          // Following the selection to the body's own hinge is almost always what you want
          // next, and saves a trip to the other tab.
          selectedHingeId: inboundHinge(state.hinges, id)?.id ?? state.selectedHingeId,
        })),

      addBody: (parentBodyId) => {
        const id = nextId('body');
        set((state) => {
          const parent =
            parentBodyId && state.bodies[parentBodyId]
              ? parentBodyId
              : (state.selectedBodyId ?? GROUND_ID);
          const parentBody = state.bodies[parent] ?? state.bodies[GROUND_ID]!;

          const root = makeNode(nextId('node'), 'Root');
          const com = makeNode(nextId('node'), 'CoM', [0, 0, -0.4]);
          const tip = makeNode(nextId('node'), 'Tip', [0, 0, -0.8]);
          const body: Body = {
            id,
            name: uniqueName(Object.values(state.bodies), `Body ${state.bodyOrder.length}`),
            nodes: { [root.id]: root, [com.id]: com, [tip.id]: tip },
            nodeOrder: [root.id, com.id, tip.id],
            originNodeId: root.id,
            comNodeId: com.id,
            mass: 1,
            inertia: { about: 'com', ixx: 0.1, iyy: 0.1, izz: 0.02, ixy: 0, ixz: 0, iyz: 0 },
            color: pickColor(Object.values(state.bodies), BODY_COLORS),
            visible: true,
          };

          const dof = neutralDofSet();
          dof[4] = neutralDof(true);
          const hinge: Hinge = {
            id: nextId('hinge'),
            name: uniqueName(Object.values(state.hinges), `Hinge ${state.hingeOrder.length + 1}`),
            parentBodyId: parent,
            // Attach at the parent's last node, which is the far end of a chain body — the
            // usual place to extend from.
            parentNodeId: parentBody.nodeOrder[parentBody.nodeOrder.length - 1] ?? parentBody.originNodeId,
            childBodyId: id,
            childNodeId: root.id,
            mount: [...IDENTITY_QUAT] as Quat,
            dof,
          };

          return {
            bodies: { ...state.bodies, [id]: body },
            bodyOrder: [...state.bodyOrder, id],
            hinges: { ...state.hinges, [hinge.id]: hinge },
            hingeOrder: [...state.hingeOrder, hinge.id],
            selectedBodyId: id,
            selectedHingeId: hinge.id,
          };
        });
        return id;
      },

      removeBody: (id) =>
        set((state) => {
          const body = state.bodies[id];
          if (!body || body.isGround) return state;

          const own = inboundHinge(state.hinges, id);
          const grandparentId = own?.parentBodyId ?? GROUND_ID;
          const grandparent = state.bodies[grandparentId];

          const bodies = { ...state.bodies };
          const hinges = { ...state.hinges };
          delete bodies[id];
          if (own) delete hinges[own.id];

          // Re-home the children, folding the removed body's geometry into a new node on
          // the grandparent so nothing jumps.
          if (grandparent && own) {
            const parentNode = grandparent.nodes[own.parentNodeId];
            const rehomed: Record<string, Node> = { ...grandparent.nodes };
            const rehomedOrder = [...grandparent.nodeOrder];

            for (const hinge of Object.values(hinges)) {
              if (hinge.parentBodyId !== id) continue;
              const attachment = body.nodes[hinge.parentNodeId];
              if (!parentNode || !attachment) {
                hinges[hinge.id] = { ...hinge, parentBodyId: grandparentId, parentNodeId: grandparent.originNodeId };
                continue;
              }
              const pose = foldThroughRemovedBody(body, own, parentNode, attachment);
              const carried = makeNode(nextId('node'), uniqueName(Object.values(rehomed), `${body.name} · ${attachment.name}`), pose.position);
              carried.orientation = pose.orientation;
              rehomed[carried.id] = carried;
              rehomedOrder.push(carried.id);
              hinges[hinge.id] = { ...hinge, parentBodyId: grandparentId, parentNodeId: carried.id };
            }

            bodies[grandparentId] = { ...grandparent, nodes: rehomed, nodeOrder: rehomedOrder };
          }

          // Actuators on the deleted body go with it — there is nothing left to push on.
          const actuators = { ...state.actuators };
          for (const actuator of Object.values(actuators)) {
            if (actuator.bodyId === id) delete actuators[actuator.id];
          }
          const actuatorOrder = state.actuatorOrder.filter((a) => actuators[a]);
          const contactSpheres = { ...state.contactSpheres };
          for (const sphere of Object.values(contactSpheres)) {
            if (sphere.bodyId === id) delete contactSpheres[sphere.id];
          }
          const contactSphereOrder = state.contactSphereOrder.filter((sphere) => contactSpheres[sphere]);

          const bodyOrder = state.bodyOrder.filter((b) => b !== id);
          const hingeOrder = state.hingeOrder.filter((h) => hinges[h]);
          const fallbackBody = bodyOrder[bodyOrder.length - 1] ?? GROUND_ID;

          return {
            bodies,
            bodyOrder,
            hinges,
            hingeOrder,
            actuators,
            actuatorOrder,
            contactSpheres,
            contactSphereOrder,
            selectedBodyId: state.selectedBodyId === id ? fallbackBody : state.selectedBodyId,
            selectedHingeId:
              state.selectedHingeId && hinges[state.selectedHingeId]
                ? state.selectedHingeId
                : (hingeOrder[hingeOrder.length - 1] ?? null),
            selectedActuatorId:
              state.selectedActuatorId && actuators[state.selectedActuatorId]
                ? state.selectedActuatorId
                : (actuatorOrder[actuatorOrder.length - 1] ?? null),
          };
        }),

      renameBody: (id, name) =>
        set((state) => {
          const body = state.bodies[id];
          return body ? patchBody(state, id, { ...body, name }) : state;
        }),

      setMass: (id, mass) =>
        set((state) => {
          const body = state.bodies[id];
          return body ? patchBody(state, id, { ...body, mass }) : state;
        }),

      setInertia: (id, patch) =>
        set((state) => {
          const body = state.bodies[id];
          return body ? patchBody(state, id, { ...body, inertia: { ...body.inertia, ...patch } }) : state;
        }),

      /**
       * Change what the numbers are taken to mean, leaving them alone.
       *
       * The same discipline Rotation Wizard applies to vector kinds: a toggle changes the
       * interpretation, never the data. `convertInertiaReference` is the other operation,
       * for when you want the numbers rewritten instead.
       */
      setInertiaReference: (id, about) =>
        set((state) => {
          const body = state.bodies[id];
          return body ? patchBody(state, id, { ...body, inertia: { ...body.inertia, about } }) : state;
        }),

      /** Re-express the tensor about the other point, so the physical body is unchanged. */
      convertInertiaReference: (id, about) =>
        set((state) => {
          const body = state.bodies[id];
          if (!body || body.inertia.about === about) return state;
          const com = body.nodes[body.comNodeId]?.position ?? [0, 0, 0];
          const zero = v3();
          const offset = v3(com[0], com[1], com[2]);
          // Going to 'origin' shifts the reference from the CoM out to the origin, which is
          // the CoM measured from the origin; going the other way is the reverse.
          const converted =
            about === 'origin'
              ? moveInertiaReference(body.mass, body.inertia, zero, offset)
              : moveInertiaReference(body.mass, body.inertia, offset, zero);
          return patchBody(state, id, { ...body, inertia: { about, ...converted } });
        }),

      toggleBodyVisible: (id) =>
        set((state) => {
          const body = state.bodies[id];
          return body ? patchBody(state, id, { ...body, visible: !body.visible }) : state;
        }),

      // --- nodes ----------------------------------------------------------------------

      addNode: (bodyId) => {
        const body = get().bodies[bodyId];
        if (!body) return null;
        const id = nextId('node');
        set((state) => {
          const target = state.bodies[bodyId];
          if (!target) return state;
          const node = makeNode(id, uniqueName(Object.values(target.nodes), `Node ${target.nodeOrder.length + 1}`));
          return patchBody(state, bodyId, {
            ...target,
            nodes: { ...target.nodes, [id]: node },
            nodeOrder: [...target.nodeOrder, id],
          });
        });
        return id;
      },

      removeNode: (bodyId, nodeId) =>
        set((state) => {
          const body = state.bodies[bodyId];
          if (!body || body.nodeOrder.length <= 1) return state;
          // The origin and centre of mass are structural; they are reassigned, not deleted.
          if (nodeId === body.originNodeId || nodeId === body.comNodeId) return state;
          // A node a hinge or actuator hangs off cannot vanish underneath it.
          const inUse =
            Object.values(state.hinges).some(
              (h) =>
                (h.parentBodyId === bodyId && h.parentNodeId === nodeId) ||
                (h.childBodyId === bodyId && h.childNodeId === nodeId),
            ) || Object.values(state.actuators).some((a) => a.bodyId === bodyId && a.nodeId === nodeId)
              || Object.values(state.contactSpheres).some((sphere) => sphere.bodyId === bodyId && sphere.nodeId === nodeId);
          if (inUse) return state;

          const nodes = { ...body.nodes };
          delete nodes[nodeId];
          return patchBody(state, bodyId, {
            ...body,
            nodes,
            nodeOrder: body.nodeOrder.filter((n) => n !== nodeId),
          });
        }),

      renameNode: (bodyId, nodeId, name) =>
        set((state) => {
          const body = state.bodies[bodyId];
          const node = body?.nodes[nodeId];
          if (!body || !node) return state;
          return patchBody(state, bodyId, { ...body, nodes: { ...body.nodes, [nodeId]: { ...node, name } } });
        }),

      setNodePosition: (bodyId, nodeId, position) =>
        set((state) => {
          const body = state.bodies[bodyId];
          const node = body?.nodes[nodeId];
          if (!body || !node) return state;
          // The origin node defines the body frame; it is fixed at zero by construction.
          if (nodeId === body.originNodeId) return state;
          return patchBody(state, bodyId, { ...body, nodes: { ...body.nodes, [nodeId]: { ...node, position } } });
        }),

      setNodeOrientation: (bodyId, nodeId, orientation) =>
        set((state) => {
          const body = state.bodies[bodyId];
          const node = body?.nodes[nodeId];
          if (!body || !node) return state;
          return patchBody(state, bodyId, { ...body, nodes: { ...body.nodes, [nodeId]: { ...node, orientation } } });
        }),

      /**
       * Move the body frame onto a different node.
       *
       * Every other node shifts by the same offset, so nothing moves in space. If the
       * inertia was stated about the origin it is re-expressed about the new one for the
       * same reason: the physical body has not changed, so its inertia should not either.
       */
      setOriginNode: (bodyId, nodeId) =>
        set((state) => {
          const body = state.bodies[bodyId];
          const target = body?.nodes[nodeId];
          if (!body || !target || body.originNodeId === nodeId) return state;

          const offset = target.position;
          const nodes: Record<string, Node> = {};
          for (const [key, node] of Object.entries(body.nodes)) {
            nodes[key] = { ...node, position: subtract(node.position, offset) };
          }

          let inertia = body.inertia;
          if (body.inertia.about === 'origin') {
            const comNow = body.nodes[body.comNodeId]?.position ?? [0, 0, 0];
            const comNext = nodes[body.comNodeId]?.position ?? [0, 0, 0];
            inertia = {
              about: 'origin',
              ...moveInertiaReference(
                body.mass,
                body.inertia,
                v3(comNow[0], comNow[1], comNow[2]),
                v3(comNext[0], comNext[1], comNext[2]),
              ),
            };
          }

          return patchBody(state, bodyId, { ...body, nodes, originNodeId: nodeId, inertia });
        }),

      setComNode: (bodyId, nodeId) =>
        set((state) => {
          const body = state.bodies[bodyId];
          if (!body || !body.nodes[nodeId]) return state;
          return patchBody(state, bodyId, { ...body, comNodeId: nodeId });
        }),

      // --- hinges ---------------------------------------------------------------------

      selectHinge: (id) => set({ selectedHingeId: id }),

      renameHinge: (id, name) =>
        set((state) => {
          const hinge = state.hinges[id];
          return hinge ? { hinges: { ...state.hinges, [id]: { ...hinge, name } } } : state;
        }),

      /**
       * Re-parent a body.
       *
       * Refuses anything that would close a loop. The picker disables these options too;
       * this is the backstop, because a cyclic hinge set is not something the solver can be
       * asked to interpret.
       */
      setHingeParent: (id, parentBodyId, parentNodeId) =>
        set((state) => {
          const hinge = state.hinges[id];
          const parent = state.bodies[parentBodyId];
          if (!hinge || !parent) return state;
          if (wouldCreateCycle(state.hinges, hinge.childBodyId, parentBodyId)) return state;
          const node = parentNodeId && parent.nodes[parentNodeId] ? parentNodeId : parent.originNodeId;
          return { hinges: { ...state.hinges, [id]: { ...hinge, parentBodyId, parentNodeId: node } } };
        }),

      setHingeParentNode: (id, nodeId) =>
        set((state) => {
          const hinge = state.hinges[id];
          if (!hinge || !state.bodies[hinge.parentBodyId]?.nodes[nodeId]) return state;
          return { hinges: { ...state.hinges, [id]: { ...hinge, parentNodeId: nodeId } } };
        }),

      setHingeChildNode: (id, nodeId) =>
        set((state) => {
          const hinge = state.hinges[id];
          if (!hinge || !state.bodies[hinge.childBodyId]?.nodes[nodeId]) return state;
          return { hinges: { ...state.hinges, [id]: { ...hinge, childNodeId: nodeId } } };
        }),

      setHingeMount: (id, mount) =>
        set((state) => {
          const hinge = state.hinges[id];
          return hinge ? { hinges: { ...state.hinges, [id]: { ...hinge, mount } } } : state;
        }),

      setDof: (hingeId, axis, patch) =>
        set((state) => {
          const hinge = state.hinges[hingeId];
          const dof = hinge?.dof[axis];
          if (!hinge || !dof) return state;
          const next = [...hinge.dof];
          next[axis] = { ...dof, ...patch };
          return { hinges: { ...state.hinges, [hingeId]: { ...hinge, dof: next } } };
        }),

      setDofLimit: (hingeId, axis, patch) =>
        set((state) => {
          const hinge = state.hinges[hingeId];
          const dof = hinge?.dof[axis];
          if (!hinge || !dof) return state;
          const next = [...hinge.dof];
          next[axis] = { ...dof, limit: { ...dof.limit, ...patch } };
          return { hinges: { ...state.hinges, [hingeId]: { ...hinge, dof: next } } };
        }),

      resetHingeDof: (hingeId) =>
        set((state) => {
          const hinge = state.hinges[hingeId];
          if (!hinge) return state;
          return { hinges: { ...state.hinges, [hingeId]: { ...hinge, dof: neutralDofSet() } } };
        }),

      // --- actuators ------------------------------------------------------------------

      selectActuator: (id) => set({ selectedActuatorId: id }),

      addActuator: (bodyId) => {
        const state = get();
        const target =
          bodyId && state.bodies[bodyId] && !state.bodies[bodyId]!.isGround
            ? bodyId
            : state.bodyOrder.find((b) => !state.bodies[b]?.isGround);
        if (!target) return null;

        const id = nextId('actuator');
        set((current) => {
          const body = current.bodies[target]!;
          const actuator: Actuator = {
            id,
            name: uniqueName(Object.values(current.actuators), `Actuator ${current.actuatorOrder.length + 1}`),
            kind: 'force',
            bodyId: target,
            nodeId: body.nodeOrder[body.nodeOrder.length - 1] ?? body.originNodeId,
            frame: 'body',
            vector: [0, 0, 1],
            profile: { kind: 'constant' },
            enabled: true,
            color: pickColor(Object.values(current.actuators), ACTUATOR_COLORS),
          };
          return {
            actuators: { ...current.actuators, [id]: actuator },
            actuatorOrder: [...current.actuatorOrder, id],
            selectedActuatorId: id,
          };
        });
        return id;
      },

      removeActuator: (id) =>
        set((state) => {
          if (!state.actuators[id]) return state;
          const actuators = { ...state.actuators };
          delete actuators[id];
          const actuatorOrder = state.actuatorOrder.filter((a) => a !== id);
          return {
            actuators,
            actuatorOrder,
            selectedActuatorId:
              state.selectedActuatorId === id
                ? (actuatorOrder[actuatorOrder.length - 1] ?? null)
                : state.selectedActuatorId,
          };
        }),

      renameActuator: (id, name) =>
        set((state) => {
          const actuator = state.actuators[id];
          return actuator ? { actuators: { ...state.actuators, [id]: { ...actuator, name } } } : state;
        }),

      setActuatorTarget: (id, bodyId, nodeId) =>
        set((state) => {
          const actuator = state.actuators[id];
          const body = state.bodies[bodyId];
          if (!actuator || !body) return state;
          const node = nodeId && body.nodes[nodeId] ? nodeId : body.originNodeId;
          return { actuators: { ...state.actuators, [id]: { ...actuator, bodyId, nodeId: node } } };
        }),

      setActuatorNode: (id, nodeId) =>
        set((state) => {
          const actuator = state.actuators[id];
          if (!actuator || !state.bodies[actuator.bodyId]?.nodes[nodeId]) return state;
          return { actuators: { ...state.actuators, [id]: { ...actuator, nodeId } } };
        }),

      setActuatorKind: (id, kind) =>
        set((state) => {
          const actuator = state.actuators[id];
          return actuator ? { actuators: { ...state.actuators, [id]: { ...actuator, kind } } } : state;
        }),

      setActuatorFrame: (id, frame) =>
        set((state) => {
          const actuator = state.actuators[id];
          return actuator ? { actuators: { ...state.actuators, [id]: { ...actuator, frame } } } : state;
        }),

      setActuatorVector: (id, vector) =>
        set((state) => {
          const actuator = state.actuators[id];
          return actuator ? { actuators: { ...state.actuators, [id]: { ...actuator, vector } } } : state;
        }),

      setActuatorProfile: (id, profile) =>
        set((state) => {
          const actuator = state.actuators[id];
          return actuator ? { actuators: { ...state.actuators, [id]: { ...actuator, profile } } } : state;
        }),

      toggleActuator: (id) =>
        set((state) => {
          const actuator = state.actuators[id];
          return actuator
            ? { actuators: { ...state.actuators, [id]: { ...actuator, enabled: !actuator.enabled } } }
            : state;
        }),

      // --- contact geometry -----------------------------------------------------------

      addContactSphere: (bodyId) => {
        const state = get();
        const target =
          bodyId && state.bodies[bodyId] && !state.bodies[bodyId]!.isGround
            ? bodyId
            : state.bodyOrder.find((id) => !state.bodies[id]?.isGround);
        if (!target) return null;
        const id = nextId('contact-sphere');
        set((current) => {
          const body = current.bodies[target]!;
          const sphere: ContactSphere = {
            id,
            name: uniqueName(Object.values(current.contactSpheres), `Sphere ${current.contactSphereOrder.length + 1}`),
            bodyId: target,
            nodeId: body.nodeOrder[body.nodeOrder.length - 1] ?? body.originNodeId,
            radius: 0.1,
            material: { stiffness: 10000, damping: 100, friction: 0, frictionVelocity: 0.01 },
            enabled: true,
          };
          return {
            contactSpheres: { ...current.contactSpheres, [id]: sphere },
            contactSphereOrder: [...current.contactSphereOrder, id],
          };
        });
        return id;
      },

      removeContactSphere: (id) =>
        set((state) => {
          if (!state.contactSpheres[id]) return state;
          const contactSpheres = { ...state.contactSpheres };
          delete contactSpheres[id];
          return { contactSpheres, contactSphereOrder: state.contactSphereOrder.filter((entry) => entry !== id) };
        }),

      setContactSphere: (id, patch) =>
        set((state) => {
          const sphere = state.contactSpheres[id];
          if (!sphere) return state;
          let next = { ...sphere, ...patch };
          if (patch.bodyId) {
            const body = state.bodies[patch.bodyId];
            if (!body || body.isGround) return state;
            if (!body.nodes[next.nodeId]) next = { ...next, nodeId: body.originNodeId };
          }
          if (!state.bodies[next.bodyId]?.nodes[next.nodeId]) return state;
          return { contactSpheres: { ...state.contactSpheres, [id]: next } };
        }),

      addContactPlane: () => {
        const id = nextId('contact-plane');
        set((state) => {
          const plane: ContactPlane = {
            id,
            name: uniqueName(Object.values(state.contactPlanes), `Plane ${state.contactPlaneOrder.length + 1}`),
            point: [0, 0, -2],
            normal: [0, 0, 1],
            size: 4,
            material: { stiffness: 10000, damping: 100, friction: 0, frictionVelocity: 0.01 },
            enabled: true,
          };
          return {
            contactPlanes: { ...state.contactPlanes, [id]: plane },
            contactPlaneOrder: [...state.contactPlaneOrder, id],
          };
        });
        return id;
      },

      removeContactPlane: (id) =>
        set((state) => {
          if (!state.contactPlanes[id]) return state;
          const contactPlanes = { ...state.contactPlanes };
          delete contactPlanes[id];
          return { contactPlanes, contactPlaneOrder: state.contactPlaneOrder.filter((entry) => entry !== id) };
        }),

      setContactPlane: (id, patch) =>
        set((state) => {
          const plane = state.contactPlanes[id];
          return plane ? { contactPlanes: { ...state.contactPlanes, [id]: { ...plane, ...patch } } } : state;
        }),

      setContactMaterial: (kind, id, patch) =>
        set((state) => {
          if (kind === 'sphere') {
            const sphere = state.contactSpheres[id];
            return sphere
              ? { contactSpheres: { ...state.contactSpheres, [id]: { ...sphere, material: { ...sphere.material, ...patch } } } }
              : state;
          }
          const plane = state.contactPlanes[id];
          return plane
            ? { contactPlanes: { ...state.contactPlanes, [id]: { ...plane, material: { ...plane.material, ...patch } } } }
            : state;
        }),

      // --- settings -------------------------------------------------------------------

      setSettings: (patch) => set((state) => ({ settings: { ...state.settings, ...patch } })),

      /**
       * Switch unit system.
       *
       * Changes labels and which plausibility checks run — never a stored number. The one
       * exception is gravity, and only when it is still sitting at the previous system's
       * standard value, i.e. the user never touched it. A customised gravity survives the
       * switch untouched.
       */
      setUnits: (units) =>
        set((state) => {
          if (units === state.settings.units) return state;
          const gravity = gravityOnSystemChange(state.settings.gravity, state.settings.units, units);
          return {
            settings: { ...state.settings, units, ...(gravity ? { gravity } : {}) },
          };
        }),

      setConventions: (patch) => set((state) => ({ conventions: { ...state.conventions, ...patch } })),

      applyFix: (fix) =>
        set((state) => {
          switch (fix.kind) {
            case 'setGravity':
              return { settings: { ...state.settings, gravity: fix.value } };
            case 'setUnits': {
              // Deliberately bypasses the gravity preset: this fix exists precisely because
              // the current gravity is the value the user wants to keep.
              return { settings: { ...state.settings, units: fix.value } };
            }
            case 'setTimestep':
              return { settings: { ...state.settings, dt: fix.value } };
          }
        }),

      resetModel: () => set({ ...initialModel(), conventions: get().conventions }),

      loadModel: (model) => set({ ...model }),
    }),
    {
      name: 'toy-dynamics/model',
      version: 1,
      partialize: modelSnapshot,
      /**
       * Falls back to an in-memory map where there is no DOM — under vitest, and in the
       * worker. Without it the middleware logs a warning on every single write, which
       * buries real output in the test run.
       */
      storage: createJSONStorage(() =>
        typeof localStorage === 'undefined' ? memoryStorage() : localStorage,
      ),
      /**
       * Rehydration is the one place untrusted data enters the store, so the payload is
       * repaired rather than trusted. A corrupt save degrades to a working model instead of
       * a blank screen.
       */
      merge: (persisted, current) => {
        const repaired = repairModel(persisted);
        return repaired ? { ...current, ...repaired } : current;
      },
    },
  ),
);
