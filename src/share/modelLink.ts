import type { ModelPersisted } from '../store/modelRepair';
import { repairModel } from '../store/modelRepair';

/**
 * Encoding a model into a URL hash, so a setup moves between phone and desktop.
 *
 * The wire format is deliberately terse — records as positional arrays, ids remapped to
 * indices, floats rounded — because a link that wraps across three lines in a chat app is a
 * link people stop using.
 *
 * Decoding leans on `repairModel` rather than trusting the payload. A URL is even less
 * trustworthy than localStorage: it can be truncated by a chat client, hand-edited, or
 * produced by an older version of this app. Anything malformed degrades to a model that
 * renders instead of a blank screen.
 */

/** Bumped only for changes the decoder cannot absorb; unknown versions are rejected. */
const FORMAT_VERSION = 1;

export const MODEL_HASH_PREFIX = '#m=';

/** Six decimals is well past what any readout shows, and keeps the link short. */
const round = (n: number): number => (Number.isFinite(n) ? Math.round(n * 1e6) / 1e6 : 0);
const roundAll = (v: readonly number[]): number[] => v.map(round);

type EncodedNode = [name: string, x: number, y: number, z: number, qx: number, qy: number, qz: number, qw: number];

type EncodedBody = [
  name: string,
  nodes: EncodedNode[],
  originIndex: number,
  comIndex: number,
  mass: number,
  inertia: [about: 0 | 1, ixx: number, iyy: number, izz: number, ixy: number, ixz: number, iyz: number],
  color: string,
  visible: 0 | 1,
];

/**
 * `[free, q0, u0, k, rest, c, friction, limitOn, lo, hi, limitK, stiction]`.
 *
 * New fields are appended, never inserted. A link written before `stiction` existed simply
 * has no twelfth entry, which decodes to 0 — disabled, exactly the old behaviour — so older
 * links keep working without a format-version bump.
 */
type EncodedDof = [0 | 1, number, number, number, number, number, number, 0 | 1, number, number, number, number];

type EncodedHinge = [
  name: string,
  parentIndex: number,
  parentNodeIndex: number,
  childIndex: number,
  childNodeIndex: number,
  mount: [number, number, number, number],
  dof: EncodedDof[],
];

type EncodedActuator = [
  name: string,
  kind: 0 | 1,
  bodyIndex: number,
  nodeIndex: number,
  frame: 0 | 1,
  vector: [number, number, number],
  profile: unknown[],
  enabled: 0 | 1,
  color: string,
];

type EncodedModel = {
  v: number;
  b: EncodedBody[];
  h: EncodedHinge[];
  a: EncodedActuator[];
  /** `[name, body, node, radius, stiffness, damping, enabled]`. */
  cs?: [string, number, number, number, number, number, 0 | 1][];
  /** `[name, point, normal, stiffness, damping, enabled, displaySize?]`. */
  cp?: [string, number[], number[], number, number, 0 | 1, number?][];
  /** `[units, gx, gy, gz, dt, duration, integrator, sampleRate]`. */
  s: [string, number, number, number, number, number, string, number];
  /** `[upAxis, eulerOrder, rotationMode, angleUnit]`. */
  c: [string, string, string, string];
};

// --- base64url over UTF-8, so names in any language survive the trip -----------------

function toBase64Url(text: string): string {
  const bytes = new TextEncoder().encode(text);
  let binary = '';
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return btoa(binary).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}

function fromBase64Url(encoded: string): string {
  const padded = encoded.replace(/-/g, '+').replace(/_/g, '/');
  const binary = atob(padded + '='.repeat((4 - (padded.length % 4)) % 4));
  return new TextDecoder().decode(Uint8Array.from(binary, (ch) => ch.charCodeAt(0)));
}

// --- encode ---------------------------------------------------------------------------

export function encodeModel(model: ModelPersisted): string {
  const bodyIds = model.bodyOrder.filter((id) => model.bodies[id]);
  const bodyIndex = new Map(bodyIds.map((id, i) => [id, i]));
  /** Body id → (node id → index), so hinge and actuator references become small integers. */
  const nodeIndex = new Map<string, Map<string, number>>();

  const bodies: EncodedBody[] = bodyIds.map((id) => {
    const body = model.bodies[id]!;
    const order = body.nodeOrder.filter((n) => body.nodes[n]);
    nodeIndex.set(id, new Map(order.map((n, i) => [n, i])));

    return [
      body.name,
      order.map((nodeId): EncodedNode => {
        const node = body.nodes[nodeId]!;
        const [x, y, z] = roundAll(node.position);
        const [qx, qy, qz, qw] = roundAll(node.orientation);
        return [node.name, x!, y!, z!, qx!, qy!, qz!, qw!];
      }),
      order.indexOf(body.originNodeId),
      order.indexOf(body.comNodeId),
      round(body.mass),
      [
        body.inertia.about === 'origin' ? 1 : 0,
        round(body.inertia.ixx),
        round(body.inertia.iyy),
        round(body.inertia.izz),
        round(body.inertia.ixy),
        round(body.inertia.ixz),
        round(body.inertia.iyz),
      ],
      body.color,
      body.visible ? 1 : 0,
    ];
  });

  const hinges: EncodedHinge[] = model.hingeOrder
    .map((id) => model.hinges[id])
    .filter((hinge): hinge is NonNullable<typeof hinge> => Boolean(hinge))
    .map((hinge) => {
      const [qx, qy, qz, qw] = roundAll(hinge.mount);
      return [
        hinge.name,
        bodyIndex.get(hinge.parentBodyId) ?? 0,
        nodeIndex.get(hinge.parentBodyId)?.get(hinge.parentNodeId) ?? 0,
        bodyIndex.get(hinge.childBodyId) ?? 0,
        nodeIndex.get(hinge.childBodyId)?.get(hinge.childNodeId) ?? 0,
        [qx!, qy!, qz!, qw!],
        hinge.dof.map((d): EncodedDof => [
          d.free ? 1 : 0,
          round(d.q0),
          round(d.u0),
          round(d.stiffness),
          round(d.rest),
          round(d.damping),
          round(d.friction),
          d.limit.enabled ? 1 : 0,
          round(d.limit.lo),
          round(d.limit.hi),
          round(d.limit.stiffness),
          round(d.stiction),
        ]),
      ];
    });

  const actuators: EncodedActuator[] = model.actuatorOrder
    .map((id) => model.actuators[id])
    .filter((a): a is NonNullable<typeof a> => Boolean(a))
    .map((actuator) => {
      const [vx, vy, vz] = roundAll(actuator.vector);
      return [
        actuator.name,
        actuator.kind === 'moment' ? 1 : 0,
        bodyIndex.get(actuator.bodyId) ?? 0,
        nodeIndex.get(actuator.bodyId)?.get(actuator.nodeId) ?? 0,
        actuator.frame === 'world' ? 1 : 0,
        [vx!, vy!, vz!],
        encodeProfile(actuator.profile),
        actuator.enabled ? 1 : 0,
        actuator.color,
      ];
    });

  const payload: EncodedModel = {
    v: FORMAT_VERSION,
    b: bodies,
    h: hinges,
    a: actuators,
    cs: model.contactSphereOrder.flatMap((id) => {
      const sphere = model.contactSpheres[id];
      if (!sphere) return [];
      return [[sphere.name, bodyIndex.get(sphere.bodyId) ?? 0,
        nodeIndex.get(sphere.bodyId)?.get(sphere.nodeId) ?? 0, round(sphere.radius),
        round(sphere.material.stiffness), round(sphere.material.damping), sphere.enabled ? 1 : 0]];
    }),
    cp: model.contactPlaneOrder.flatMap((id) => {
      const plane = model.contactPlanes[id];
      if (!plane) return [];
      return [[plane.name, roundAll(plane.point), roundAll(plane.normal),
        round(plane.material.stiffness), round(plane.material.damping), plane.enabled ? 1 : 0,
        round(plane.size)]];
    }),
    s: [
      model.settings.units,
      ...(roundAll(model.settings.gravity) as [number, number, number]),
      round(model.settings.dt),
      round(model.settings.duration),
      model.settings.integrator,
      round(model.settings.sampleRate),
    ],
    c: [
      model.conventions.upAxis,
      model.conventions.eulerOrder,
      model.conventions.rotationMode,
      model.conventions.angleUnit,
    ],
  };

  return toBase64Url(JSON.stringify(payload));
}

// --- decode ---------------------------------------------------------------------------

const PROFILE_KINDS = ['constant', 'step', 'ramp', 'sine', 'impulse', 'expr'] as const;

function encodeProfile(profile: ModelPersisted['actuators'][string]['profile']): unknown[] {
  const kind = PROFILE_KINDS.indexOf(profile.kind);
  switch (profile.kind) {
    case 'constant':
      return [kind];
    case 'step':
      return [kind, round(profile.tOn), round(profile.tOff)];
    case 'ramp':
      return [kind, round(profile.t0), round(profile.t1), round(profile.from), round(profile.to)];
    case 'sine':
      return [kind, round(profile.frequency), round(profile.phase), round(profile.offset)];
    case 'impulse':
      return [kind, round(profile.t0), round(profile.width)];
    case 'expr':
      return [kind, profile.source];
  }
}

function decodeProfile(encoded: unknown): unknown {
  if (!Array.isArray(encoded)) return { kind: 'constant' };
  const kind = PROFILE_KINDS[Number(encoded[0])] ?? 'constant';
  switch (kind) {
    case 'step':
      return { kind, tOn: encoded[1], tOff: encoded[2] };
    case 'ramp':
      return { kind, t0: encoded[1], t1: encoded[2], from: encoded[3], to: encoded[4] };
    case 'sine':
      return { kind, frequency: encoded[1], phase: encoded[2], offset: encoded[3] };
    case 'impulse':
      return { kind, t0: encoded[1], width: encoded[2] };
    case 'expr':
      return { kind, source: encoded[1] };
    default:
      return { kind: 'constant' };
  }
}

/**
 * Decode a hash payload back into a model, or null if it is unusable.
 *
 * The decoded shape is deliberately handed to `repairModel` rather than returned directly:
 * everything downstream then gets the same guarantees it gets from localStorage, and there
 * is one place that knows what a valid model looks like.
 */
export function decodeModel(encoded: string): ModelPersisted | null {
  let payload: EncodedModel;
  try {
    payload = JSON.parse(fromBase64Url(encoded)) as EncodedModel;
  } catch {
    return null;
  }
  if (!payload || payload.v !== FORMAT_VERSION || !Array.isArray(payload.b)) return null;

  const bodies: Record<string, unknown> = {};
  const bodyOrder: string[] = [];
  /** Index → id, mirroring the encoder's remapping. */
  const bodyIds: string[] = [];
  const nodeIds: string[][] = [];

  payload.b.forEach((body, i) => {
    const id = i === 0 ? 'ground' : `b${i}`;
    bodyIds.push(id);
    bodyOrder.push(id);

    const nodes: Record<string, unknown> = {};
    const ids: string[] = [];
    const rawNodes = Array.isArray(body?.[1]) ? body[1] : [];
    rawNodes.forEach((node, j) => {
      const nodeId = `${id}n${j}`;
      ids.push(nodeId);
      nodes[nodeId] = {
        name: node?.[0],
        position: [node?.[1], node?.[2], node?.[3]],
        orientation: [node?.[4], node?.[5], node?.[6], node?.[7]],
      };
    });
    nodeIds.push(ids);

    const inertia = Array.isArray(body?.[5]) ? body[5] : [];
    bodies[id] = {
      name: body?.[0],
      nodes,
      nodeOrder: ids,
      originNodeId: ids[Number(body?.[2])] ?? ids[0],
      comNodeId: ids[Number(body?.[3])] ?? ids[0],
      mass: body?.[4],
      inertia: {
        about: inertia[0] === 1 ? 'origin' : 'com',
        ixx: inertia[1],
        iyy: inertia[2],
        izz: inertia[3],
        ixy: inertia[4],
        ixz: inertia[5],
        iyz: inertia[6],
      },
      color: body?.[6],
      visible: body?.[7] !== 0,
      ...(i === 0 ? { isGround: true } : {}),
    };
  });

  const hinges: Record<string, unknown> = {};
  const hingeOrder: string[] = [];
  (Array.isArray(payload.h) ? payload.h : []).forEach((hinge, i) => {
    const id = `h${i}`;
    hingeOrder.push(id);
    const parent = bodyIds[Number(hinge?.[1])] ?? bodyIds[0];
    const child = bodyIds[Number(hinge?.[3])] ?? bodyIds[0];
    hinges[id] = {
      name: hinge?.[0],
      parentBodyId: parent,
      parentNodeId: nodeIds[Number(hinge?.[1])]?.[Number(hinge?.[2])],
      childBodyId: child,
      childNodeId: nodeIds[Number(hinge?.[3])]?.[Number(hinge?.[4])],
      mount: hinge?.[5],
      dof: (Array.isArray(hinge?.[6]) ? hinge[6] : []).map((d) => ({
        free: d?.[0] === 1,
        q0: d?.[1],
        u0: d?.[2],
        stiffness: d?.[3],
        rest: d?.[4],
        damping: d?.[5],
        friction: d?.[6],
        limit: { enabled: d?.[7] === 1, lo: d?.[8], hi: d?.[9], stiffness: d?.[10] },
        stiction: d?.[11] ?? 0,
      })),
    };
  });

  const actuators: Record<string, unknown> = {};
  const actuatorOrder: string[] = [];
  (Array.isArray(payload.a) ? payload.a : []).forEach((actuator, i) => {
    const id = `a${i}`;
    actuatorOrder.push(id);
    actuators[id] = {
      name: actuator?.[0],
      kind: actuator?.[1] === 1 ? 'moment' : 'force',
      bodyId: bodyIds[Number(actuator?.[2])],
      nodeId: nodeIds[Number(actuator?.[2])]?.[Number(actuator?.[3])],
      frame: actuator?.[4] === 1 ? 'world' : 'body',
      vector: actuator?.[5],
      profile: decodeProfile(actuator?.[6]),
      enabled: actuator?.[7] !== 0,
      color: actuator?.[8],
    };
  });

  const contactSpheres: Record<string, unknown> = {};
  const contactSphereOrder: string[] = [];
  (Array.isArray(payload.cs) ? payload.cs : []).forEach((sphere, i) => {
    const id = `cs${i}`;
    contactSphereOrder.push(id);
    contactSpheres[id] = {
      name: sphere?.[0], bodyId: bodyIds[Number(sphere?.[1])],
      nodeId: nodeIds[Number(sphere?.[1])]?.[Number(sphere?.[2])], radius: sphere?.[3],
      material: { stiffness: sphere?.[4], damping: sphere?.[5] }, enabled: sphere?.[6] !== 0,
    };
  });
  const contactPlanes: Record<string, unknown> = {};
  const contactPlaneOrder: string[] = [];
  (Array.isArray(payload.cp) ? payload.cp : []).forEach((plane, i) => {
    const id = `cp${i}`;
    contactPlaneOrder.push(id);
    contactPlanes[id] = {
      name: plane?.[0], point: plane?.[1], normal: plane?.[2],
      material: { stiffness: plane?.[3], damping: plane?.[4] }, enabled: plane?.[5] !== 0,
      size: plane?.[6],
    };
  });

  const s = Array.isArray(payload.s) ? payload.s : [];
  const c = Array.isArray(payload.c) ? payload.c : [];

  return repairModel({
    bodies,
    bodyOrder,
    hinges,
    hingeOrder,
    actuators,
    actuatorOrder,
    contactSpheres,
    contactSphereOrder,
    contactPlanes,
    contactPlaneOrder,
    settings: {
      units: s[0],
      gravity: [s[1], s[2], s[3]],
      dt: s[4],
      duration: s[5],
      integrator: s[6],
      sampleRate: s[7],
    },
    conventions: { upAxis: c[0], eulerOrder: c[1], rotationMode: c[2], angleUnit: c[3] },
    selectedBodyId: bodyOrder[1] ?? bodyOrder[0],
    selectedHingeId: hingeOrder[0] ?? null,
    selectedActuatorId: actuatorOrder[0] ?? null,
  });
}

/** A shareable absolute URL for a model. */
export function modelLink(model: ModelPersisted): string {
  const { origin, pathname } = window.location;
  return `${origin}${pathname}${MODEL_HASH_PREFIX}${encodeModel(model)}`;
}
