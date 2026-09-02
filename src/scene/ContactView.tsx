import { useMemo } from 'react';
import * as THREE from 'three';
import type { Body, ContactHeightfield, ContactPlane, ContactSphere } from '../types';
import { nodeWorldPosition, type BodyPose } from '../sim/kinematics';
import { planeBasis } from '../dyn/contact';
import { v3 } from '../dyn/spatial';
import { Arrow } from './Arrow';

export const CONTACT_COLOR = '#fb7185';

export function ContactSphereView({ sphere, body, pose, scale }: { sphere: ContactSphere; body: Body; pose: BodyPose; scale: number }) {
  const node = body.nodes[sphere.nodeId];
  const position = node ? nodeWorldPosition(pose, node.position) : pose.position;
  const radius = sphere.radius > 0 ? sphere.radius : 0.025 * scale;
  return <mesh position={position} visible={sphere.enabled}>
    <sphereGeometry args={[radius, 20, 14]} />
    <meshBasicMaterial color={CONTACT_COLOR} wireframe transparent opacity={0.8} depthWrite={false} />
  </mesh>;
}

/**
 * A contact plane: its patch, and an arrow along the side it pushes from.
 *
 * The patch is oriented with the solver's own in-plane basis rather than a separately
 * chosen one, so on a bounded plate the square drawn here is exactly the square that stops
 * a sphere. The arrow exists because a plane looks identical from both sides and only one
 * of them is solid — without it, which way the normal points is something you can only
 * discover by running.
 */
export function ContactPlaneView({ plane, scale }: { plane: ContactPlane; scale: number }) {
  const quaternion = useMemo(() => {
    const normal = new THREE.Vector3(...plane.normal);
    if (normal.lengthSq() < 1e-18) return new THREE.Quaternion();
    normal.normalize();
    const u = v3();
    const v = v3();
    planeBasis(v3(normal.x, normal.y, normal.z), u, v);
    // A plane's geometry lies in its local XY with +Z out, so mapping the basis onto those
    // axes puts the drawn square on the solver's footprint.
    const basis = new THREE.Matrix4().makeBasis(
      new THREE.Vector3(u[0], u[1], u[2]),
      new THREE.Vector3(v[0], v[1], v[2]),
      normal,
    );
    return new THREE.Quaternion().setFromRotationMatrix(basis);
  }, [plane.normal]);

  // A bounded plate is drawn at exactly its extent. An unbounded plane has no extent to
  // draw, so `size` is only how much of it to show.
  const size = plane.size > 0 ? plane.size : Math.max(2, scale * 4);
  const geometry = useMemo(() => new THREE.PlaneGeometry(size, size), [size]);
  const edges = useMemo(() => new THREE.EdgesGeometry(geometry), [geometry]);

  return <group position={plane.point} quaternion={quaternion} visible={plane.enabled}>
    <mesh geometry={geometry}>
      <meshBasicMaterial color={CONTACT_COLOR} transparent opacity={0.12} side={THREE.DoubleSide} depthWrite={false} />
    </mesh>
    {/* A real edge is drawn as one; the border of an unbounded plane is only where the
        drawing stops, so it is faint enough not to be read as a rim. */}
    <lineSegments geometry={edges}>
      <lineBasicMaterial color={CONTACT_COLOR} transparent opacity={plane.bounded ? 0.75 : 0.25} />
    </lineSegments>
    <Arrow direction={NORMAL_AXIS} length={Math.max(0.35 * size, 0.1 * scale)} color={CONTACT_COLOR} />
  </group>;
}

/** The group is already turned so its +Z is the plane normal. */
const NORMAL_AXIS = new THREE.Vector3(0, 0, 1);

/** Fixed regular height grid. Cells touching no-data samples are omitted entirely. */
export function ContactHeightfieldView({ field }: { field: ContactHeightfield }) {
  const geometry = useMemo(() => {
    const positions = new Float32Array(field.columns * field.rows * 3);
    for (let row = 0; row < field.rows; row++) {
      for (let column = 0; column < field.columns; column++) {
        const sample = row * field.columns + column;
        const at = sample * 3;
        positions[at] = column * field.spacing;
        positions[at + 1] = row * field.spacing;
        positions[at + 2] = field.heights[sample] ?? 0;
      }
    }
    const indices: number[] = [];
    for (let row = 0; row < field.rows - 1; row++) {
      for (let column = 0; column < field.columns - 1; column++) {
        const a = row * field.columns + column;
        const b = a + 1;
        const c = a + field.columns;
        const d = c + 1;
        if ([a, b, c, d].some((index) => field.heights[index] === null)) continue;
        indices.push(a, b, d, a, d, c);
      }
    }
    const result = new THREE.BufferGeometry();
    result.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    result.setIndex(indices);
    result.computeVertexNormals();
    return result;
  }, [field.columns, field.rows, field.spacing, field.heights]);
  const edges = useMemo(() => new THREE.EdgesGeometry(geometry, 12), [geometry]);

  return (
    <group position={field.origin} visible={field.enabled}>
      <mesh geometry={geometry}>
        <meshStandardMaterial color={CONTACT_COLOR} transparent opacity={0.2} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      <lineSegments geometry={edges}>
        <lineBasicMaterial color={CONTACT_COLOR} transparent opacity={0.55} />
      </lineSegments>
    </group>
  );
}
