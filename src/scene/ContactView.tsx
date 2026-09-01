import { useMemo } from 'react';
import * as THREE from 'three';
import type { Body, ContactPlane, ContactSphere } from '../types';
import { nodeWorldPosition, type BodyPose } from '../sim/kinematics';

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

export function ContactPlaneView({ plane }: { plane: ContactPlane }) {
  const quaternion = useMemo(() => {
    const normal = new THREE.Vector3(...plane.normal);
    if (normal.lengthSq() < 1e-18) return new THREE.Quaternion();
    return new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal.normalize());
  }, [plane.normal]);
  const size = Math.max(0.01, plane.size);
  return <group position={plane.point} quaternion={quaternion} visible={plane.enabled}>
    <mesh>
      <planeGeometry args={[size, size]} />
      <meshBasicMaterial color={CONTACT_COLOR} transparent opacity={0.12} side={THREE.DoubleSide} depthWrite={false} />
    </mesh>
    <lineSegments>
      <edgesGeometry args={[new THREE.PlaneGeometry(size, size)]} />
      <lineBasicMaterial color={CONTACT_COLOR} transparent opacity={0.7} />
    </lineSegments>
  </group>;
}
