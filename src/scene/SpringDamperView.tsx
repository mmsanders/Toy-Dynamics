import { useMemo } from 'react';
import * as THREE from 'three';
import { Line } from '@react-three/drei';
import type { Body, SpringDamper } from '../types';
import { nodeWorldPosition, type BodyPose } from '../sim/kinematics';
import { Label } from './Label';

type Props = {
  device: SpringDamper;
  bodies: Record<string, Body>;
  poses: Map<string, BodyPose>;
  selected: boolean;
  scale: number;
  onSelect: () => void;
};

/** A simple, readable world-space line between the device's two live attachment nodes. */
export function SpringDamperView({ device, bodies, poses, selected, scale, onSelect }: Props) {
  const placement = useMemo(() => {
    const endpoint = (bodyId: string, nodeId: string): THREE.Vector3 | null => {
      const body = bodies[bodyId];
      const node = body?.nodes[nodeId];
      if (!body || !node) return null;
      if (body.isGround) return new THREE.Vector3(...node.position);
      const pose = poses.get(bodyId);
      return pose ? new THREE.Vector3(...nodeWorldPosition(pose, node.position)) : null;
    };
    const a = endpoint(device.bodyAId, device.nodeAId);
    const b = endpoint(device.bodyBId, device.nodeBId);
    if (!a || !b) return null;
    return { a, b, midpoint: a.clone().add(b).multiplyScalar(0.5) };
  }, [bodies, device.bodyAId, device.bodyBId, device.nodeAId, device.nodeBId, poses]);

  if (!placement) return null;
  const opacity = device.enabled ? 0.9 : 0.25;

  return (
    <group onClick={(event) => { event.stopPropagation(); onSelect(); }}>
      <Line
        points={[placement.a, placement.b]}
        color={device.color}
        lineWidth={selected ? 3.8 : 2.2}
        dashed
        dashScale={8 / Math.max(scale, 1e-6)}
        gapSize={0.4}
        dashSize={0.7}
        transparent
        opacity={opacity}
      />
      {selected && (
        <Label
          text={`${device.name}${device.enabled ? '' : ' (off)'}`}
          color={device.color}
          position={[placement.midpoint.x, placement.midpoint.y, placement.midpoint.z]}
          scale={0.06 * scale}
        />
      )}
    </group>
  );
}
