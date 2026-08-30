import { useMemo } from 'react';
import * as THREE from 'three';
import type { Body, Vec3 } from '../types';
import type { BodyPose } from '../sim/kinematics';
import { AXIS_COLORS, COM_COLOR } from '../theme';
import { Arrow } from './Arrow';
import { Label } from './Label';

/**
 * One rigid body: its nodes, the struts between them, its frame axes and its centre of mass.
 *
 * A body has no geometry of its own in this model — it *is* a set of nodes and a set of
 * mass properties. So it is drawn as exactly that, and the struts from the origin out to
 * each node are what make the collection read as one object rather than as loose points.
 * Nothing here implies a shape the model does not actually have.
 */

const AXES: { name: 'X' | 'Y' | 'Z'; direction: Vec3 }[] = [
  { name: 'X', direction: [1, 0, 0] },
  { name: 'Y', direction: [0, 1, 0] },
  { name: 'Z', direction: [0, 0, 1] },
];

/**
 * A node's label, qualified by its role only when the name does not already say it.
 *
 * Without the check a node the user sensibly called "CoM" renders as "CoM · CoM".
 */
function labelFor(name: string, isOrigin: boolean, isCom: boolean): string {
  const role = isCom ? 'CoM' : isOrigin ? 'origin' : null;
  if (!role) return name;
  return name.toLowerCase() === role.toLowerCase() ? name : `${name} · ${role}`;
}

type Props = {
  body: Body;
  pose: BodyPose;
  selected: boolean;
  /** Scene-wide size hint, so markers stay proportionate on a large or tiny model. */
  scale: number;
  onSelect: () => void;
};

export function BodyView({ body, pose, selected, scale, onSelect }: Props) {
  const position = useMemo(() => new THREE.Vector3(...pose.position), [pose.position]);
  const quaternion = useMemo(() => new THREE.Quaternion(...pose.quaternion), [pose.quaternion]);

  const nodeRadius = 0.035 * scale;
  const axisLength = 0.32 * scale;
  const originNode = body.nodes[body.originNodeId];

  const struts = useMemo(() => {
    if (!originNode) return [];
    return body.nodeOrder
      .filter((id) => id !== body.originNodeId)
      .map((id) => body.nodes[id])
      .filter((node): node is NonNullable<typeof node> => Boolean(node))
      .map((node) => {
        const start = new THREE.Vector3(...originNode.position);
        const end = new THREE.Vector3(...node.position);
        const delta = end.clone().sub(start);
        const length = delta.length();
        return {
          id: node.id,
          length,
          midpoint: start.clone().add(end).multiplyScalar(0.5),
          // Cylinders are authored along +Y, so each strut is rotated onto its own axis.
          quaternion:
            length < 1e-9
              ? new THREE.Quaternion()
              : new THREE.Quaternion().setFromUnitVectors(
                  new THREE.Vector3(0, 1, 0),
                  delta.clone().normalize(),
                ),
        };
      })
      .filter((strut) => strut.length > 1e-9);
  }, [body.nodeOrder, body.nodes, body.originNodeId, originNode]);

  return (
    <group position={position} quaternion={quaternion}>
      {/* Frame axes at the body origin. Short, so they orient without dominating. */}
      {AXES.map((axis) => (
        <Arrow
          key={axis.name}
          direction={new THREE.Vector3(...axis.direction)}
          length={axisLength}
          color={AXIS_COLORS[axis.name]}
          opacity={selected ? 1 : 0.55}
          radiusScale={selected ? 1.5 : 1}
        />
      ))}

      {struts.map((strut) => (
        <mesh key={strut.id} position={strut.midpoint} quaternion={strut.quaternion}>
          <cylinderGeometry args={[nodeRadius * 0.35, nodeRadius * 0.35, strut.length, 8]} />
          <meshStandardMaterial
            color={body.color}
            transparent
            opacity={selected ? 0.9 : 0.5}
            roughness={0.5}
          />
        </mesh>
      ))}

      {body.nodeOrder.map((id) => {
        const node = body.nodes[id];
        if (!node) return null;
        const isCom = id === body.comNodeId;
        const isOrigin = id === body.originNodeId;
        return (
          <group key={id} position={node.position}>
            <mesh
              onClick={(event) => {
                event.stopPropagation();
                onSelect();
              }}
            >
              <sphereGeometry args={[isCom ? nodeRadius * 1.5 : nodeRadius, 16, 12]} />
              <meshStandardMaterial
                color={isCom ? COM_COLOR : body.color}
                emissive={isCom ? COM_COLOR : body.color}
                emissiveIntensity={selected ? 0.5 : 0.2}
                roughness={0.4}
              />
            </mesh>
            {/* A ring around the centre of mass, so it is unmistakable at a glance even
                when it coincides with the frame origin. */}
            {isCom && (
              <mesh rotation={[Math.PI / 2, 0, 0]}>
                <torusGeometry args={[nodeRadius * 2.4, nodeRadius * 0.22, 8, 24]} />
                <meshStandardMaterial color={COM_COLOR} transparent opacity={0.85} />
              </mesh>
            )}
            {selected && (
              <Label
                text={labelFor(node.name, isOrigin, isCom)}
                color={isCom ? COM_COLOR : body.color}
                position={[0, 0, nodeRadius * 3.4]}
                scale={0.055 * scale}
              />
            )}
          </group>
        );
      })}

      {selected && (
        <Label text={body.name} color={body.color} position={[0, 0, axisLength * 1.25]} scale={0.075 * scale} />
      )}
    </group>
  );
}
