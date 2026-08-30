import { useMemo } from 'react';
import * as THREE from 'three';
import type { Actuator, Body } from '../types';
import { nodeWorldPosition, type BodyPose } from '../sim/kinematics';
import { Arrow } from './Arrow';
import { Label } from './Label';

/**
 * An actuator at its node.
 *
 * A force is a straight arrow; a moment is a ring with a tangential arrowhead, because the
 * two are genuinely different objects and drawing both as arrows is how people end up
 * reading a torque as a push.
 *
 * The arrow follows the body when the actuator is body-fixed and stays put when it is
 * world-fixed — visibly, as the model moves. That difference is the single most common
 * source of surprise in this kind of model, so it is shown rather than only stated.
 */

type Props = {
  actuator: Actuator;
  bodies: Record<string, Body>;
  poses: Map<string, BodyPose>;
  selected: boolean;
  scale: number;
  /** Longest actuator vector in the scene, so lengths are relative rather than absolute. */
  reference: number;
  onSelect: () => void;
};

export function ActuatorView({ actuator, bodies, poses, selected, scale, reference, onSelect }: Props) {
  const placement = useMemo(() => {
    const body = bodies[actuator.bodyId];
    const node = body?.nodes[actuator.nodeId];
    if (!body || !node) return null;

    const pose = poses.get(actuator.bodyId) ?? { position: [0, 0, 0], quaternion: [0, 0, 0, 1] } as BodyPose;
    const origin = new THREE.Vector3(...nodeWorldPosition(pose, node.position));

    const local = new THREE.Vector3(...actuator.vector);
    if (local.lengthSq() < 1e-20) return null;

    // A body-fixed vector is written in the node's axes and has to be carried out through
    // the node and the body; a world-fixed one is already in world axes.
    const direction =
      actuator.frame === 'body'
        ? local
            .clone()
            .applyQuaternion(new THREE.Quaternion(...node.orientation))
            .applyQuaternion(new THREE.Quaternion(...pose.quaternion))
        : local.clone();

    return { origin, direction, magnitude: local.length() };
  }, [actuator.bodyId, actuator.frame, actuator.nodeId, actuator.vector, bodies, poses]);

  if (!placement) return null;

  // Square-root scaling: a linear map makes a strong actuator dwarf everything else, and a
  // log one makes them all look the same. This keeps the ordering visible without the
  // extremes taking over.
  const relative = reference > 0 ? Math.sqrt(placement.magnitude / reference) : 1;
  const length = (0.28 + 0.38 * relative) * scale;
  const dimmed = actuator.enabled ? 1 : 0.25;

  return (
    <group position={placement.origin}>
      {actuator.kind === 'force' ? (
        <group onClick={(event) => { event.stopPropagation(); onSelect(); }}>
          <Arrow
            direction={placement.direction}
            length={length}
            color={actuator.color}
            opacity={dimmed}
            radiusScale={selected ? 2.2 : 1.4}
          />
        </group>
      ) : (
        <MomentGlyph
          direction={placement.direction}
          radius={length * 0.45}
          color={actuator.color}
          opacity={dimmed}
          selected={selected}
          onSelect={onSelect}
        />
      )}

      {selected && (
        <Label
          text={`${actuator.name}${actuator.enabled ? '' : ' (off)'}`}
          color={actuator.color}
          position={[0, 0, length * 0.42]}
          scale={0.06 * scale}
        />
      )}
    </group>
  );
}

/**
 * A moment: three-quarters of a ring about the axis, with an arrowhead on the open end.
 *
 * Open rather than closed so the sense of rotation is readable — a full torus has no
 * direction, and a moment that could be either way round is not much of a readout.
 */
function MomentGlyph({
  direction,
  radius,
  color,
  opacity,
  selected,
  onSelect,
}: {
  direction: THREE.Vector3;
  radius: number;
  color: string;
  opacity: number;
  selected: boolean;
  onSelect: () => void;
}) {
  const quaternion = useMemo(
    () =>
      new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 0, 1),
        direction.clone().normalize(),
      ),
    [direction],
  );

  const tube = radius * (selected ? 0.075 : 0.055);
  const sweep = Math.PI * 1.5;

  return (
    <group quaternion={quaternion} onClick={(event) => { event.stopPropagation(); onSelect(); }}>
      <mesh>
        <torusGeometry args={[radius, tube, 10, 40, sweep]} />
        <meshStandardMaterial color={color} transparent={opacity < 1} opacity={opacity} emissive={color} emissiveIntensity={selected ? 0.4 : 0.15} />
      </mesh>
      {/* Arrowhead tangent to the ring at the open end, pointing the way the moment acts. */}
      <group position={[radius * Math.cos(sweep), radius * Math.sin(sweep), 0]}>
        <mesh rotation={[0, 0, sweep]}>
          <coneGeometry args={[tube * 2.6, tube * 6, 12]} />
          <meshStandardMaterial color={color} transparent={opacity < 1} opacity={opacity} />
        </mesh>
      </group>
      {/* A stub along the axis, so the moment's direction is unambiguous edge-on.
          Cylinders are authored along +Y, hence the quarter turn onto +Z. */}
      <mesh position={[0, 0, radius * 0.5]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[tube * 0.5, tube * 0.5, radius, 8]} />
        <meshStandardMaterial color={color} transparent opacity={opacity * 0.6} />
      </mesh>
    </group>
  );
}
