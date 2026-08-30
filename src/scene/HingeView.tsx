import { useMemo } from 'react';
import * as THREE from 'three';
import type { Body, Hinge } from '../types';
import type { BodyPose } from '../sim/kinematics';
import { nodeWorldPosition } from '../sim/kinematics';
import { AXIS_COLORS, HINGE_COLOR } from '../theme';
import { Label } from './Label';

/**
 * A hinge, drawn as its free axes.
 *
 * The glyphs are the point of this: a hinge's identity in this model is *which* of its six
 * axes are free, and that is far easier to read as a shape in space than as six checkboxes
 * in a panel. A free translation is a double-headed arrow along its axis; a free rotation is
 * a ring around it. A fully locked hinge is a plain cube — a weld, visibly.
 *
 * Axis colours match the readouts, so an axis means the same thing everywhere in the app.
 */

const AXIS_VECTORS = [
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(0, 0, 1),
];

const AXIS_NAMES = ['X', 'Y', 'Z'] as const;
const UP = new THREE.Vector3(0, 1, 0);

type Props = {
  hinge: Hinge;
  bodies: Record<string, Body>;
  poses: Map<string, BodyPose>;
  selected: boolean;
  scale: number;
  onSelect: () => void;
};

export function HingeView({ hinge, bodies, poses, selected, scale, onSelect }: Props) {
  const placement = useMemo(() => {
    const parent = bodies[hinge.parentBodyId];
    const node = parent?.nodes[hinge.parentNodeId];
    if (!parent || !node) return null;

    // Ground is not a link and has no computed pose, so it stands at the identity.
    const pose: BodyPose = poses.get(hinge.parentBodyId) ?? {
      position: [0, 0, 0],
      quaternion: [0, 0, 0, 1],
    };
    const world = nodeWorldPosition(pose, node.position);

    // The joint axes are the parent node's axes turned by the hinge mount.
    const orientation = new THREE.Quaternion(...pose.quaternion)
      .multiply(new THREE.Quaternion(...node.orientation))
      .multiply(new THREE.Quaternion(...hinge.mount));

    return { position: new THREE.Vector3(...world), quaternion: orientation };
  }, [bodies, hinge.mount, hinge.parentBodyId, hinge.parentNodeId, poses]);

  if (!placement) return null;

  const size = 0.055 * scale;
  const glyph = 0.3 * scale;
  const freeTranslations = [0, 1, 2].filter((i) => hinge.dof[i]?.free);
  const freeRotations = [0, 1, 2].filter((i) => hinge.dof[i + 3]?.free);
  const locked = freeTranslations.length === 0 && freeRotations.length === 0;

  return (
    <group position={placement.position} quaternion={placement.quaternion}>
      <mesh
        onClick={(event) => {
          event.stopPropagation();
          onSelect();
        }}
      >
        {locked ? <boxGeometry args={[size * 1.6, size * 1.6, size * 1.6]} /> : <sphereGeometry args={[size, 16, 12]} />}
        <meshStandardMaterial
          color={HINGE_COLOR}
          emissive={HINGE_COLOR}
          emissiveIntensity={selected ? 0.6 : 0.25}
          roughness={0.35}
        />
      </mesh>

      {freeTranslations.map((axis) => {
        const direction = AXIS_VECTORS[axis]!;
        const quaternion = new THREE.Quaternion().setFromUnitVectors(UP, direction);
        return (
          <group key={`t${axis}`} quaternion={quaternion}>
            {/* Double-headed: a prismatic axis slides both ways. */}
            {[1, -1].map((sign) => (
              <group key={sign} scale={[1, sign, 1]}>
                <mesh position={[0, glyph * 0.55, 0]}>
                  <cylinderGeometry args={[size * 0.16, size * 0.16, glyph * 0.7, 8]} />
                  <meshStandardMaterial color={AXIS_COLORS[AXIS_NAMES[axis]!]} />
                </mesh>
                <mesh position={[0, glyph * 0.95, 0]}>
                  <coneGeometry args={[size * 0.42, glyph * 0.24, 12]} />
                  <meshStandardMaterial color={AXIS_COLORS[AXIS_NAMES[axis]!]} />
                </mesh>
              </group>
            ))}
          </group>
        );
      })}

      {freeRotations.map((axis) => {
        const direction = AXIS_VECTORS[axis]!;
        // A torus lies in its own XY plane with its axis along +Z, so point +Z at the axis.
        const quaternion = new THREE.Quaternion().setFromUnitVectors(
          new THREE.Vector3(0, 0, 1),
          direction,
        );
        return (
          <mesh key={`r${axis}`} quaternion={quaternion}>
            <torusGeometry args={[glyph * 0.5, size * 0.17, 10, 32]} />
            <meshStandardMaterial
              color={AXIS_COLORS[AXIS_NAMES[axis]!]}
              emissive={AXIS_COLORS[AXIS_NAMES[axis]!]}
              emissiveIntensity={selected ? 0.35 : 0.15}
            />
          </mesh>
        );
      })}

      {selected && (
        // Below the joint, where node labels are not: a hinge and the node it attaches to
        // occupy the same point, so putting both labels above would stack them.
        <Label text={hinge.name} color={HINGE_COLOR} position={[0, 0, -glyph * 0.75]} scale={0.055 * scale} />
      )}
    </group>
  );
}
