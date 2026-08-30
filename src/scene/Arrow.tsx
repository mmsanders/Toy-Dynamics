import { useMemo } from 'react';
import * as THREE from 'three';

/** Cylinders and cones are authored along +Y, so every arrow is built there and rotated. */
const UP = new THREE.Vector3(0, 1, 0);

type Props = {
  /** Direction the arrow points, in the parent group's space. Need not be normalised. */
  direction: THREE.Vector3;
  /** Length from the origin to the tip. */
  length: number;
  color: string;
  opacity?: number;
  /** Multiplier on the shaft radius, for emphasising a selection. */
  radiusScale?: number;
};

/**
 * A shaft-and-cone arrow pointing in an arbitrary direction.
 *
 * Shared by the coordinate axes and by scene vectors so the two read as the same object at
 * different orientations, rather than as two separately-tuned shapes.
 */
export function Arrow({ direction, length, color, opacity = 1, radiusScale = 1 }: Props) {
  const quaternion = useMemo(() => {
    const normalized = direction.clone();
    // A zero-length direction has no rotation to derive; leave it unrotated.
    if (normalized.lengthSq() < 1e-20) return new THREE.Quaternion();
    return new THREE.Quaternion().setFromUnitVectors(UP, normalized.normalize());
  }, [direction]);

  if (length <= 1e-6) return null;

  const headLength = Math.min(0.22 * length, 0.3);
  const shaftLength = Math.max(length - headLength, 1e-4);
  const shaftRadius = 0.012 * radiusScale * Math.max(1, length);
  const headRadius = shaftRadius * 2.6;
  const transparent = opacity < 1;

  return (
    <group quaternion={quaternion}>
      <mesh position={[0, shaftLength / 2, 0]}>
        <cylinderGeometry args={[shaftRadius, shaftRadius, shaftLength, 12]} />
        <meshStandardMaterial
          color={color}
          transparent={transparent}
          opacity={opacity}
          roughness={0.45}
          metalness={0.05}
        />
      </mesh>
      <mesh position={[0, shaftLength + headLength / 2, 0]}>
        <coneGeometry args={[headRadius, headLength, 16]} />
        <meshStandardMaterial
          color={color}
          transparent={transparent}
          opacity={opacity}
          roughness={0.45}
          metalness={0.05}
        />
      </mesh>
    </group>
  );
}
