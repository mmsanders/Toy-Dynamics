import * as THREE from 'three';
import { Grid } from '@react-three/drei';

/**
 * The horizontal reference plane.
 *
 * Rendered in three.js space, *outside* the scene's mount group. That is deliberate:
 * drei's Grid lies in the three.js XZ plane, which is the horizontal plane on screen no
 * matter which engineering axis the user has chosen as up. Putting the grid inside the
 * mount group would tip it on its side in Z-up mode, and would need a compensating
 * rotation to undo something we never wanted applied.
 */
export function GroundGrid() {
  return (
    <Grid
      args={[40, 40]}
      cellSize={0.5}
      cellThickness={0.6}
      cellColor="#1e2636"
      sectionSize={2}
      sectionThickness={1}
      sectionColor="#2f3d55"
      fadeDistance={34}
      fadeStrength={1.2}
      infiniteGrid
      side={THREE.DoubleSide}
    />
  );
}
