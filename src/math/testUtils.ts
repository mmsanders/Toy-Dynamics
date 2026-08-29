import { expect } from 'vitest';
import type { Quat } from '../types';

/**
 * Assert two quaternions describe the same rotation.
 *
 * Deliberately not implemented with THREE.Quaternion.angleTo: that routes through
 * acos, whose derivative blows up at zero, so comparing near-identical rotations by
 * angle has a noise floor around 3e-8 even when every component is correct to machine
 * precision. Comparing components is well-conditioned and much stricter.
 *
 * The sign is aligned by the dot product rather than by canonicalising, because q and
 * -q are the same rotation and a half-turn can sit either side of w = 0.
 */
export function expectSameRotation(actual: Quat, expected: Quat, digits = 12): void {
  const dot =
    actual[0] * expected[0] +
    actual[1] * expected[1] +
    actual[2] * expected[2] +
    actual[3] * expected[3];
  const sign = dot < 0 ? -1 : 1;
  for (let i = 0; i < 4; i++) {
    expect(actual[i]).toBeCloseTo(sign * expected[i]!, digits);
  }
}
