import type { V3 } from './spatial';

/**
 * Contact geometry, separated from the force law that uses it.
 *
 * One implementation answers three questions — what force to apply, how much energy the
 * contact spring is holding, and whether the model starts out already overlapping — so a
 * diagnostic can never quietly disagree with the solver about where a surface is.
 */

/** The geometric half of a compiled contact plane. */
export type PlaneGeometry = {
  point: V3;
  /** Unit normal, pointing into the allowed half-space. */
  normal: V3;
  /** In-plane axes the patch is measured and drawn along. */
  tangentU: V3;
  tangentV: V3;
  /** Half the plate's side length. Meaningless when `bounded` is false. */
  halfSize: number;
  /** False for an unbounded plane, which is the whole half-space and has no edges. */
  bounded: boolean;
};

/**
 * A right-handed in-plane basis for a unit normal.
 *
 * Any choice is as good as any other for the maths — a plane is the same plane whichever
 * way its square is turned — but the scene *draws* that square, so the solver and the
 * renderer have to make the same choice or the plate you see is not the plate you hit.
 * Hence one shared construction rather than two conventions that agree by luck.
 *
 * Frisvad's branchless basis, with the sign trick that removes the singularity at the
 * south pole: `sign + nz` cannot vanish, so a normal pointing straight down is no more
 * special than any other.
 */
export function planeBasis(normal: V3, outU: V3, outV: V3): void {
  const nx = normal[0]!, ny = normal[1]!, nz = normal[2]!;
  const sign = nz >= 0 ? 1 : -1;
  const a = -1 / (sign + nz);
  const b = nx * ny * a;
  outU[0] = 1 + sign * nx * nx * a;
  outU[1] = sign * b;
  outU[2] = -sign * nx;
  outV[0] = b;
  outV[1] = sign + ny * ny * a;
  outV[2] = -ny;
}

/**
 * How far a sphere overlaps a plane, and which way that plane pushes it.
 *
 * Returns the penetration — positive when touching — and writes the unit contact normal
 * into `outNormal`. Allocation-free, because this runs inside the integrator's inner loop.
 *
 * An unbounded plane is the familiar half-space: penetration is `r − d` for signed distance
 * `d`, and the normal is the plane's own. Note that `d` is unbounded below, so a *point*
 * contact — radius zero, which only ever touches at `d ≤ 0` — still works.
 *
 * A bounded plate adds one case. When the centre hangs past an edge, the nearest part of
 * the plate is a point on its rim, so the contact is against that rim: the normal tips over
 * as the sphere goes past and the support fades out smoothly, which is what lets a ball
 * roll off an edge instead of dropping down an invisible step. Over the plate the two cases
 * agree exactly, so nothing about the interior changes by making a plane finite.
 */
export function spherePlanePenetration(
  plane: PlaneGeometry,
  cx: number,
  cy: number,
  cz: number,
  radius: number,
  outNormal: V3,
): number {
  const n = plane.normal;
  const dx = cx - plane.point[0]!;
  const dy = cy - plane.point[1]!;
  const dz = cz - plane.point[2]!;
  const distance = n[0]! * dx + n[1]! * dy + n[2]! * dz;

  if (plane.bounded) {
    const u = plane.tangentU;
    const v = plane.tangentV;
    const h = plane.halfSize;
    const alongU = u[0]! * dx + u[1]! * dy + u[2]! * dz;
    const alongV = v[0]! * dx + v[1]! * dy + v[2]! * dz;
    // How far the centre overhangs the plate along each axis — zero while it is over the
    // plate, which is then exactly the half-space case below.
    const overU = alongU - Math.min(h, Math.max(-h, alongU));
    const overV = alongV - Math.min(h, Math.max(-h, alongV));

    if (overU !== 0 || overV !== 0) {
      // Offset from the nearest rim point to the sphere centre.
      const wx = overU * u[0]! + overV * v[0]! + distance * n[0]!;
      const wy = overU * u[1]! + overV * v[1]! + distance * n[1]!;
      const wz = overU * u[2]! + overV * v[2]! + distance * n[2]!;
      const length = Math.hypot(wx, wy, wz);
      // A centre sitting exactly on the rim has no direction to be pushed in; keep the
      // face normal so the answer stays finite and deterministic.
      if (!(length > 1e-12)) {
        outNormal.set(n);
        return radius;
      }
      outNormal[0] = wx / length;
      outNormal[1] = wy / length;
      outNormal[2] = wz / length;
      return radius - length;
    }
  }

  outNormal.set(n);
  return radius - distance;
}
