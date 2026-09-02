import type { V3 } from './spatial';

/** Largest grid accepted from storage or the authoring UI. Keeps links and render meshes sane. */
export const MAX_HEIGHTFIELD_AXIS = 64;
export const MAX_HEIGHTFIELD_SAMPLES = MAX_HEIGHTFIELD_AXIS * MAX_HEIGHTFIELD_AXIS;

/** One allocation-free answer shared by planes and heightfields. */
export type SurfaceQueryResult = {
  /** Positive outside the surface, zero at touch, negative while overlapping. */
  separation: number;
  /** Unit normal pointing to the allowed side. */
  normal: V3;
  /** Closest/query surface point in world coordinates. */
  point: V3;
  /** World velocity of that point. Fixed surfaces write zero. */
  velocity: V3;
};

/**
 * The solver-facing surface seam. A future moving or imported surface can implement this
 * without changing the compliant force law.
 */
export interface SurfaceQuery {
  querySphere(
    cx: number,
    cy: number,
    cz: number,
    radius: number,
    out: SurfaceQueryResult,
  ): boolean;
}

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

/** A single-valued Z-up regular grid in world coordinates. NaN is the compiled no-data value. */
export type HeightfieldGeometry = {
  origin: V3;
  spacing: number;
  columns: number;
  rows: number;
  heights: Float64Array;
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

/** Surface-query form of the analytical plane calculation. */
export function queryPlaneSurface(
  plane: PlaneGeometry,
  cx: number,
  cy: number,
  cz: number,
  radius: number,
  out: SurfaceQueryResult,
): boolean {
  const penetration = spherePlanePenetration(plane, cx, cy, cz, radius, out.normal);
  out.separation = -penetration;
  const distanceToSurface = radius + out.separation;
  out.point[0] = cx - out.normal[0]! * distanceToSurface;
  out.point[1] = cy - out.normal[1]! * distanceToSurface;
  out.point[2] = cz - out.normal[2]! * distanceToSurface;
  out.velocity.fill(0);
  return true;
}

/**
 * Bilinearly sample a heightfield and its analytic gradient.
 *
 * The outer sample row/column is included: it uses the adjacent final cell at interpolation
 * coordinate 1. Anything beyond it is no contact. A cell with any NaN corner is a hole.
 */
export function sampleHeightfield(
  field: HeightfieldGeometry,
  x: number,
  y: number,
  out: { height: number; dx: number; dy: number },
): boolean {
  if (!(field.spacing > 0) || field.columns < 2 || field.rows < 2) return false;
  let u = (x - field.origin[0]!) / field.spacing;
  let v = (y - field.origin[1]!) / field.spacing;
  const maxU = field.columns - 1;
  const maxV = field.rows - 1;
  const tolerance = 1e-12;
  if (u < -tolerance || v < -tolerance || u > maxU + tolerance || v > maxV + tolerance) return false;
  u = Math.min(maxU, Math.max(0, u));
  v = Math.min(maxV, Math.max(0, v));
  const ix = Math.min(field.columns - 2, Math.floor(u));
  const iy = Math.min(field.rows - 2, Math.floor(v));
  const tx = u - ix;
  const ty = v - iy;
  const row0 = iy * field.columns;
  const row1 = row0 + field.columns;
  const z00 = field.heights[row0 + ix]!;
  const z10 = field.heights[row0 + ix + 1]!;
  const z01 = field.heights[row1 + ix]!;
  const z11 = field.heights[row1 + ix + 1]!;
  if (!Number.isFinite(z00) || !Number.isFinite(z10)
      || !Number.isFinite(z01) || !Number.isFinite(z11)) return false;

  const oneX = 1 - tx;
  const oneY = 1 - ty;
  out.height = oneX * oneY * z00 + tx * oneY * z10 + oneX * ty * z01 + tx * ty * z11;
  out.dx = ((1 - ty) * (z10 - z00) + ty * (z11 - z01)) / field.spacing;
  out.dy = ((1 - tx) * (z01 - z00) + tx * (z11 - z10)) / field.spacing;
  return true;
}

/**
 * Query the tangent plane under a sphere centre.
 *
 * This is the documented finite-radius approximation: the grid supplies height and local
 * normal at the centre's X/Y projection. Spheres should not be much larger than a cell.
 */
export function queryHeightfieldSurface(
  field: HeightfieldGeometry,
  cx: number,
  cy: number,
  cz: number,
  radius: number,
  out: SurfaceQueryResult,
  sample: { height: number; dx: number; dy: number } = { height: 0, dx: 0, dy: 0 },
): boolean {
  if (!sampleHeightfield(field, cx, cy, sample)) return false;
  const length = Math.hypot(sample.dx, sample.dy, 1);
  out.normal[0] = -sample.dx / length;
  out.normal[1] = -sample.dy / length;
  out.normal[2] = 1 / length;
  out.point[0] = cx;
  out.point[1] = cy;
  out.point[2] = field.origin[2]! + sample.height;
  out.velocity.fill(0);
  out.separation =
    (cx - out.point[0]!) * out.normal[0]! +
    (cy - out.point[1]!) * out.normal[1]! +
    (cz - out.point[2]!) * out.normal[2]! - radius;
  return true;
}
