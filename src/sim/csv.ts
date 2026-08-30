import type { Body } from '../types';
import { frameEnergy, frameQ, frameTime, frameV, type Trajectory } from './useSimulation';
import { bodyPoses, bodyVelocities, nodeWorldPosition, totalMomentum, type SolverModel } from './kinematics';

/**
 * CSV export.
 *
 * One button, everything in it. The point of this tool is to be compared against a more
 * serious simulator, and that comparison happens in a spreadsheet — so making the user
 * choose columns first only creates the chance of discovering the missing one after the run
 * is gone. Every column that can be derived from a frame is emitted.
 *
 * The per-frame derived quantities (poses, velocities, momentum) are recomputed here rather
 * than read from storage, for the same reason the trajectory does not store them: it is
 * microseconds of work against megabytes of memory.
 */

/** Excel-safe: quote anything containing a separator, a quote or a newline. */
function cell(value: string): string {
  return /[",\n\r]/.test(value) ? `"${value.replace(/"/g, '""')}"` : value;
}

/**
 * Enough digits to round-trip a double exactly, without the noise of always printing 17.
 *
 * A comparison against another tool is only as good as the digits that survive the export,
 * so this errs firmly on the side of precision over readability.
 */
function fmt(value: number): string {
  if (!Number.isFinite(value)) return '';
  if (value === 0) return '0';
  return String(Number(value.toPrecision(15)));
}

export type CsvOptions = {
  /** Restrict to these column headers. Omit for everything, which is the default. */
  columns?: Set<string>;
};

export function buildCsv(
  trajectory: Trajectory,
  solver: SolverModel,
  bodies: Record<string, Body>,
  options: CsvOptions = {},
): string {
  const { meta } = trajectory;
  const headers: string[] = ['time'];
  const rows: string[][] = [];

  const movable = [...solver.linkOf.keys()].filter((id) => bodies[id] && !bodies[id]!.isGround);

  // --- header ------------------------------------------------------------------------
  for (const name of meta.dofNames) headers.push(`q.${name}`);
  for (const name of meta.dofNames) headers.push(`u.${name}`);

  for (const id of movable) {
    const body = bodies[id]!;
    headers.push(
      `${body.name}.x`, `${body.name}.y`, `${body.name}.z`,
      `${body.name}.qx`, `${body.name}.qy`, `${body.name}.qz`, `${body.name}.qw`,
      `${body.name}.vx`, `${body.name}.vy`, `${body.name}.vz`,
      `${body.name}.wx`, `${body.name}.wy`, `${body.name}.wz`,
    );
    for (const nodeId of body.nodeOrder) {
      const node = body.nodes[nodeId]!;
      headers.push(`${body.name}.${node.name}.x`, `${body.name}.${node.name}.y`, `${body.name}.${node.name}.z`);
    }
  }

  headers.push(
    'energy.kinetic', 'energy.potential', 'energy.total',
    'momentum.px', 'momentum.py', 'momentum.pz',
    'momentum.lx', 'momentum.ly', 'momentum.lz',
  );

  // --- rows --------------------------------------------------------------------------
  for (let frame = 0; frame < trajectory.count; frame++) {
    const q = frameQ(trajectory, frame);
    const v = frameV(trajectory, frame);
    const poses = bodyPoses(solver, q);
    const velocities = bodyVelocities(solver, q, v);
    const momentum = totalMomentum(solver, q, v);
    const energy = frameEnergy(trajectory, frame);

    const row: string[] = [fmt(frameTime(trajectory, frame))];
    for (let i = 0; i < meta.nq; i++) row.push(fmt(q[i]!));
    for (let i = 0; i < meta.nv; i++) row.push(fmt(v[i]!));

    for (const id of movable) {
      const body = bodies[id]!;
      const pose = poses.get(id);
      const vel = velocities.get(id);
      if (!pose || !vel) {
        // 13 body columns plus 3 per node, kept in step with the header above.
        row.push(...new Array(13 + body.nodeOrder.length * 3).fill(''));
        continue;
      }
      row.push(...pose.position.map(fmt), ...pose.quaternion.map(fmt));
      row.push(...vel.linear.map(fmt), ...vel.angular.map(fmt));
      for (const nodeId of body.nodeOrder) {
        row.push(...nodeWorldPosition(pose, body.nodes[nodeId]!.position).map(fmt));
      }
    }

    row.push(fmt(energy.kinetic), fmt(energy.potential), fmt(energy.total));
    row.push(...momentum.linear.map(fmt), ...momentum.angular.map(fmt));
    rows.push(row);
  }

  const keep = options.columns
    ? headers.map((h, i) => (options.columns!.has(h) || i === 0 ? i : -1)).filter((i) => i >= 0)
    : headers.map((_, i) => i);

  const lines = [keep.map((i) => cell(headers[i]!)).join(',')];
  for (const row of rows) lines.push(keep.map((i) => cell(row[i] ?? '')).join(','));
  return lines.join('\n');
}

/**
 * Hand the CSV to the browser as a download.
 *
 * An object URL rather than a data URL: a long run is easily tens of megabytes, which is
 * past what data URLs handle reliably.
 */
export function downloadCsv(filename: string, contents: string): void {
  const blob = new Blob([contents], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  link.remove();
  // Revoked on the next tick so the click has definitely been dispatched first.
  setTimeout(() => URL.revokeObjectURL(url), 0);
}
