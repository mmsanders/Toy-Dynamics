import { GROUND_ID, type Hinge } from '../types';

/**
 * Tree topology over the hinge set.
 *
 * The invariant the whole solver rests on: **every non-ground body has exactly one inbound
 * hinge**, and that hinge is its parent pointer. There is no separate parent field to fall
 * out of sync with the hinge list, and no way to express a closed loop — which is precisely
 * what makes over-constraint impossible in reduced coordinates.
 *
 * Structured after `wouldCreateCycle` / `ancestorsOf` in Rotation Wizard's transforms
 * module, for the same reason: it is cheaper to disable an illegal choice in the picker
 * than to let the user make it and then fail.
 */

/** The hinge that attaches `bodyId` to its parent, if any. Ground has none. */
export function inboundHinge(hinges: Record<string, Hinge>, bodyId: string): Hinge | undefined {
  for (const hinge of Object.values(hinges)) {
    if (hinge.childBodyId === bodyId) return hinge;
  }
  return undefined;
}

export function parentOf(hinges: Record<string, Hinge>, bodyId: string): string | null {
  return inboundHinge(hinges, bodyId)?.parentBodyId ?? null;
}

export function childrenOf(hinges: Record<string, Hinge>, bodyId: string): string[] {
  return Object.values(hinges)
    .filter((h) => h.parentBodyId === bodyId)
    .map((h) => h.childBodyId);
}

/** Ancestor body ids, nearest parent first. Terminates even on a malformed cyclic set. */
export function ancestorsOf(hinges: Record<string, Hinge>, bodyId: string): string[] {
  const out: string[] = [];
  const seen = new Set<string>([bodyId]);
  let current = parentOf(hinges, bodyId);
  while (current && !seen.has(current)) {
    out.push(current);
    seen.add(current);
    current = parentOf(hinges, current);
  }
  return out;
}

/**
 * Whether re-parenting `bodyId` onto `newParentId` would close a loop.
 *
 * Used to grey out ineligible options in the parent picker rather than letting the user
 * choose one and then explaining why it failed.
 */
export function wouldCreateCycle(
  hinges: Record<string, Hinge>,
  bodyId: string,
  newParentId: string,
): boolean {
  if (newParentId === bodyId) return true;
  return ancestorsOf(hinges, newParentId).includes(bodyId);
}

/** Depth below ground, for indenting the body list. Ground is 0. */
export function depthOf(hinges: Record<string, Hinge>, bodyId: string): number {
  return bodyId === GROUND_ID ? 0 : ancestorsOf(hinges, bodyId).length;
}

/**
 * Hinge ids ordered parent-before-child.
 *
 * The solver requires this ordering, and a set that cannot be ordered has a cycle in it —
 * reported rather than thrown, so the UI can show a warning and still render everything
 * else.
 */
export function orderHingeIds(hinges: Record<string, Hinge>): {
  ordered: string[];
  cycle: string[] | null;
} {
  const byChild = new Map<string, Hinge>();
  for (const hinge of Object.values(hinges)) byChild.set(hinge.childBodyId, hinge);

  const ordered: string[] = [];
  const state = new Map<string, 'visiting' | 'done'>();

  const visit = (bodyId: string, trail: string[]): string[] | null => {
    const mark = state.get(bodyId);
    if (mark === 'done') return null;
    if (mark === 'visiting') {
      const start = trail.indexOf(bodyId);
      return trail.slice(start === -1 ? 0 : start);
    }
    const hinge = byChild.get(bodyId);
    if (!hinge) return null;

    state.set(bodyId, 'visiting');
    const cycle = visit(hinge.parentBodyId, [...trail, bodyId]);
    if (cycle) return cycle;
    state.set(bodyId, 'done');
    ordered.push(hinge.id);
    return null;
  };

  for (const hinge of Object.values(hinges)) {
    const cycle = visit(hinge.childBodyId, []);
    if (cycle) return { ordered: [], cycle };
  }
  return { ordered, cycle: null };
}
