import type { AxisName } from './math/conventions';

/**
 * Colours shared by the 3D scene and the readouts.
 *
 * Kept in their own module so both layers import the same constants, and so no component
 * file has to export a non-component alongside its component.
 */

export const AXIS_COLORS: Record<AxisName, string> = {
  X: '#ff5a5f',
  Y: '#4ade80',
  Z: '#60a5fa',
};

/** Marks the centre of mass, distinct from every body colour so it always reads as itself. */
export const COM_COLOR = '#e6ebf5';

/** Hinges and their free-axis glyphs. */
export const HINGE_COLOR = '#5b9dff';

/** A trajectory trace, which must stay legible against every body colour. */
export const TRACE_COLOR = '#8ea3c4';

export const SEVERITY_COLORS = {
  error: '#f87171',
  warning: '#f5a524',
  info: '#5b9dff',
} as const;

/**
 * Categorical plot slots, in fixed order.
 *
 * Assigned by series identity and never cycled: past eight, a ninth colour would be
 * indistinguishable from one of these under colour-vision deficiency, so the caller folds
 * the tail away instead.
 *
 * Validated against this app's panel surface (#121722) with the data-visualization
 * palette validator — lightness band, chroma floor, adjacent-pair CVD separation,
 * normal-vision floor and 3:1 contrast all pass.
 */
export const SERIES_COLORS = [
  '#3987e5',
  '#d95926',
  '#199e70',
  '#c98500',
  '#d55181',
  '#008300',
  '#9085e9',
  '#e66767',
] as const;

export const MAX_SERIES = SERIES_COLORS.length;
