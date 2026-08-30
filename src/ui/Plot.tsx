import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

/**
 * Time-history plots.
 *
 * The point of this tool is to be compared against a more serious simulator, and that
 * comparison is usually "does this curve look like that curve". So these are built for
 * reading values off, not for decoration.
 *
 * Decisions worth stating:
 *
 *  - **One unit per chart, never two y-scales.** Aligning two scales on one plot invents a
 *    correlation that is not in the data. Angles and offsets get separate charts even
 *    though both are "joint coordinates", because radians and metres do not share an axis.
 *  - **Colour follows the series, not its position.** Unchecking a series never repaints
 *    the others, so "the elbow is orange" stays true as you filter.
 *  - **Extremes survive downsampling.** A long run has far more samples than pixels, and
 *    plain stride subsampling drops exactly the spikes a sanity check is looking for. Each
 *    pixel column keeps its own minimum and maximum.
 */

export type Series = {
  id: string;
  label: string;
  color: string;
  /** Value at a frame index. Read lazily so nothing is materialized until it is drawn. */
  at: (index: number) => number;
};

type Props = {
  title: string;
  unit: string;
  series: Series[];
  count: number;
  timeAt: (index: number) => number;
  /** Frame the playhead sits on, drawn as a vertical rule. */
  playhead?: number;
  height?: number;
};

/** Container width, so strokes and text render at true pixel size rather than being scaled. */
function useWidth(): [React.RefObject<HTMLDivElement | null>, number] {
  const ref = useRef<HTMLDivElement | null>(null);
  const [width, setWidth] = useState(320);

  useEffect(() => {
    const element = ref.current;
    if (!element) return;
    const observer = new ResizeObserver((entries) => {
      const next = entries[0]?.contentRect.width;
      if (next && next > 0) setWidth(next);
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  return [ref, width];
}

/** A tick step that lands on 1, 2 or 5 times a power of ten. */
function niceStep(span: number, target: number): number {
  if (span <= 0) return 1;
  const raw = span / Math.max(target, 1);
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const normalized = raw / magnitude;
  const step = normalized >= 5 ? 5 : normalized >= 2 ? 2 : 1;
  return step * magnitude;
}

function formatTick(value: number, step: number): string {
  if (value === 0) return '0';
  const decimals = Math.max(0, Math.min(6, -Math.floor(Math.log10(step))));
  const fixed = value.toFixed(decimals);
  return fixed === `-${(0).toFixed(decimals)}` ? (0).toFixed(decimals) : fixed;
}

const PAD = { left: 46, right: 12, top: 10, bottom: 22 };

export function Plot({ title, unit, series, count, timeAt, playhead, height = 132 }: Props) {
  const [ref, width] = useWidth();
  const [hover, setHover] = useState<number | null>(null);

  const plotWidth = Math.max(40, width - PAD.left - PAD.right);
  const plotHeight = height - PAD.top - PAD.bottom;

  /**
   * Per-column min/max decimation.
   *
   * Two vertices per pixel column, so a spike narrower than a pixel still shows as a spike
   * rather than vanishing between samples.
   */
  const paths = useMemo(() => {
    if (count < 2 || series.length === 0) return null;

    const columns = Math.max(2, Math.min(count, Math.floor(plotWidth)));
    let lo = Infinity;
    let hi = -Infinity;

    const perSeries = series.map((s) => {
      const points: { index: number; min: number; max: number }[] = [];
      for (let c = 0; c < columns; c++) {
        const start = Math.floor((c * count) / columns);
        const end = Math.max(start + 1, Math.floor(((c + 1) * count) / columns));
        let min = Infinity;
        let max = -Infinity;
        for (let i = start; i < end && i < count; i++) {
          const value = s.at(i);
          if (!Number.isFinite(value)) continue;
          if (value < min) min = value;
          if (value > max) max = value;
        }
        if (min === Infinity) continue;
        points.push({ index: start, min, max });
        if (min < lo) lo = min;
        if (max > hi) hi = max;
      }
      return { series: s, points };
    });

    if (lo === Infinity) return null;
    if (hi - lo < 1e-12) {
      const pad = Math.max(Math.abs(hi) * 0.1, 0.5);
      lo -= pad;
      hi += pad;
    } else {
      const pad = (hi - lo) * 0.08;
      lo -= pad;
      hi += pad;
    }

    const span = timeAt(count - 1) - timeAt(0) || 1;
    const x = (index: number): number => PAD.left + ((timeAt(index) - timeAt(0)) / span) * plotWidth;
    const y = (value: number): number => PAD.top + (1 - (value - lo) / (hi - lo)) * plotHeight;

    return {
      lo,
      hi,
      x,
      y,
      span,
      lines: perSeries.map(({ series: s, points }) => ({
        id: s.id,
        color: s.color,
        label: s.label,
        // Straight through the midpoint of each column, with the min/max envelope implied
        // by stepping to both — cheap, and it keeps spikes visible.
        d: points
          .map((p, i) => {
            const px = x(p.index).toFixed(2);
            const yMax = y(p.max).toFixed(2);
            const yMin = y(p.min).toFixed(2);
            return `${i === 0 ? 'M' : 'L'}${px},${yMax} L${px},${yMin}`;
          })
          .join(' '),
        last: points.length > 0 ? points[points.length - 1]! : null,
      })),
    };
  }, [series, count, plotWidth, plotHeight, timeAt]);

  const onPointerMove = useCallback(
    (event: React.PointerEvent<SVGSVGElement>) => {
      if (!paths || count < 2) return;
      const rect = event.currentTarget.getBoundingClientRect();
      const fraction = (event.clientX - rect.left - PAD.left) / plotWidth;
      const index = Math.round(Math.min(1, Math.max(0, fraction)) * (count - 1));
      setHover(index);
    },
    [paths, count, plotWidth],
  );

  if (!paths) {
    return (
      <figure className="plot plot--empty" ref={ref}>
        <figcaption className="plot__title">{title}</figcaption>
        <p className="plot__empty">Nothing selected.</p>
      </figure>
    );
  }

  const yStep = niceStep(paths.hi - paths.lo, 3);
  const yTicks: number[] = [];
  for (let t = Math.ceil(paths.lo / yStep) * yStep; t <= paths.hi; t += yStep) yTicks.push(t);

  const xStep = niceStep(paths.span, 4);
  const xTicks: number[] = [];
  for (let t = Math.ceil(timeAt(0) / xStep) * xStep; t <= timeAt(count - 1) + 1e-9; t += xStep) xTicks.push(t);

  const cursor = hover ?? playhead ?? null;

  return (
    <figure className="plot" ref={ref}>
      <figcaption className="plot__title">
        {title}
        <span className="plot__unit">{unit}</span>
      </figcaption>

      <svg
        className="plot__svg"
        width={width}
        height={height}
        onPointerMove={onPointerMove}
        onPointerLeave={() => setHover(null)}
        role="img"
        aria-label={`${title} against time`}
      >
        {/* Solid hairlines, one shade off the surface. Dashing would read as a threshold. */}
        {yTicks.map((tick) => (
          <g key={`y${tick}`}>
            <line
              x1={PAD.left}
              y1={paths.y(tick)}
              x2={PAD.left + plotWidth}
              y2={paths.y(tick)}
              className="plot__grid"
            />
            <text x={PAD.left - 6} y={paths.y(tick)} className="plot__tick plot__tick--y">
              {formatTick(tick, yStep)}
            </text>
          </g>
        ))}

        {xTicks.map((tick) => {
          const index = Math.round(((tick - timeAt(0)) / paths.span) * (count - 1));
          return (
            <text key={`x${tick}`} x={paths.x(index)} y={height - 6} className="plot__tick plot__tick--x">
              {formatTick(tick, xStep)}
            </text>
          );
        })}

        {cursor !== null && cursor < count && (
          <line
            x1={paths.x(cursor)}
            y1={PAD.top}
            x2={paths.x(cursor)}
            y2={PAD.top + plotHeight}
            className={hover === null ? 'plot__playhead' : 'plot__crosshair'}
          />
        )}

        {paths.lines.map((line) => (
          <path key={line.id} d={line.d} stroke={line.color} className="plot__line" />
        ))}

        {/* Direct labels only where they fit: past four series they collide and the legend
            carries identity instead. */}
        {series.length <= 4 &&
          paths.lines.map((line) =>
            line.last ? (
              <circle
                key={`dot-${line.id}`}
                cx={paths.x(line.last.index)}
                cy={paths.y(line.last.max)}
                r={3}
                fill={line.color}
                className="plot__endpoint"
              />
            ) : null,
          )}

        {cursor !== null &&
          cursor < count &&
          series.map((s) => {
            const value = s.at(cursor);
            if (!Number.isFinite(value)) return null;
            return (
              <circle
                key={`cursor-${s.id}`}
                cx={paths.x(cursor)}
                cy={paths.y(value)}
                r={3.5}
                fill={s.color}
                className="plot__marker"
              />
            );
          })}
      </svg>

      {cursor !== null && cursor < count && (
        <div className="plot__values">
          <span className="plot__time">t = {timeAt(cursor).toFixed(3)}</span>
          {series.map((s) => (
            <span key={s.id} className="plot__value">
              <span className="plot__swatch" style={{ background: s.color }} />
              {s.label}
              <b>{formatValue(s.at(cursor))}</b>
            </span>
          ))}
        </div>
      )}
    </figure>
  );
}

function formatValue(value: number): string {
  if (!Number.isFinite(value)) return '—';
  const abs = Math.abs(value);
  if (abs !== 0 && (abs >= 1e5 || abs < 1e-3)) return value.toExponential(2);
  return String(Math.round(value * 1e4) / 1e4);
}

/**
 * The legend.
 *
 * Always present for two or more series, so identity never rests on colour alone. The
 * swatch carries the colour; the text stays in the panel's own ink, because coloured text
 * on a dark surface reads worse and doubles up an encoding that the swatch already makes.
 */
export function Legend({ series }: { series: Series[] }) {
  if (series.length < 2) return null;
  return (
    <div className="legend">
      {series.map((s) => (
        <span key={s.id} className="legend__item">
          <span className="legend__swatch" style={{ background: s.color }} />
          {s.label}
        </span>
      ))}
    </div>
  );
}
