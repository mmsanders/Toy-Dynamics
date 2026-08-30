import { useCopy, type CopyStatus } from './useCopy';

export type CopyValueProps = {
  name: string;
  value: string;
  color?: string;
  copyKey: string;
  status: CopyStatus;
  onCopy: (key: string, text: string) => void;
};

/** One tap-to-copy number with its label. */
export function CopyValue({ name, value, color, copyKey, status, onCopy }: CopyValueProps) {
  // A failed copy says so rather than looking like nothing happened — over plain http the
  // Clipboard API is simply unavailable.
  const feedback = status?.key === copyKey ? (status.ok ? 'copied' : 'no clipboard') : null;

  return (
    <button
      type="button"
      className="value"
      title={`Copy ${name}`}
      onClick={() => onCopy(copyKey, value)}
    >
      <span className="value__name" style={color ? { color } : undefined}>
        {name}
      </span>
      <span className={`value__num${feedback && !status?.ok ? ' value__num--failed' : ''}`}>
        {feedback ?? value}
      </span>
    </button>
  );
}

type RowProps = {
  heading: string;
  values: { name: string; value: string; color?: string }[];
};

/**
 * A labelled row of tap-to-copy values that owns its own copy state.
 *
 * The richer `Readout` threads copy state across several rows so one "copy all" can share
 * it; this is the simple case, for a standalone row.
 */
export function CopyableRow({ heading, values }: RowProps) {
  const [status, copy] = useCopy();
  const columns = values.length >= 4 ? 'readout__grid--4' : 'readout__grid--3';

  return (
    <div className="readout__block">
      <header className="readout__head">
        <h4>{heading}</h4>
      </header>
      <div className={`readout__grid ${columns}`}>
        {values.map((entry) => (
          <CopyValue
            key={entry.name}
            name={entry.name}
            value={entry.value}
            {...(entry.color ? { color: entry.color } : {})}
            copyKey={`${heading}:${entry.name}`}
            status={status}
            onCopy={copy}
          />
        ))}
      </div>
    </div>
  );
}
