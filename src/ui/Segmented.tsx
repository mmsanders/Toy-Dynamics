type Option<T extends string> = {
  value: T;
  label: string;
  title?: string;
};

type Props<T extends string> = {
  label: string;
  value: T;
  options: readonly Option<T>[];
  onChange: (value: T) => void;
  /** Lets a long option list wrap onto more than one row on a narrow screen. */
  wrap?: boolean;
};

/** A row of mutually exclusive choices, sized for thumbs rather than cursors. */
export function Segmented<T extends string>({
  label,
  value,
  options,
  onChange,
  wrap,
}: Props<T>) {
  return (
    <div className="segmented">
      <span className="segmented__label">{label}</span>
      <div
        className={`segmented__options${wrap ? ' segmented__options--wrap' : ''}`}
        role="group"
        aria-label={label}
      >
        {options.map((option) => (
          <button
            key={option.value}
            type="button"
            title={option.title ?? option.label}
            aria-pressed={option.value === value}
            className={`segmented__button${option.value === value ? ' is-active' : ''}`}
            onClick={() => onChange(option.value)}
          >
            {option.label}
          </button>
        ))}
      </div>
    </div>
  );
}
