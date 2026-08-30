import type { ReactNode } from 'react';

/**
 * Small shared pieces of panel furniture.
 *
 * Here rather than repeated inline so that a list row, a section header and a picker look
 * and behave the same in every tab — the panels differ in what they edit, not in how they
 * are built.
 */

export function Section({
  title,
  action,
  children,
}: {
  title: string;
  action?: ReactNode;
  children: ReactNode;
}) {
  return (
    <section className="section">
      <header className="section__head">
        <h3>{title}</h3>
        {action}
      </header>
      <div className="section__body">{children}</div>
    </section>
  );
}

export function ListRow({
  label,
  detail,
  color,
  active,
  indent = 0,
  onSelect,
  actions,
}: {
  label: string;
  detail?: string;
  color?: string;
  active: boolean;
  /** Tree depth, so a chain reads as a chain rather than a flat list. */
  indent?: number;
  onSelect: () => void;
  actions?: ReactNode;
}) {
  return (
    <div className={`row${active ? ' is-active' : ''}`} style={{ paddingLeft: 8 + indent * 14 }}>
      <button type="button" className="row__main" onClick={onSelect}>
        {color && <span className="row__swatch" style={{ background: color }} />}
        <span className="row__label">{label}</span>
        {detail && <span className="row__detail">{detail}</span>}
      </button>
      {actions && <div className="row__actions">{actions}</div>}
    </div>
  );
}

export function IconButton({
  label,
  onClick,
  danger,
  active,
  children,
  disabled,
}: {
  label: string;
  onClick: () => void;
  danger?: boolean;
  active?: boolean;
  children: ReactNode;
  disabled?: boolean;
}) {
  return (
    <button
      type="button"
      className={`icon-button${danger ? ' is-danger' : ''}${active ? ' is-active' : ''}`}
      title={label}
      aria-label={label}
      onClick={onClick}
      disabled={disabled ?? false}
    >
      {children}
    </button>
  );
}

export function TextField({
  label,
  value,
  onChange,
}: {
  label: string;
  value: string;
  onChange: (value: string) => void;
}) {
  return (
    <label className="text-field">
      <span className="text-field__label">{label}</span>
      <input
        className="text-field__input"
        type="text"
        value={value}
        onChange={(event) => onChange(event.target.value)}
      />
    </label>
  );
}

export function Picker<T extends string>({
  label,
  value,
  options,
  onChange,
}: {
  label: string;
  value: T;
  options: { value: T; label: string; disabled?: boolean }[];
  onChange: (value: T) => void;
}) {
  return (
    <label className="picker">
      <span className="picker__label">{label}</span>
      <select
        className="picker__select"
        value={value}
        onChange={(event) => onChange(event.target.value as T)}
      >
        {options.map((option) => (
          <option key={option.value} value={option.value} disabled={option.disabled ?? false}>
            {option.label}
          </option>
        ))}
      </select>
    </label>
  );
}

export function Toggle({
  label,
  checked,
  onChange,
  hint,
}: {
  label: string;
  checked: boolean;
  onChange: (checked: boolean) => void;
  hint?: string;
}) {
  return (
    <label className="toggle">
      <input type="checkbox" checked={checked} onChange={(event) => onChange(event.target.checked)} />
      <span className="toggle__label">
        {label}
        {hint && <span className="toggle__hint">{hint}</span>}
      </span>
    </label>
  );
}

export function Note({ children, tone = 'plain' }: { children: ReactNode; tone?: 'plain' | 'warn' }) {
  return <p className={`note${tone === 'warn' ? ' note--warn' : ''}`}>{children}</p>;
}

export function EmptyState({ children }: { children: ReactNode }) {
  return <p className="empty">{children}</p>;
}
