import { useMemo, useState } from 'react';
import { useModelStore } from '../store/useModelStore';
import { runDiagnostics, type Diagnostic } from '../model/diagnostics';
import { SEVERITY_COLORS } from '../theme';

/**
 * The diagnostics banner.
 *
 * Advisory, never blocking. A tool whose job is checking other tools has to let you run a
 * model it dislikes — sometimes seeing what an absurd model does *is* the question. So this
 * explains and offers, and never refuses.
 *
 * Collapsed to a single line by default so it does not eat the screen, and dismissable per
 * finding: a warning you have read and decided to live with should stop shouting. Dismissals
 * key on the finding's identity, so fixing and reintroducing the same problem shows it
 * again rather than staying silently hidden.
 */

const SEVERITY_ORDER = { error: 0, warning: 1, info: 2 } as const;

export function WarningBanner() {
  const bodies = useModelStore((s) => s.bodies);
  const hinges = useModelStore((s) => s.hinges);
  const actuators = useModelStore((s) => s.actuators);
  const settings = useModelStore((s) => s.settings);
  const applyFix = useModelStore((s) => s.applyFix);
  const selectBody = useModelStore((s) => s.selectBody);
  const selectHinge = useModelStore((s) => s.selectHinge);
  const selectActuator = useModelStore((s) => s.selectActuator);

  const [dismissed, setDismissed] = useState<Set<string>>(() => new Set());
  const [open, setOpen] = useState(false);

  const diagnostics = useMemo(() => {
    const found = runDiagnostics(bodies, hinges, actuators, settings);
    return found.sort((a, b) => SEVERITY_ORDER[a.severity] - SEVERITY_ORDER[b.severity]);
  }, [bodies, hinges, actuators, settings]);

  const active = diagnostics.filter((d) => !dismissed.has(d.id));
  if (active.length === 0) return null;

  const worst = active[0]!.severity;
  const errors = active.filter((d) => d.severity === 'error').length;
  const warnings = active.filter((d) => d.severity === 'warning').length;

  const summary =
    errors > 0
      ? `${errors} problem${errors === 1 ? '' : 's'}`
      : warnings > 0
        ? `${warnings} warning${warnings === 1 ? '' : 's'}`
        : `${active.length} note${active.length === 1 ? '' : 's'}`;

  const goTo = (target: Diagnostic['target']) => {
    if (!target) return;
    if (target.kind === 'body') selectBody(target.id);
    if (target.kind === 'hinge') selectHinge(target.id);
    if (target.kind === 'actuator') selectActuator(target.id);
  };

  return (
    <div className={`banner banner--${worst}`}>
      <button type="button" className="banner__summary" onClick={() => setOpen((value) => !value)}>
        <span className="banner__dot" style={{ background: SEVERITY_COLORS[worst] }} />
        <span className="banner__text">{active[0]!.title}</span>
        {active.length > 1 && <span className="banner__count">{summary}</span>}
        <span className="banner__chevron">{open ? '▾' : '▸'}</span>
      </button>

      {open && (
        <ul className="banner__list">
          {active.map((diagnostic) => (
            <li key={diagnostic.id} className={`finding finding--${diagnostic.severity}`}>
              <div className="finding__head">
                <span className="finding__dot" style={{ background: SEVERITY_COLORS[diagnostic.severity] }} />
                <strong>{diagnostic.title}</strong>
              </div>
              <p className="finding__detail">{diagnostic.detail}</p>
              <div className="finding__actions">
                {diagnostic.fix && (
                  <button
                    type="button"
                    className="ghost-button"
                    onClick={() => applyFix(diagnostic.fix!)}
                  >
                    {diagnostic.fix.label}
                  </button>
                )}
                {diagnostic.target && diagnostic.target.kind !== 'settings' && (
                  <button type="button" className="ghost-button" onClick={() => goTo(diagnostic.target)}>
                    Show me
                  </button>
                )}
                <button
                  type="button"
                  className="ghost-button"
                  onClick={() => setDismissed((prev) => new Set(prev).add(diagnostic.id))}
                >
                  Dismiss
                </button>
              </div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
