/**
 * A tiny expression parser for the general `f(t)` actuator profile.
 *
 * Deliberately a real parser rather than `new Function`. Three reasons, in order of how
 * much they matter here:
 *
 *  - It can fail *before* the run starts, with a message and a caret pointing at the
 *    offending character, instead of throwing from inside the worker's inner loop.
 *  - It works under a strict Content-Security-Policy, where `new Function` does not.
 *  - Only `t` and the listed functions are in scope, so a pasted expression cannot reach
 *    anything else in the page.
 *
 * The parse produces a closure, not a token list walked per call: an actuator is evaluated
 * once per integration substep, which is several thousand times a second.
 */

const CONSTANTS: Record<string, number> = {
  pi: Math.PI,
  e: Math.E,
  tau: Math.PI * 2,
};

type Fn = (args: number[]) => number;

/**
 * `step`, `pulse` and `clamp` are here because they are what a thruster schedule is
 * actually made of, and writing them out of `min`/`max` every time is miserable.
 */
const FUNCTIONS: Record<string, { arity: number | [number, number]; fn: Fn }> = {
  sin: { arity: 1, fn: (a) => Math.sin(a[0]!) },
  cos: { arity: 1, fn: (a) => Math.cos(a[0]!) },
  tan: { arity: 1, fn: (a) => Math.tan(a[0]!) },
  asin: { arity: 1, fn: (a) => Math.asin(a[0]!) },
  acos: { arity: 1, fn: (a) => Math.acos(a[0]!) },
  atan: { arity: 1, fn: (a) => Math.atan(a[0]!) },
  atan2: { arity: 2, fn: (a) => Math.atan2(a[0]!, a[1]!) },
  exp: { arity: 1, fn: (a) => Math.exp(a[0]!) },
  log: { arity: 1, fn: (a) => Math.log(a[0]!) },
  log10: { arity: 1, fn: (a) => Math.log10(a[0]!) },
  sqrt: { arity: 1, fn: (a) => Math.sqrt(a[0]!) },
  abs: { arity: 1, fn: (a) => Math.abs(a[0]!) },
  sign: { arity: 1, fn: (a) => Math.sign(a[0]!) },
  floor: { arity: 1, fn: (a) => Math.floor(a[0]!) },
  ceil: { arity: 1, fn: (a) => Math.ceil(a[0]!) },
  round: { arity: 1, fn: (a) => Math.round(a[0]!) },
  min: { arity: [2, 8], fn: (a) => Math.min(...a) },
  max: { arity: [2, 8], fn: (a) => Math.max(...a) },
  /** 1 once its argument reaches zero, 0 before. `step(t - 2)` switches on at t = 2. */
  step: { arity: 1, fn: (a) => (a[0]! >= 0 ? 1 : 0) },
  /** 1 while `x` is in [lo, hi), 0 outside — a finite burn in one call. */
  pulse: { arity: 3, fn: (a) => (a[0]! >= a[1]! && a[0]! < a[2]! ? 1 : 0) },
  clamp: { arity: 3, fn: (a) => Math.min(a[2]!, Math.max(a[1]!, a[0]!)) },
};

export const EXPR_FUNCTION_NAMES = Object.keys(FUNCTIONS);
export const EXPR_CONSTANT_NAMES = Object.keys(CONSTANTS);

type Token =
  | { kind: 'number'; value: number; at: number }
  | { kind: 'ident'; name: string; at: number }
  | { kind: 'op'; op: string; at: number };

class ParseError extends Error {
  constructor(
    message: string,
    readonly at: number,
  ) {
    super(message);
  }
}

function tokenize(src: string): Token[] {
  const tokens: Token[] = [];
  let i = 0;
  while (i < src.length) {
    const ch = src[i]!;
    if (ch === ' ' || ch === '\t' || ch === '\n' || ch === '\r') {
      i++;
      continue;
    }
    if ((ch >= '0' && ch <= '9') || ch === '.') {
      const start = i;
      while (i < src.length && /[0-9.]/.test(src[i]!)) i++;
      // Exponent notation, but only when it really is one: `2e3` is a number, while the
      // `e` in `2*e` is the constant.
      if (src[i] === 'e' || src[i] === 'E') {
        const save = i;
        i++;
        if (src[i] === '+' || src[i] === '-') i++;
        if (i < src.length && /[0-9]/.test(src[i]!)) {
          while (i < src.length && /[0-9]/.test(src[i]!)) i++;
        } else {
          i = save;
        }
      }
      const text = src.slice(start, i);
      const value = Number(text);
      if (!Number.isFinite(value)) throw new ParseError(`"${text}" is not a number`, start);
      tokens.push({ kind: 'number', value, at: start });
      continue;
    }
    if (/[a-zA-Z_]/.test(ch)) {
      const start = i;
      while (i < src.length && /[a-zA-Z0-9_]/.test(src[i]!)) i++;
      tokens.push({ kind: 'ident', name: src.slice(start, i), at: start });
      continue;
    }
    if ('+-*/^%(),'.includes(ch)) {
      tokens.push({ kind: 'op', op: ch, at: i });
      i++;
      continue;
    }
    throw new ParseError(`Unexpected character "${ch}"`, i);
  }
  return tokens;
}

type Node = (t: number) => number;

/**
 * Recursive descent, chosen over shunting-yard because unary minus and multi-argument
 * function calls are where shunting-yard implementations quietly go wrong.
 */
function parseTokens(tokens: Token[], src: string): Node {
  let pos = 0;

  const peek = (): Token | undefined => tokens[pos];
  const eat = (op: string): boolean => {
    const tok = tokens[pos];
    if (tok && tok.kind === 'op' && tok.op === op) {
      pos++;
      return true;
    }
    return false;
  };
  const expect = (op: string): void => {
    if (!eat(op)) {
      const tok = tokens[pos];
      throw new ParseError(`Expected "${op}"`, tok ? tok.at : src.length);
    }
  };

  const parseExpr = (): Node => {
    let left = parseTerm();
    for (;;) {
      if (eat('+')) {
        const right = parseTerm();
        const l = left;
        left = (t) => l(t) + right(t);
      } else if (eat('-')) {
        const right = parseTerm();
        const l = left;
        left = (t) => l(t) - right(t);
      } else return left;
    }
  };

  const parseTerm = (): Node => {
    let left = parseUnary();
    for (;;) {
      if (eat('*')) {
        const right = parseUnary();
        const l = left;
        left = (t) => l(t) * right(t);
      } else if (eat('/')) {
        const right = parseUnary();
        const l = left;
        left = (t) => l(t) / right(t);
      } else if (eat('%')) {
        const right = parseUnary();
        const l = left;
        left = (t) => l(t) % right(t);
      } else return left;
    }
  };

  /**
   * Unary minus binds *looser* than exponentiation, so `-2^2` is `−(2²) = −4` rather than
   * `(−2)² = 4`. That is the maths convention, and the other way round is a classic
   * hand-rolled-parser bug — hence `parseUnary` sitting above `parsePower` rather than
   * below it.
   */
  const parseUnary = (): Node => {
    if (eat('-')) {
      const operand = parseUnary();
      return (t) => -operand(t);
    }
    if (eat('+')) return parseUnary();
    return parsePower();
  };

  const parsePower = (): Node => {
    const base = parseAtom();
    // Right-associative, so 2^3^2 is 2^(3^2). The exponent is a unary expression so that
    // `2^-3` parses.
    if (eat('^')) {
      const exponent = parseUnary();
      return (t) => base(t) ** exponent(t);
    }
    return base;
  };

  const parseAtom = (): Node => {
    const tok = peek();
    if (!tok) throw new ParseError('Unexpected end of expression', src.length);

    if (tok.kind === 'number') {
      pos++;
      const { value } = tok;
      return () => value;
    }

    if (tok.kind === 'ident') {
      pos++;
      const name = tok.name;
      const lower = name.toLowerCase();

      if (peek()?.kind === 'op' && (peek() as { op: string }).op === '(') {
        const entry = FUNCTIONS[lower];
        if (!entry) throw new ParseError(`Unknown function "${name}"`, tok.at);
        expect('(');
        const args: Node[] = [];
        if (!eat(')')) {
          do {
            args.push(parseExpr());
          } while (eat(','));
          expect(')');
        }
        const [lo, hi] = Array.isArray(entry.arity)
          ? entry.arity
          : [entry.arity, entry.arity];
        if (args.length < lo || args.length > hi) {
          const want = lo === hi ? `${lo}` : `${lo} to ${hi}`;
          throw new ParseError(
            `${name}() takes ${want} argument${hi === 1 ? '' : 's'}, got ${args.length}`,
            tok.at,
          );
        }
        const fn = entry.fn;
        const buffer: number[] = new Array(args.length).fill(0);
        return (t) => {
          for (let i = 0; i < args.length; i++) buffer[i] = args[i]!(t);
          return fn(buffer);
        };
      }

      if (lower === 't') return (t) => t;
      const constant = CONSTANTS[lower];
      if (constant !== undefined) return () => constant;
      throw new ParseError(
        `Unknown name "${name}" — the only variable is t`,
        tok.at,
      );
    }

    if (tok.op === '(') {
      pos++;
      const inner = parseExpr();
      expect(')');
      return inner;
    }

    throw new ParseError(`Unexpected "${tok.op}"`, tok.at);
  };

  const root = parseExpr();
  const trailing = peek();
  if (trailing) {
    throw new ParseError(
      `Unexpected "${trailing.kind === 'op' ? trailing.op : 'input'}" after the expression`,
      trailing.at,
    );
  }
  return root;
}

export type CompiledExpr =
  | { ok: true; fn: (t: number) => number }
  | { ok: false; error: string; at: number };

/**
 * Compile an expression in `t` to a closure.
 *
 * Never throws: a bad expression comes back as `{ ok: false }` with a message and the
 * character offset, so the UI can point at the mistake and the run simply does not start.
 */
export function compileExpr(source: string): CompiledExpr {
  const trimmed = source.trim();
  if (trimmed === '') return { ok: false, error: 'Expression is empty', at: 0 };
  try {
    const fn = parseTokens(tokenize(trimmed), trimmed);
    // Evaluate once so a malformed-but-parseable expression is caught here rather than
    // producing NaN halfway through a run.
    const probe = fn(0);
    if (typeof probe !== 'number') {
      return { ok: false, error: 'Expression did not produce a number', at: 0 };
    }
    return { ok: true, fn };
  } catch (err) {
    if (err instanceof ParseError) return { ok: false, error: err.message, at: err.at };
    return { ok: false, error: err instanceof Error ? err.message : 'Could not parse', at: 0 };
  }
}
