import { describe, expect, it } from 'vitest';
import { compileExpr } from './expr';

const evaluate = (source: string, t = 0): number => {
  const result = compileExpr(source);
  if (!result.ok) throw new Error(`${source} failed to compile: ${result.error}`);
  return result.fn(t);
};

const failure = (source: string): { error: string; at: number } => {
  const result = compileExpr(source);
  if (result.ok) throw new Error(`${source} unexpectedly compiled`);
  return { error: result.error, at: result.at };
};

describe('expression parser', () => {
  it('evaluates arithmetic with the usual precedence', () => {
    expect(evaluate('1 + 2 * 3')).toBe(7);
    expect(evaluate('(1 + 2) * 3')).toBe(9);
    expect(evaluate('10 - 4 - 3')).toBe(3); // left-associative
    expect(evaluate('12 / 4 / 3')).toBe(1);
    expect(evaluate('7 % 4')).toBe(3);
  });

  it('treats exponentiation as right-associative', () => {
    // 2^(3^2) = 512, not (2^3)^2 = 64. The wrong answer here is the classic parser bug.
    expect(evaluate('2^3^2')).toBe(512);
  });

  it('handles unary minus, including before a power', () => {
    expect(evaluate('-5')).toBe(-5);
    expect(evaluate('--5')).toBe(5);
    expect(evaluate('3 * -2')).toBe(-6);
    expect(evaluate('-2^2')).toBe(-4); // negation applies to the power, as in maths
    expect(evaluate('2^-3')).toBe(0.125); // and a negative exponent still parses
  });

  it('exposes t as the only variable', () => {
    expect(evaluate('t * 2', 3)).toBe(6);
    expect(evaluate('t^2 + 1', 4)).toBe(17);
    expect(failure('x + 1').error).toMatch(/only variable is t/);
  });

  it('knows the usual constants', () => {
    expect(evaluate('pi')).toBeCloseTo(Math.PI, 12);
    expect(evaluate('e')).toBeCloseTo(Math.E, 12);
    expect(evaluate('tau')).toBeCloseTo(Math.PI * 2, 12);
  });

  it('does not mistake the constant e for exponent notation', () => {
    expect(evaluate('2e3')).toBe(2000);
    expect(evaluate('2*e')).toBeCloseTo(2 * Math.E, 12);
    expect(evaluate('1.5e-2')).toBeCloseTo(0.015, 12);
    expect(evaluate('2e3 + e')).toBeCloseTo(2000 + Math.E, 12);
  });

  it('calls maths functions', () => {
    expect(evaluate('sin(0)')).toBe(0);
    expect(evaluate('cos(0)')).toBe(1);
    expect(evaluate('sqrt(16)')).toBe(4);
    expect(evaluate('abs(-3)')).toBe(3);
    expect(evaluate('min(4, 2, 9)')).toBe(2);
    expect(evaluate('max(4, 2, 9)')).toBe(9);
    expect(evaluate('atan2(1, 1)')).toBeCloseTo(Math.PI / 4, 12);
  });

  it('provides the profile-shaping helpers', () => {
    expect(evaluate('step(t - 2)', 1)).toBe(0);
    expect(evaluate('step(t - 2)', 2)).toBe(1);
    expect(evaluate('step(t - 2)', 5)).toBe(1);

    expect(evaluate('pulse(t, 1, 3)', 0.5)).toBe(0);
    expect(evaluate('pulse(t, 1, 3)', 2)).toBe(1);
    expect(evaluate('pulse(t, 1, 3)', 3)).toBe(0);

    expect(evaluate('clamp(t, 0, 1)', 5)).toBe(1);
    expect(evaluate('clamp(t, 0, 1)', -5)).toBe(0);
  });

  it('composes into the kind of schedule an actuator actually needs', () => {
    // A 2 N burn from t=1 to t=3, then a sinusoidal dither.
    const src = '2*pulse(t, 1, 3) + 0.1*sin(10*t)*step(t - 3)';
    expect(evaluate(src, 0)).toBeCloseTo(0, 12);
    expect(evaluate(src, 2)).toBeCloseTo(2, 12);
    expect(evaluate(src, 4)).toBeCloseTo(0.1 * Math.sin(40), 12);
  });

  it('is case-insensitive for names', () => {
    expect(evaluate('SIN(0)')).toBe(0);
    expect(evaluate('PI')).toBeCloseTo(Math.PI, 12);
    expect(evaluate('T', 3)).toBe(3);
  });

  it('reports errors with a position instead of throwing', () => {
    expect(compileExpr('').ok).toBe(false);
    expect(compileExpr('   ').ok).toBe(false);

    expect(failure('1 +').error).toMatch(/end of expression/);
    expect(failure('(1 + 2').error).toMatch(/Expected "\)"/);
    expect(failure('1 2').error).toMatch(/after the expression/);
    expect(failure('nope(1)').error).toMatch(/Unknown function "nope"/);
    expect(failure('sin(1, 2)').error).toMatch(/takes 1 argument/);
    expect(failure('min(1)').error).toMatch(/takes 2 to 8 arguments/);

    const bad = failure('1 + $');
    expect(bad.error).toMatch(/Unexpected character/);
    expect(bad.at).toBe(4);
  });

  it('refuses to reach outside its own scope', () => {
    // No host globals are in scope — the parser only knows t, the constants and the
    // function table, which is the point of not using new Function.
    expect(compileExpr('Math.sin(1)').ok).toBe(false);
    expect(compileExpr('globalThis').ok).toBe(false);
    expect(compileExpr('window').ok).toBe(false);
    expect(compileExpr('constructor').ok).toBe(false);
  });

  it('compiles once and evaluates many times', () => {
    const result = compileExpr('sin(t) * 2');
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    for (let i = 0; i < 10; i++) {
      expect(result.fn(i)).toBeCloseTo(Math.sin(i) * 2, 12);
    }
  });
});
