import { defineConfig } from 'vitest/config';

// The math layer is pure functions over three.js primitives, so it needs no DOM.
export default defineConfig({
  test: {
    environment: 'node',
    include: ['src/**/*.test.ts'],
    // Several physics checks intentionally integrate hundreds of thousands of RK stages.
    // Keep a per-test timeout, but leave enough headroom for shared/loaded CI runners; the
    // dedicated benchmark retains the actual solver throughput ceiling.
    testTimeout: 15_000,
  },
});
