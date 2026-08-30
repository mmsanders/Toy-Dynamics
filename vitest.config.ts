import { defineConfig } from 'vitest/config';

// The math layer is pure functions over three.js primitives, so it needs no DOM.
export default defineConfig({
  test: {
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
});
