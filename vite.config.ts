import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Deployed to GitHub Pages at https://<user>.github.io/Toy-Dynamics/, so built assets must
// resolve under that sub-path. The dev server stays at '/' for convenience.
export default defineConfig(({ command }) => ({
  base: command === 'build' ? '/Toy-Dynamics/' : '/',
  plugins: [react()],
  server: { host: true },
  worker: { format: 'es' },
}));
