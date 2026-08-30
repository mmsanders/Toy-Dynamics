import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { App } from './App';
import { importModelFromHash } from './share/importOnBoot';
import './styles.css';
import './ui/ui.css';
import './ui/panels.css';

// Before the first render, so a shared model is what actually paints rather than a flash
// of the previous one.
importModelFromHash();

const container = document.getElementById('root');
if (!container) throw new Error('Missing #root element');

createRoot(container).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
