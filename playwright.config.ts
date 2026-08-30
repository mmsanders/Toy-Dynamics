import { defineConfig, devices } from '@playwright/test';

/**
 * Smoke coverage only — the maths is unit-tested, so what's left to prove is that the app
 * boots, the WebGL canvas comes up, and the controls move the numbers.
 *
 * Chromium is preinstalled in this environment at PLAYWRIGHT_BROWSERS_PATH; `npx playwright
 * install` is neither needed nor wanted.
 */
/**
 * Use the Chromium already on the machine rather than downloading one: this environment
 * ships a build that may not match the pinned Playwright's expected revision, and
 * `playwright install` is blocked here. Unset the env var and Playwright picks its own.
 */
const CHROMIUM = process.env.PLAYWRIGHT_BROWSERS_PATH
  ? `${process.env.PLAYWRIGHT_BROWSERS_PATH}/chromium`
  : undefined;

export default defineConfig({
  testDir: './e2e',
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 1 : 0,
  reporter: process.env.CI ? 'line' : 'list',
  use: {
    baseURL: 'http://127.0.0.1:5173',
    trace: 'retain-on-failure',
  },
  projects: [
    {
      name: 'desktop',
      use: { ...devices['Desktop Chrome'], launchOptions: { executablePath: CHROMIUM } },
    },
    {
      // A real phone profile, so the sheet layout is what actually gets exercised. The
      // iPhone preset defaults to WebKit, which isn't available here, so it runs on
      // Chromium with the phone's viewport, DPR and touch flags.
      name: 'mobile',
      use: {
        ...devices['iPhone 13'],
        browserName: 'chromium',
        launchOptions: { executablePath: CHROMIUM },
      },
    },
  ],
  webServer: {
    command: 'npm run dev -- --port 5173 --strictPort',
    url: 'http://127.0.0.1:5173',
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
});
