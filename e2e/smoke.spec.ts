import { expect, test } from '@playwright/test';

/**
 * Smoke coverage.
 *
 * The physics and the model layer are unit-tested, so what is left to prove here is that the
 * app boots, the WebGL canvas comes up, the controls move the numbers, and — the part only a
 * browser can check — that the worker actually computes a trajectory and the UI receives it.
 */

/** Fail a test on any console error, rather than letting a broken app quietly pass. */
async function openApp(page: import('@playwright/test').Page): Promise<string[]> {
  const errors: string[] = [];
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/');
  await expect(page.locator('canvas')).toBeVisible();
  return errors;
}

test('boots with the example model', async ({ page }) => {
  const errors = await openApp(page);
  await expect(page.getByRole('tab', { name: 'Bodies' })).toBeVisible();
  await expect(page.getByText('Upper Arm').first()).toBeVisible();
  await expect(page.getByText('Forearm').first()).toBeVisible();
  await expect(page.getByText('Ground').first()).toBeVisible();
  expect(errors).toEqual([]);
});

test('adds and edits a two-node spring-damper', async ({ page }) => {
  const errors = await openApp(page);
  await page.getByRole('tab', { name: 'Actuators' }).click();
  await page.getByRole('button', { name: 'Spring-dampers' }).click();
  await expect(page.getByText('No spring-dampers.')).toBeVisible();

  await page.getByRole('button', { name: 'Add a spring-damper' }).click();
  await expect(page.getByRole('combobox', { name: 'End A body' })).toHaveValue('ground');
  await expect(page.getByRole('combobox', { name: 'End B body' })).toHaveValue('upper');

  const stiffness = page.getByRole('textbox', { name: 'Stiffness' });
  await stiffness.fill('250');
  await stiffness.press('Enter');
  await expect(stiffness).toHaveValue('250');
  expect(errors).toEqual([]);
});

test('computes a trajectory in the worker and reports it', async ({ page }) => {
  const errors = await openApp(page);
  await page.getByRole('tab', { name: 'Run' }).click();

  // The frame counter only reaches its total once the worker has streamed everything back,
  // so this is a real end-to-end check of the worker round trip.
  await expect
    .poll(async () => page.locator('.status__chip').first().innerText(), { timeout: 30_000 })
    .toMatch(/^601 \/ 601 frames$/);

  await expect(page.getByText(/Energy ·/)).toBeVisible();
  expect(errors).toEqual([]);
});

test('draws the time-history plots with a legend and a unit', async ({ page }) => {
  await openApp(page);
  await page.getByRole('tab', { name: 'Run' }).click();
  await expect(page.locator('.plot').first()).toBeVisible({ timeout: 30_000 });

  expect(await page.locator('.plot').count()).toBeGreaterThanOrEqual(2);
  // Identity never rests on colour alone, and every chart names its unit.
  await expect(page.locator('.legend').first()).toBeVisible();
  await expect(page.locator('.plot__unit').first()).not.toBeEmpty();
});

test('editing a value recomputes the run', async ({ page }) => {
  await openApp(page);
  await page.getByRole('tab', { name: 'Run' }).click();
  await expect(page.locator('.plot').first()).toBeVisible({ timeout: 30_000 });

  await page.getByRole('tab', { name: 'Setup' }).click();
  const duration = page.getByRole('textbox', { name: 'Duration' });
  await duration.fill('4');
  await duration.press('Enter');

  await page.getByRole('tab', { name: 'Run' }).click();
  // 4 s at 60 Hz, inclusive of t = 0.
  await expect
    .poll(async () => page.locator('.status__chip').first().innerText(), { timeout: 30_000 })
    .toMatch(/^241 \/ 241 frames$/);
});

test('freeing all three rotations explains why springs stop applying', async ({ page }) => {
  await openApp(page);
  await page.getByRole('tab', { name: 'Hinges' }).click();
  await page.getByText('Elbow').first().click();

  // A joint with every rotation free is stored as a quaternion, which has no single angle
  // for a spring or a stop to work against — and the panel says so rather than ignoring it.
  for (const axis of ['rx', 'ry', 'rz']) {
    const toggle = page.getByRole('checkbox', { name: new RegExp(`^${axis}`) });
    if (!(await toggle.isChecked())) await toggle.check();
  }
  await expect(page.getByText(/stores its orientation as a quaternion/).first()).toBeVisible();
});

test('the diagnostics banner flags a non-physical inertia', async ({ page }) => {
  await openApp(page);
  await page.getByRole('tab', { name: 'Bodies' }).click();
  await page.getByText('Upper Arm').first().click();

  const izz = page.getByRole('textbox', { name: 'Izz', exact: true });
  await izz.fill('50');
  await izz.press('Enter');

  const banner = page.locator('.banner');
  await expect(banner).toBeVisible();
  await banner.locator('.banner__summary').click();
  await expect(page.getByText(/triangle inequality/i).first()).toBeVisible();
});

test('copies a share link that carries the whole model', async ({ page, context }) => {
  await openApp(page);
  await context.grantPermissions(['clipboard-read', 'clipboard-write']);
  await page.getByRole('tab', { name: 'Setup' }).click();
  await page.getByRole('button', { name: /Copy a link/ }).click();

  const link = await page.evaluate(() => navigator.clipboard.readText());
  expect(link).toContain('#m=');

  // Pasting into the tab that is already open. This is a *same-document* navigation — the
  // page does not reload — which is exactly the case a boot-time-only importer misses.
  await page.goto(link);
  await expect(page.getByText('Loaded a shared model.')).toBeVisible();
  // The hash is cleared, so a refresh cannot re-import over later edits.
  expect(new URL(page.url()).hash).toBe('');

  await page.getByRole('tab', { name: 'Bodies' }).click();
  await expect(page.getByText('Upper Arm').first()).toBeVisible();

  // And the ordinary case: a fresh page load carrying the hash.
  const fresh = await context.newPage();
  await fresh.goto(link);
  await expect(fresh.getByText('Loaded a shared model.')).toBeVisible();
  await expect(fresh.getByText('Upper Arm').first()).toBeVisible();
  await expect(fresh.getByText('Forearm').first()).toBeVisible();
  expect(new URL(fresh.url()).hash).toBe('');
  await fresh.close();
});

test('the timestep warning offers a fix that resolves it', async ({ page }) => {
  await openApp(page);
  await page.getByRole('tab', { name: 'Hinges' }).click();
  await page.getByText('Elbow').first().click();

  // A very stiff travel stop needs a smaller step than the default, and the diagnostics say
  // exactly which step — with a button to apply it.
  await page.getByRole('checkbox', { name: 'Travel limits' }).check();
  const stiffness = page.getByRole('textbox', { name: 'Stop stiffness' });
  await stiffness.fill('5000000');
  await stiffness.press('Enter');

  const banner = page.locator('.banner');
  await expect(banner).toBeVisible();
  await banner.locator('.banner__summary').click();
  const fix = page.getByRole('button', { name: /Use dt =/ });
  await expect(fix).toBeVisible();
  await fix.click();
  await expect(page.getByText(/Timestep is too large/)).toHaveCount(0);
});

test('static friction holds a loaded joint completely still', async ({ page }) => {
  await openApp(page);

  // Silence the driving actuator so gravity alone loads the joints.
  await page.getByRole('tab', { name: 'Actuators' }).click();
  await page.getByRole('button', { name: 'Disable' }).first().click();

  const forearmPositionAt = async (time: string) => {
    await page.getByRole('tab', { name: 'Run' }).click();
    await expect(page.locator('.plot').first()).toBeVisible({ timeout: 30_000 });
    await page.getByRole('combobox', { name: 'Body' }).selectOption({ label: 'Forearm' });
    await page.locator('.scrubber__track').fill(time);
    await expect(page.locator('.scrubber__time')).toContainText(time === '0' ? '0.00' : '10.00');
    return (await page.locator('.readout__block').first().locator('.value__num').allInnerTexts()).join();
  };

  const startsAt = await forearmPositionAt('0');
  const freeEnd = await forearmPositionAt('10');
  // Under gravity alone the arm swings; it must not still be where it started.
  expect(freeEnd).not.toBe(startsAt);

  await page.getByRole('tab', { name: 'Hinges' }).click();
  for (const hinge of ['Shoulder', 'Elbow']) {
    await page.getByText(hinge, { exact: true }).first().click();
    const stiction = page.getByRole('textbox', { name: 'Stiction' });
    await stiction.fill('500');
    await stiction.press('Enter');
  }

  // With a breakaway force far above the gravity load, nothing moves at all — not "barely",
  // exactly. A stuck axis is dropped from the system rather than held by a spring.
  expect(await forearmPositionAt('10')).toBe(startsAt);
});
