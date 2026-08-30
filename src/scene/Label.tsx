import { useEffect } from 'react';
import * as THREE from 'three';

/**
 * Text in the 3D scene, drawn as a canvas-texture sprite.
 *
 * Deliberately not drei's <Text>: that pulls a font over the network at runtime, which
 * means labels can silently fail to appear on a slow or offline connection. A canvas
 * texture is self-contained, uses the same system font stack as the rest of the UI, and
 * sprites billboard toward the camera for free.
 *
 * The canvas is sized from the *measured* text rather than fixed — a fixed square clips
 * anything longer than a character or two — and the measured aspect ratio is used to scale
 * the sprite without stretching the glyphs.
 *
 * ## Why the cache is reference-counted
 *
 * Renaming a frame fires once per keystroke, so a plain unevicted cache leaks a GPU
 * texture per character typed. But naive LRU eviction is worse than the leak: a mounted
 * sprite keeps drawing with whichever texture it was handed, so evicting by age alone can
 * dispose one that is still on screen and blank the label.
 *
 * So mounts and unmounts maintain a live-reference count per key, and the sweep only ever
 * disposes entries that are over the cap *and* unreferenced. That bounds memory without
 * ever pulling a texture out from under a live sprite, and it keeps the shared X/Y/Z axis
 * labels — identical text and colour across every frame — down to one texture each.
 */

type Entry = { texture: THREE.Texture; aspect: number };

/** Enough for a busy scene many times over; the churn this bounds is renaming. */
const MAX_ENTRIES = 48;

const cache = new Map<string, Entry>();
/** Live mount count per cache key, kept beside the cache rather than inside the entry so
 *  the mount effect mutates module state instead of a value produced during render. */
const liveRefs = new Map<string, number>();
/** Keys that have completed at least one mount. Entries are inserted during render but
 *  only retained in the effect, so without this a sweep in that window could dispose a
 *  texture a sprite has already been handed. */
const everMounted = new Set<string>();

const FONT_PX = 64;
const FONT_STACK =
  'ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif';

function buildEntry(text: string, color: string): Entry {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  const font = `bold ${FONT_PX}px ${FONT_STACK}`;

  // Measure first, then size the canvas to fit, with room for the outline stroke.
  let width = FONT_PX;
  if (ctx) {
    ctx.font = font;
    width = Math.ceil(ctx.measureText(text).width);
  }
  const padding = Math.round(FONT_PX * 0.28);
  canvas.width = Math.max(FONT_PX, width + padding * 2);
  canvas.height = Math.round(FONT_PX * 1.5);

  if (ctx) {
    // Resizing the canvas resets the context, so restate everything after.
    ctx.font = font;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.lineJoin = 'round';
    // Dark outline first, so labels stay readable over the grid and over each other.
    ctx.lineWidth = 10;
    ctx.strokeStyle = 'rgba(4, 6, 12, 0.92)';
    ctx.strokeText(text, canvas.width / 2, canvas.height / 2);
    ctx.fillStyle = color;
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.needsUpdate = true;

  return { texture, aspect: canvas.width / canvas.height };
}

/**
 * Populating the cache during render is a deliberate memo-style side effect: it is
 * idempotent, and a render that never commits leaves at most one unreferenced entry,
 * which the sweep reclaims.
 */
function acquire(key: string, text: string, color: string): Entry {
  const existing = cache.get(key);
  if (existing) return existing;
  const entry = buildEntry(text, color);
  cache.set(key, entry);
  return entry;
}

/** Dispose unreferenced entries, oldest first, until back under the cap. */
function sweep(): void {
  if (cache.size <= MAX_ENTRIES) return;
  for (const [key, entry] of cache) {
    if (cache.size <= MAX_ENTRIES) break;
    if ((liveRefs.get(key) ?? 0) > 0) continue;
    // Created but not yet mounted: still in flight, so not ours to dispose.
    if (!everMounted.has(key)) continue;
    cache.delete(key);
    everMounted.delete(key);
    entry.texture.dispose();
  }
}

function retain(key: string): void {
  liveRefs.set(key, (liveRefs.get(key) ?? 0) + 1);
  everMounted.add(key);
}

function release(key: string): void {
  const remaining = (liveRefs.get(key) ?? 1) - 1;
  if (remaining > 0) liveRefs.set(key, remaining);
  else liveRefs.delete(key);
  sweep();
}

type Props = {
  text: string;
  color: string;
  position: [number, number, number];
  /** Cap height of the label in world units; width follows from the text. */
  scale?: number;
  opacity?: number;
};

export function Label({ text, color, position, scale = 0.34, opacity = 1 }: Props) {
  const key = `${text}|${color}`;
  const entry = acquire(key, text, color);

  useEffect(() => {
    retain(key);
    return () => release(key);
  }, [key]);

  return (
    <sprite position={position} scale={[scale * entry.aspect, scale, 1]}>
      <spriteMaterial
        map={entry.texture}
        transparent
        opacity={opacity}
        depthTest={false}
        depthWrite={false}
        sizeAttenuation
      />
    </sprite>
  );
}
