/* thermal.js - decoding and drawing for the Thermal Master P1 feed.
 *
 * The node ships a colour-mapped PNG rather than raw temperatures, because a
 * 160x120 float32 field is ~102 KB of base64 per frame and rosbridge would be
 * carrying 800 KB/s of it. The exact display range travels INSIDE the PNG as
 * tEXt, so the browser can INVERT the colour map and recover a temperature
 * for any pixel - which is what makes a spot readout possible without
 * shipping the field. Pairing the image with a separate stats topic instead
 * would be off by up to a frame; Float32MultiArray has no header to align on.
 */

/** Parse the fpms_* tEXt chunks. Reads the PNG bytes directly - an <img>
 *  exposes none of this. */
export function readPngText(bytes) {
  const out = {};
  if (bytes.length < 8) return out;
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let i = 8;
  while (i + 8 <= bytes.length) {
    const len = dv.getUint32(i);
    const type = String.fromCharCode(bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]);
    if (type === 'tEXt') {
      let z = i + 8;
      const end = z + len;
      let key = '';
      while (z < end && bytes[z] !== 0) key += String.fromCharCode(bytes[z++]);
      z++;
      let val = '';
      while (z < end) val += String.fromCharCode(bytes[z++]);
      out[key] = val;
    }
    if (type === 'IEND') break;
    i += 12 + len;
  }
  return out;
}

/* The SAME ramps the node uses. Kept in step deliberately: the inverse lookup
 * below is only exact because the palettes match byte for byte. */
function ironbow() {
  const l = new Uint8Array(256 * 3);
  for (let i = 0; i < 256; i++) {
    const x = i / 255;
    l[i * 3] = Math.round(Math.min(1, x * 3) * 255);
    l[i * 3 + 1] = Math.round(Math.min(1, Math.max(0, x * 3 - 1)) * 255);
    l[i * 3 + 2] = Math.round(Math.min(1, Math.max(0, x * 3 - 2)) * 255);
  }
  return l;
}
function greyRamp() {
  const l = new Uint8Array(256 * 3);
  for (let i = 0; i < 256; i++) { l[i * 3] = l[i * 3 + 1] = l[i * 3 + 2] = i; }
  return l;
}

export const PALETTES = { ironbow: ironbow(), grey: greyRamp() };

/** packed RGB -> palette index, so a pixel can be turned back into a value. */
function buildInverse(lut) {
  const m = new Map();
  for (let i = 0; i < 256; i++) {
    m.set((lut[i * 3] << 16) | (lut[i * 3 + 1] << 8) | lut[i * 3 + 2], i);
  }
  return m;
}
const INVERSE = { ironbow: buildInverse(PALETTES.ironbow), grey: buildInverse(PALETTES.grey) };

/**
 * Temperature at a source pixel, recovered by inverting the colour map.
 * Returns null when the colour is not in the palette (should not happen, but
 * guessing a temperature from an unknown colour would be worse than saying
 * nothing).
 */
export function tempAt(imgData, x, y, meta, palette) {
  if (!imgData || !meta) return null;
  const w = imgData.width, h = imgData.height;
  if (x < 0 || y < 0 || x >= w || y >= h) return null;
  const o = (y * w + x) * 4;
  const key = (imgData.data[o] << 16) | (imgData.data[o + 1] << 8) | imgData.data[o + 2];
  const inv = INVERSE[palette] || INVERSE.ironbow;
  let idx = inv.get(key);
  if (idx === undefined) {
    // Nearest match, in case the PNG round-trip shifted a channel by one.
    const lut = PALETTES[palette] || PALETTES.ironbow;
    let best = 1e9;
    const r = imgData.data[o], g = imgData.data[o + 1], b = imgData.data[o + 2];
    for (let i = 0; i < 256; i++) {
      const d = (lut[i * 3] - r) ** 2 + (lut[i * 3 + 1] - g) ** 2 + (lut[i * 3 + 2] - b) ** 2;
      if (d < best) { best = d; idx = i; }
    }
    if (best > 3000) return null;
  }
  const lo = parseFloat(meta.fpms_lo_c), hi = parseFloat(meta.fpms_hi_c);
  if (!isFinite(lo) || !isFinite(hi)) return null;
  return lo + (idx / 255) * (hi - lo);
}

/** Draw the colour ramp into a canvas, low on the left. */
export function drawColorbar(cv, palette) {
  const ctx = cv.getContext('2d');
  const w = cv.width, h = cv.height;
  const lut = PALETTES[palette] || PALETTES.ironbow;
  const img = ctx.createImageData(w, h);
  for (let x = 0; x < w; x++) {
    const i = Math.min(255, Math.round((x / Math.max(1, w - 1)) * 255));
    for (let y = 0; y < h; y++) {
      const o = (y * w + x) * 4;
      img.data[o] = lut[i * 3];
      img.data[o + 1] = lut[i * 3 + 1];
      img.data[o + 2] = lut[i * 3 + 2];
      img.data[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
}
