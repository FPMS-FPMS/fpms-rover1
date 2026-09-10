/* panel_cameras - CAMERAS panel: thermal (rover) and regular (ground station).
 *
 *   thermal   Thermal Master P1 on the rover. No kernel driver binds it
 *             (vendor USB class 255/240, not UVC), so a userspace libusb
 *             driver reads it and thermal_node publishes it. Frames arrive
 *             over rosbridge as a colour-mapped PNG carrying its own display
 *             range in tEXt, which is what lets this panel report a
 *             temperature for any pixel without shipping the float field.
 *
 *   regular   the LAPTOP's built-in webcam. Rover 1 has NO camera fitted -
 *             its only video nodes are the RK3588 hardware codecs. The panel
 *             says so rather than letting the obvious assumption stand.
 *
 * The thermal view is a canvas, not an <img>, because it has to be readable:
 * hover gives a spot temperature, the hot and cold extremes are marked, and a
 * colour bar states what the colours mean. A picture of heat with no numbers
 * on it is decoration.
 *
 * mount(root, api) -> { update(state), destroy() }
 */
import { connect, release, subscribe } from './ros.js';
import { readPngText, tempAt, drawColorbar } from './thermal.js';

const PANEL = 'cameras';
const FRAME_MS = 320;          // ~3 fps for the webcam; its source is 5.

const CSS = `
.fcam{display:grid;gap:var(--s3,14px);grid-template-columns:repeat(auto-fit,minmax(340px,1fr))}
.fcam .wide{grid-column:1/-1}
.fcam .feed{position:relative;background:#000;border:1px solid var(--line);
  border-radius:10px;overflow:hidden;aspect-ratio:4/3;display:grid;place-items:center}
.fcam .feed img,.fcam .feed canvas.main{width:100%;height:100%;object-fit:contain;display:block}
.fcam .badge{position:absolute;left:10px;top:10px;font:500 11px/1.4 ui-monospace,monospace;
  background:rgba(0,0,0,.55);padding:4px 8px;border-radius:6px;color:var(--muted);
  pointer-events:none}
.fcam .badge.live{color:var(--ok)} .fcam .badge.bad{color:var(--bad)}
.fcam .spot{position:absolute;right:10px;bottom:10px;font:600 13px/1.4 ui-monospace,monospace;
  background:rgba(0,0,0,.62);padding:5px 9px;border-radius:6px;color:var(--text);
  pointer-events:none}
.fcam .empty{padding:22px;text-align:center;color:var(--muted);max-width:44ch}
.fcam .empty .hd{color:var(--warn);font:600 13px/1.4 system-ui;letter-spacing:.06em;
  text-transform:uppercase;margin-bottom:8px}
.fcam .empty .bd{font-size:12px;line-height:1.65}
.fcam .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;align-items:baseline;
  margin-top:10px}
.fcam .kv dt{color:var(--muted);font-size:12px}
.fcam .kv dd{margin:0;color:var(--text);font-variant-numeric:tabular-nums;font-size:12px}
.fcam .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:10px}
.fcam .cbar{display:grid;grid-template-columns:auto 1fr auto;gap:8px;align-items:center;
  margin-top:10px;font:500 11px/1 ui-monospace,monospace;color:var(--muted)}
.fcam .cbar canvas{width:100%;height:12px;border-radius:3px;border:1px solid var(--line);
  display:block}
.fcam .tog{font:500 11px/1 system-ui;padding:5px 9px;border-radius:6px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--muted)}
.fcam .tog.on{color:var(--accent);border-color:var(--accent)}
`;

function h(tag, attrs = {}, kids = []) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'text') e.textContent = v;
    else if (k === 'class') e.className = v;
    else e.setAttribute(k, v);
  }
  for (const c of [].concat(kids)) if (c) e.append(c);
  return e;
}
const DASH = '—';

export function mount(root, api = {}) {
  // Declared up front: subscription callbacks can fire before mount() returns.
  let dead = false, paused = false, timer = null, busy = false;
  let frames = 0, started = false;
  let rosUrl = null, twired = false, onStatus = null, tUnsubs = [];
  let tAt = 0, tFrames = 0, tStats = null;
  let meta = null, palette = 'ironbow';
  let srcData = null;            // 160x120 ImageData, for the spot readout
  let smooth = false;
  let hoverX = -1, hoverY = -1;

  const el = h('div', { class: 'fcam' });
  root.replaceChildren(h('style', { text: CSS }), el);

  // ---- thermal ---------------------------------------------------------
  const tCanvas = h('canvas', { class: 'main' });
  const tBadge = h('div', { class: 'badge bad', text: 'connecting' });
  const tSpot = h('div', { class: 'spot', text: '' });
  const tBody = h('div', { class: 'empty' });
  const tFeed = h('div', { class: 'feed' }, [tCanvas, tBody, tBadge, tSpot]);
  const cbLo = h('span', { text: DASH });
  const cbHi = h('span', { text: DASH });
  const cbCv = h('canvas', { width: '256', height: '12' });
  const cbar = h('div', { class: 'cbar' }, [cbLo, cbCv, cbHi]);
  const tKv = h('dl', { class: 'kv' });
  const btnSmooth = h('button', { class: 'tog', type: 'button', text: 'Smoothing' });
  const tCard = h('div', { class: 'card' }, [
    h('h2', { text: 'Thermal — rover' }), tFeed, cbar, tKv,
    h('div', { class: 'row' }, [btnSmooth]),
  ]);

  // ---- regular ---------------------------------------------------------
  const img = h('img', { alt: 'live camera frame' });
  const cBadge = h('div', { class: 'badge', text: 'starting…' });
  const cEmpty = h('div', { class: 'empty' });
  const cFeed = h('div', { class: 'feed' }, [img, cBadge]);
  const cKv = h('dl', { class: 'kv' });
  const btnPause = h('button', { class: 'btn', type: 'button', text: 'Pause' });
  const cCard = h('div', { class: 'card' }, [
    h('h2', { text: 'Regular — ground station' }), cFeed, cKv,
    h('div', { class: 'row' }, [btnPause]),
  ]);

  const note = h('div', { class: 'card wide note', 'data-detail': 'full' });
  el.append(tCard, cCard, note);

  drawColorbar(cbCv, palette);
  btnSmooth.addEventListener('click', () => {
    smooth = !smooth;
    btnSmooth.className = 'tog' + (smooth ? ' on' : '');
    redraw();
  });

  function kv(dl, pairs) {
    dl.replaceChildren();
    for (const [k, v] of pairs) dl.append(h('dt', { text: k }), h('dd', { text: v }));
  }
  const tFresh = () => (Date.now() - tAt) < 3000;

  // ---- thermal rendering ----------------------------------------------
  const off = document.createElement('canvas');   // holds the 160x120 source

  function redraw() {
    if (!srcData) return;
    const sw = srcData.width, sh = srcData.height;
    const dpr = window.devicePixelRatio || 1;
    const r = tFeed.getBoundingClientRect();
    const W = Math.max(1, Math.round(r.width * dpr));
    const H = Math.max(1, Math.round(r.height * dpr));
    if (tCanvas.width !== W || tCanvas.height !== H) { tCanvas.width = W; tCanvas.height = H; }
    const ctx = tCanvas.getContext('2d');
    ctx.clearRect(0, 0, W, H);
    // Nearest-neighbour by default. Smoothing invents gradients the sensor did
    // not measure, which on a thermal image reads as real structure - so it is
    // opt-in and labelled, not the default.
    ctx.imageSmoothingEnabled = smooth;

    const scale = Math.min(W / sw, H / sh);
    const dw = sw * scale, dh = sh * scale;
    const ox = (W - dw) / 2, oy = (H - dh) / 2;
    ctx.drawImage(off, 0, 0, sw, sh, ox, oy, dw, dh);

    // Mark the extremes. Their coordinates come from the node, computed on
    // the real float field - not guessed from the picture.
    const mark = (xy, color, label) => {
      if (!xy) return;
      const p = xy.split(',').map(Number);
      if (p.length !== 2 || !isFinite(p[0])) return;
      const cx = ox + (p[0] + 0.5) * scale, cy = oy + (p[1] + 0.5) * scale;
      const rad = Math.max(5, W / 90);
      ctx.strokeStyle = color; ctx.lineWidth = Math.max(1.5, W / 500);
      ctx.beginPath(); ctx.arc(cx, cy, rad, 0, Math.PI * 2); ctx.stroke();

      const fs = Math.max(10, Math.round(W / 55));
      ctx.font = `${fs}px ui-monospace, monospace`;
      const tw = ctx.measureText(label).width;
      const pad = Math.max(8, W / 70);
      // Flip the label to whichever side has room, and keep it inside the
      // canvas. A hotspot at the frame edge is common - the coldest pixel is
      // very often the corner - and a label clipped in half is worse than none.
      let lx = cx + pad;
      if (lx + tw > W - 4) lx = cx - pad - tw;
      lx = Math.max(4, Math.min(lx, W - tw - 4));
      let ly = cy - Math.max(6, W / 110);
      ly = Math.max(fs + 2, Math.min(ly, H - 4));
      // A dark plate under the text, so a label over a white-hot region stays
      // readable in either palette.
      ctx.fillStyle = 'rgba(0,0,0,.55)';
      ctx.fillRect(lx - 3, ly - fs, tw + 6, fs + 4);
      ctx.fillStyle = color;
      ctx.fillText(label, lx, ly);
    };
    if (meta) {
      mark(meta.fpms_max_xy, '#ff5c8a', parseFloat(meta.fpms_max_c).toFixed(1) + '°');
      mark(meta.fpms_min_xy, '#62a8ff', parseFloat(meta.fpms_min_c).toFixed(1) + '°');
    }
    // Crosshair follows the pointer so the spot readout is unambiguous.
    if (hoverX >= 0 && hoverY >= 0) {
      const cx = ox + (hoverX + 0.5) * scale, cy = oy + (hoverY + 0.5) * scale;
      ctx.strokeStyle = 'rgba(255,255,255,.85)'; ctx.lineWidth = Math.max(1, W / 700);
      ctx.beginPath();
      ctx.moveTo(cx - 10, cy); ctx.lineTo(cx + 10, cy);
      ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10);
      ctx.stroke();
    }
  }

  function feedToSource(ev) {
    if (!srcData) return null;
    const r = tCanvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const W = tCanvas.width, H = tCanvas.height;
    const sw = srcData.width, sh = srcData.height;
    const scale = Math.min(W / sw, H / sh);
    const ox = (W - sw * scale) / 2, oy = (H - sh * scale) / 2;
    const px = (ev.clientX - r.left) * (W / r.width);
    const py = (ev.clientY - r.top) * (H / r.height);
    return [Math.floor((px - ox) / scale), Math.floor((py - oy) / scale)];
  }

  tCanvas.addEventListener('mousemove', (ev) => {
    const p = feedToSource(ev);
    if (!p) return;
    hoverX = p[0]; hoverY = p[1];
    const t = tempAt(srcData, hoverX, hoverY, meta, palette);
    tSpot.textContent = t == null ? '' : `${t.toFixed(1)} °C`;
    redraw();
  });
  tCanvas.addEventListener('mouseleave', () => {
    hoverX = hoverY = -1; tSpot.textContent = ''; redraw();
  });

  function b64ToBytes(b64) {
    const bin = atob(b64);
    const u8 = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    return u8;
  }

  async function onThermalFrame(m) {
    if (dead || !m.data) return;
    const bytes = b64ToBytes(m.data);
    meta = readPngText(bytes);
    if (meta.fpms_palette) {
      if (meta.fpms_palette !== palette) { palette = meta.fpms_palette; drawColorbar(cbCv, palette); }
    }
    const blob = new Blob([bytes], { type: 'image/' + (m.format || 'png') });
    let bmp;
    try { bmp = await createImageBitmap(blob); } catch (e) { return; }
    if (dead) { bmp.close && bmp.close(); return; }
    off.width = bmp.width; off.height = bmp.height;
    const octx = off.getContext('2d', { willReadFrequently: true });
    octx.imageSmoothingEnabled = false;
    octx.drawImage(bmp, 0, 0);
    bmp.close && bmp.close();
    srcData = octx.getImageData(0, 0, off.width, off.height);
    tAt = Date.now(); tFrames++;
    tBody.style.display = 'none';
    redraw();
    paintThermal();
  }

  function paintThermal() {
    const live = tFresh();
    tBadge.className = 'badge ' + (live ? 'live' : 'bad');
    if (live && meta) {
      tBadge.textContent = `live  ${parseFloat(meta.fpms_min_c).toFixed(1)}–`
        + `${parseFloat(meta.fpms_max_c).toFixed(1)} °C`;
      cbLo.textContent = parseFloat(meta.fpms_lo_c).toFixed(1) + '°';
      cbHi.textContent = parseFloat(meta.fpms_hi_c).toFixed(1) + '°';
    } else {
      tBadge.textContent = live ? 'live' : 'no frames';
    }
  }

  async function wireThermal(url, t) {
    rosUrl = url;
    onStatus = (ok) => { if (!ok && !dead) paintThermal(); };
    try {
      const { ROSLIB, ros } = await connect(url, onStatus);
      if (dead) { release(url, onStatus); return; }
      tUnsubs.push(subscribe(ROSLIB, ros, t.image_topic, t.image_type, onThermalFrame));
      tUnsubs.push(subscribe(ROSLIB, ros, t.stats_topic, t.stats_type, (m) => {
        if (dead || !m.data) return;
        tStats = m.data; tAt = Date.now();
      }));
    } catch (e) {
      console.error('cameras: roslib', e);
    }
  }

  // ---- regular camera --------------------------------------------------
  btnPause.addEventListener('click', async () => {
    paused = !paused;
    btnPause.textContent = paused ? 'Resume' : 'Pause';
    if (paused) {
      cBadge.className = 'badge'; cBadge.textContent = 'paused';
      try { if (api.call) await api.call(PANEL, 'stop', {}); } catch (e) { /* best effort */ }
    }
  });

  async function grab() {
    if (dead || paused || busy || !api.call) return;
    busy = true;
    try {
      const r = await api.call(PANEL, 'frame', {});
      if (dead) return;
      if (r && r.b64) {
        img.src = 'data:image/jpeg;base64,' + r.b64;
        img.style.display = '';
        cEmpty.remove();
        frames++; started = true;
        cBadge.className = 'badge live';
        cBadge.textContent = `live  ${(r.bytes / 1024).toFixed(0)} KB`;
      }
    } catch (e) {
      if (dead) return;
      cBadge.className = 'badge bad'; cBadge.textContent = 'no frame';
      if (!started) {
        img.style.display = 'none';
        if (!cEmpty.isConnected) cFeed.append(cEmpty);
        cEmpty.replaceChildren(h('div', { class: 'hd', text: 'no frame' }),
                               h('div', { class: 'bd', text: (e && e.message) || String(e) }));
      }
    } finally { busy = false; }
  }

  // ---- state -----------------------------------------------------------
  function update(full) {
    const st = ((full || {}).panels || {})[PANEL];
    if (!st) return;
    const t = st.thermal || {};
    if (!twired && t.usable && t.rosbridge && t.image_topic) {
      twired = true;
      wireThermal(t.rosbridge, t);
    }
    if (!tFresh()) {
      tBody.style.display = '';
      tBody.replaceChildren(
        h('div', { class: 'hd', text: t.usable ? 'waiting for frames' : 'unavailable' }),
        h('div', { class: 'bd', text: t.usable
          ? ('subscribing to ' + (t.image_topic || '') + ' on ' + (t.rosbridge || ''))
          : (t.note || 'no driver binds this device') }));
    }
    paintThermal();
    kv(tKv, [
      ['device', t.device || 'Thermal Master P1'],
      ['where', t.host || DASH],
      ['size', t.size ? t.size.join(' × ') : DASH],
      ['transport', t.usable ? 'ROS 2 → rosbridge' : DASH],
      ['frames shown', String(tFrames)],
      ['scene range', meta ? `${parseFloat(meta.fpms_min_c).toFixed(1)} – `
        + `${parseFloat(meta.fpms_max_c).toFixed(1)} °C` : DASH],
      ['display range', meta ? `${parseFloat(meta.fpms_lo_c).toFixed(1)} – `
        + `${parseFloat(meta.fpms_hi_c).toFixed(1)} °C` : DASH],
      ['centre', tStats && tStats.length > 3 ? `${tStats[3].toFixed(1)} °C` : DASH],
    ]);

    const c = st.regular || {};
    const cap = c.capture || {};
    kv(cKv, [
      ['device', c.device || DASH],
      ['where', c.where || DASH],
      ['size', c.size ? c.size.join(' × ') : DASH],
      ['capture', cap.running ? 'running' : (paused ? 'paused' : 'idle')],
      ['source fps', cap.fps != null ? String(cap.fps) : DASH],
      ['frames shown', String(frames)],
    ]);

    if (!note.dataset.filled && c.note) {
      note.dataset.filled = '1';
      note.replaceChildren(h('div', { class: 'small', text: c.note
        + ' The webcam is opened only while this tab is open and released when '
        + 'you leave it, because /dev/video0 is exclusive. On the thermal view, '
        + 'hover for a spot temperature: the display range travels inside each '
        + 'PNG, so the colour map can be inverted exactly. Smoothing is off by '
        + 'default - interpolation invents detail the 160x120 sensor never '
        + 'measured.' }));
    }
  }

  const onResize = () => redraw();
  window.addEventListener('resize', onResize);
  timer = setInterval(grab, FRAME_MS);
  grab();
  if (api && api.state) update(api.state);

  return {
    update,
    destroy() {
      dead = true;
      clearInterval(timer);
      window.removeEventListener('resize', onResize);
      for (const u of tUnsubs) { try { u(); } catch (e) { /* gone */ } }
      tUnsubs = [];
      if (rosUrl) { release(rosUrl, onStatus); rosUrl = null; }
      try { if (api.call) api.call(PANEL, 'stop', {}); } catch (e) { /* best effort */ }
    },
  };
}
