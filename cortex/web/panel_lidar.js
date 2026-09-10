/* panel_lidar - LIDAR panel for Rover 1 (LDROBOT D500 (LD19 protocol)).
 *
 * The scan is fetched STRAIGHT FROM THE ROVER, not through the Cortex backend.
 * A 360-point scan is ~6 KB and the scanner turns at 10 Hz; pushing that
 * through panel state would put it in the SSE snapshot sent to every client on
 * every tick, which is the exact problem the 2026-09-02 rate change fixed (the
 * panels dict was 96% of a 211 KB frame). panel_lidar.py hands us the URL and
 * gets out of the way - the same reasoning panel_rover.js uses for rosbridge.
 *
 * Drawing is dirty-driven: one redraw per scan that actually arrives. There is
 * no requestAnimationFrame loop here, deliberately - a spinning rAF loop for
 * data that changes 5 times a second is what the 2026-09-04 trim removed.
 *
 * mount(root, api) -> { update(state), destroy() }
 */
import { connect, release, subscribe } from './ros.js';

const PANEL = 'lidar';

const FETCH_MS = 200;          // 5 Hz. The scanner is 10 Hz; this is plenty.
const RANGES = [1000, 2000, 4000, 8000, 12000];

const CSS = `
.flid{display:grid;gap:var(--s3,14px);grid-template-columns:minmax(320px,1.6fr) minmax(240px,1fr)}
@media(max-width:900px){.flid{grid-template-columns:1fr}}
.flid .wide{grid-column:1/-1}
.flid .plotwrap{position:relative;background:var(--panel2);border:1px solid var(--line);
  border-radius:10px;overflow:hidden;aspect-ratio:1/1}
.flid canvas{display:block;width:100%;height:100%}
.flid .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;align-items:baseline}
.flid .kv dt{color:var(--muted);font-size:12px}
.flid .kv dd{margin:0;font-variant-numeric:tabular-nums;color:var(--text)}
.flid .row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.flid .rbtn{font:500 11px/1 system-ui;padding:5px 9px;border-radius:6px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--muted)}
.flid .rbtn.on{color:var(--accent);border-color:var(--accent)}
.flid .stat{position:absolute;left:10px;top:10px;font:500 11px/1.5 ui-monospace,monospace;
  color:var(--muted);background:rgba(0,0,0,.45);padding:5px 8px;border-radius:6px;
  pointer-events:none}
.flid .bad{color:var(--bad)}
.flid .ok{color:var(--ok)}
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
const fmt = (v, unit = '', dp = 0) =>
  (v === null || v === undefined || Number.isNaN(v)) ? DASH
    : (typeof v === 'number' ? v.toFixed(dp) : String(v)) + unit;

export function mount(root, api = {}) {
  const el = h('div', { class: 'flid' });
  root.replaceChildren(h('style', { text: CSS }), el);

  const cv = h('canvas');
  const overlay = h('div', { class: 'stat', text: 'connecting…' });
  const plot = h('div', { class: 'plotwrap' }, [cv, overlay]);

  const kvEl = h('dl', { class: 'kv' });
  const rangeRow = h('div', { class: 'row' });
  const side = h('div', { class: 'card' }, [
    h('h2', { text: 'Scanner' }), kvEl,
    h('div', { class: 'small muted', style: 'margin:12px 0 6px', text: 'Range' }),
    rangeRow,
  ]);

  const info = h('div', { class: 'card wide note', 'data-detail': 'full' });
  el.append(plot, side, info);

  let maxRange = 4000;
  const rbtns = RANGES.map((r) => {
    const b = h('button', { class: 'rbtn', type: 'button', text: (r / 1000) + ' m' });
    b.addEventListener('click', () => { maxRange = r; paintButtons(); draw(); });
    rangeRow.append(b);
    return { r, b };
  });
  function paintButtons() {
    for (const x of rbtns) x.b.className = 'rbtn' + (x.r === maxRange ? ' on' : '');
  }
  paintButtons();

  // Every mutable declared up front: these are closed over by callbacks that
  // can fire before mount() finishes, and a `let` referenced above its own
  // declaration throws a ReferenceError that silently kills the panel.
  let dead = false;
  let scanUrl = null;
  let rosUrl = null, rosWired = false, unsub = null, onStatus = null;
  let stTopic = '/scan', stType = 'sensor_msgs/msg/LaserScan';
  let rosAt = 0;
  let source = 'connecting';
  // ROS is primary, HTTP is the fallback. Only one of them can own the
  // scanner at a time (the two systemd units on the rover declare Conflicts=
  // on each other), so this prefers rosbridge and drops to HTTP only when ROS
  // has actually gone quiet - not merely because it has not spoken yet.
  let ws = null, wsUrl = null, wired = false, retryTimer = null;
  let rosAt = 0;                 // last time a ROS scan arrived
  let source = 'connecting';
  let last = null;          // last good scan payload
  let lastErr = null;
  let timer = null;
  let inflight = null;

  // ---- drawing ---------------------------------------------------------
  const ctx = cv.getContext('2d');

  function sizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const r = cv.getBoundingClientRect();
    const w = Math.max(1, Math.round(r.width * dpr));
    const hgt = Math.max(1, Math.round(r.height * dpr));
    if (cv.width !== w || cv.height !== hgt) { cv.width = w; cv.height = hgt; }
    return dpr;
  }

  function cssVar(name, fallback) {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name);
    return (v && v.trim()) || fallback;
  }

  function draw() {
    if (dead) return;
    sizeCanvas();
    const W = cv.width, H = cv.height;
    const cx = W / 2, cy = H / 2;
    const pad = Math.min(W, H) * 0.06;
    const R = Math.min(W, H) / 2 - pad;

    const line = cssVar('--line', '#2c2a52');
    const muted = cssVar('--muted', '#8d8cba');
    const accent = cssVar('--accent', '#a97bff');
    const accent2 = cssVar('--accent2', '#62a8ff');

    ctx.clearRect(0, 0, W, H);

    // range rings, labelled in real metres so the plot is to scale
    ctx.strokeStyle = line; ctx.fillStyle = muted;
    ctx.lineWidth = Math.max(1, W / 900);
    ctx.font = `${Math.round(W / 78)}px ui-monospace, monospace`;
    const rings = 4;
    for (let i = 1; i <= rings; i++) {
      const rr = R * i / rings;
      ctx.beginPath(); ctx.arc(cx, cy, rr, 0, Math.PI * 2); ctx.stroke();
      ctx.fillText(((maxRange * i / rings) / 1000).toFixed(1) + ' m', cx + 4, cy - rr - 3);
    }
    // cardinal axes
    ctx.beginPath();
    ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
    ctx.stroke();

    // heading marker: 0 deg is straight up
    ctx.fillStyle = accent2;
    ctx.beginPath();
    ctx.moveTo(cx, cy - R - pad * 0.55);
    ctx.lineTo(cx - pad * 0.28, cy - R - pad * 0.05);
    ctx.lineTo(cx + pad * 0.28, cy - R - pad * 0.05);
    ctx.closePath(); ctx.fill();

    const pts = (last && last.points) || [];
    const dot = Math.max(1.4, W / 420);
    for (const [angDeg, distMm, inten] of pts) {
      if (!distMm || distMm > maxRange) continue;
      const a = (angDeg - 90) * Math.PI / 180;   // 0 deg at top, clockwise
      const rr = (distMm / maxRange) * R;
      const x = cx + Math.cos(a) * rr;
      const y = cy + Math.sin(a) * rr;
      // intensity drives alpha so weak returns read as weak, not as solid wall
      const alpha = 0.35 + 0.65 * Math.min(1, (inten || 0) / 200);
      ctx.globalAlpha = alpha;
      ctx.fillStyle = accent;
      ctx.beginPath(); ctx.arc(x, y, dot, 0, Math.PI * 2); ctx.fill();
    }
    ctx.globalAlpha = 1;

    // origin
    ctx.fillStyle = accent2;
    ctx.beginPath(); ctx.arc(cx, cy, dot * 1.6, 0, Math.PI * 2); ctx.fill();
  }

  // ---- ROS transport (roslibjs) ----------------------------------------
  function rosFresh() { return (Date.now() - rosAt) < 2000; }

  /* A LaserScan is evenly spaced rays from angle_min; this turns it into the
   * [angleDeg, distMm, intensity] triples the plot draws. Infinite and
   * out-of-band rays are DROPPED, not clamped - ROS uses +inf for "no return",
   * and clamping it to range_max would draw a solid wall at max range. */
  function fromLaserScan(m) {
    const out = [];
    const rr = m.ranges || [];
    const inten = m.intensities || [];
    const a0 = m.angle_min, da = m.angle_increment;
    for (let i = 0; i < rr.length; i++) {
      const r = rr[i];
      if (r === null || !isFinite(r)) continue;
      if (r < m.range_min || r > m.range_max) continue;
      let deg = (a0 + da * i) * 180 / Math.PI;
      deg = ((deg % 360) + 360) % 360;
      out.push([deg, Math.round(r * 1000), inten[i] || 0]);
    }
    return out;
  }

  async function wireRos(url) {
    rosUrl = url;
    onStatus = (ok, detail) => { if (!ok && !dead) { source = detail; paintSide(); } };
    try {
      const { ROSLIB, ros } = await connect(url, onStatus);
      if (dead) { release(url, onStatus); return; }
      unsub = subscribe(ROSLIB, ros, stTopic, stType, (m) => {
        if (dead) return;
        const pts = fromLaserScan(m);
        rosAt = Date.now();
        source = 'ROS';
        last = { ok: true, points: pts, count: pts.length,
                 hz: (last && last.hz) || 0, rpm: (last && last.rpm) || 0,
                 frames: null, crc_reject: null, age_s: 0 };
        lastErr = null;
        paintSide(); draw();
      });
    } catch (e) {
      if (!dead) { source = 'roslib: ' + (e.message || e); paintSide(); }
    }
  }

  // ---- fetching (HTTP fallback) ----------------------------------------
  async function tick() {
    // ROS is live, so do not also hammer the HTTP bridge - and on the rover
    // the HTTP bridge is stopped whenever the ROS stack owns the scanner.
    if (dead || !scanUrl || rosFresh()) return;
    try {
      const ac = new AbortController();
      inflight = ac;
      const to = setTimeout(() => ac.abort(), 1500);
      const r = await fetch(scanUrl, { cache: 'no-store', signal: ac.signal });
      clearTimeout(to);
      inflight = null;
      if (!r.ok) {
        let why = 'HTTP ' + r.status;
        try { const j = await r.json(); if (j.error) why = j.error; } catch (e) { /* body not json */ }
        lastErr = why; last = null;
      } else {
        last = await r.json();
        lastErr = null;
      }
    } catch (e) {
      if (!dead) { lastErr = e.name === 'AbortError' ? 'timeout' : (e.message || String(e)); last = null; }
      inflight = null;
    }
    if (dead) return;
    paintSide();
    draw();
  }

  function paintSide() {
    const s = last;
    const ok = !!(s && s.ok);
    overlay.className = 'stat ' + (ok ? 'ok' : 'bad');
    const via = rosFresh() ? 'ROS' : (ok ? 'HTTP' : source);
    overlay.textContent = ok
      ? `${via}   ${s.count} pts` + (s.hz ? `   ${s.hz.toFixed(2)} Hz` : '')
      : ('no scan — ' + (lastErr || (s && s.error) || 'unknown'));

    kvEl.replaceChildren();
    const rows = [
      ['status', ok ? 'spinning' : (lastErr || (s && s.error) || 'no data')],
      ['source', rosFresh() ? 'ROS 2 (rosbridge)' : (ok ? 'HTTP bridge (fallback)' : '—')],
      ['points', ok ? fmt(s.count) : DASH],
      ['rate', ok ? fmt(s.hz, ' Hz', 2) : DASH],
      ['rpm', ok ? fmt(s.rpm, '', 0) : DASH],
      ['frames', ok ? fmt(s.frames) : DASH],
      ['CRC rejects', ok ? fmt(s.crc_reject) : DASH],
      ['scan age', ok ? fmt(s.age_s, ' s', 2) : DASH],
    ];
    for (const [k, v] of rows) {
      kvEl.append(h('dt', { text: k }), h('dd', { text: v }));
    }
  }

  function update(full) {
    const st = ((full || {}).panels || {})[PANEL];
    if (!st) return;
    if (st.scan_url && st.scan_url !== scanUrl) {
      scanUrl = st.scan_url;
      tick();
    }
    if (st.scan_topic) stTopic = st.scan_topic;
    if (st.scan_type) stType = st.scan_type;
    if (!rosWired && st.rosbridge) { rosWired = true; wireRos(st.rosbridge); }
    if (!info.dataset.filled && st.lidar) {
      info.dataset.filled = '1';
      info.replaceChildren(h('div', {
        class: 'small',
        text: `${st.lidar.model} on the ${st.lidar.port} at ${st.lidar.baud} baud, `
            + `nominal ${st.lidar.nominal_hz} Hz. Scans are fetched directly from `
            + `${st.host} rather than through the Cortex backend. `
            + (st.body_note || ''),
      }));
    }
  }

  timer = setInterval(tick, FETCH_MS);
  const onResize = () => draw();
  window.addEventListener('resize', onResize);

  if (api && api.state) update(api.state);
  paintSide();
  draw();

  return {
    update,
    destroy() {
      dead = true;
      clearInterval(timer);
      window.removeEventListener('resize', onResize);
      try { if (inflight) inflight.abort(); } catch (e) { /* already settled */ }
      inflight = null;
      if (unsub) { unsub(); unsub = null; }
      if (rosUrl) { release(rosUrl, onStatus); rosUrl = null; }
    },
  };
}
