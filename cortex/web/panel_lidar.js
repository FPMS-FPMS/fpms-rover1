/* panel_lidar - the D500 scanner drawn on the ARENA, not as a bare polar plot.
 *
 * The scan arrives in the ROVER's frame. Rover 1 has NO LOCALISATION - nothing
 * publishes where it is - so the scan is drawn from an ASSUMED pose (HOME,
 * heading 90 deg) and the panel says so. Drawing it as if the pose were known
 * would make a map that looks authoritative and is not, which is worse than a
 * plot with no arena at all.
 *
 * M1 and M2 are the mission stack's own constants, carried through
 * panel_lidar.py verbatim. M3 and the water refill point are PLACEHOLDERS and
 * are drawn dashed with a "?" so they cannot be mistaken for measured values.
 *
 * The scan comes straight from the rover over rosbridge, not through panel
 * state: ~500 rays at 10 Hz in the SSE snapshot would be broadcast to every
 * client on every tick.
 *
 * mount(root, api) -> { update(state), destroy() }
 */
import { connect, release, subscribe } from './ros.js';

const PANEL = 'lidar';
const FETCH_MS = 200;
const RANGES = [1000, 2000, 4000, 8000, 12000];

const CSS = `
.flid{display:grid;gap:var(--s3,14px);grid-template-columns:minmax(420px,1.7fr) minmax(230px,1fr)}
@media(max-width:980px){.flid{grid-template-columns:1fr}}
.flid .wide{grid-column:1/-1}
.flid .plotwrap{position:relative;background:var(--panel2);border:1px solid var(--line);
  border-radius:10px;overflow:hidden;aspect-ratio:5/4}
.flid canvas{display:block;width:100%;height:100%}
.flid .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;align-items:baseline}
.flid .kv dt{color:var(--muted);font-size:12px}
.flid .kv dd{margin:0;font-variant-numeric:tabular-nums;color:var(--text);font-size:12px}
.flid .row{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin-top:8px}
.flid .rbtn{font:500 11px/1 system-ui;padding:5px 9px;border-radius:6px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--muted)}
.flid .rbtn.on{color:var(--accent);border-color:var(--accent)}
.flid .stat{position:absolute;left:10px;top:10px;font:500 11px/1.5 ui-monospace,monospace;
  color:var(--muted);background:rgba(0,0,0,.5);padding:5px 8px;border-radius:6px;
  pointer-events:none}
.flid .stat.ok{color:var(--ok)} .flid .stat.bad{color:var(--bad)}
.flid .warn{position:absolute;right:10px;top:10px;font:500 10px/1.4 ui-monospace,monospace;
  color:var(--warn);background:rgba(0,0,0,.5);padding:4px 8px;border-radius:6px;
  pointer-events:none;text-align:right}
.flid h3{font:600 11px/1.4 system-ui;letter-spacing:.07em;text-transform:uppercase;
  color:var(--muted);margin:14px 0 6px}
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
  // Hoisted: subscription callbacks can fire before mount() returns.
  let dead = false;
  let scanUrl = null;
  let rosUrl = null, rosWired = false, unsub = null, onStatus = null;
  let stTopic = '/scan', stType = 'sensor_msgs/msg/LaserScan';
  let rosAt = 0, source = 'connecting';
  let rosTimes = [], rosEverUp = false, wasFresh = false;
  const mountedAt = Date.now();
  let last = null, lastErr = null, timer = null, inflight = null;
  let arena = null, markers = [], body = null;
  let showScan = true, showMarkers = true;
  let maxRange = 4000;

  const el = h('div', { class: 'flid' });
  root.replaceChildren(h('style', { text: CSS }), el);

  const cv = h('canvas');
  const overlay = h('div', { class: 'stat', text: 'connecting…' });
  const warnBox = h('div', { class: 'warn' });
  const plot = h('div', { class: 'plotwrap' }, [cv, overlay, warnBox]);

  const kvEl = h('dl', { class: 'kv' });
  const mkEl = h('dl', { class: 'kv' });
  const rangeRow = h('div', { class: 'row' });
  const toggleRow = h('div', { class: 'row' });
  const btnRefresh = h('button', { class: 'rbtn', type: 'button', text: 'Refresh rover' });
  const btnScan = h('button', { class: 'rbtn on', type: 'button', text: 'Scan' });
  const btnMk = h('button', { class: 'rbtn on', type: 'button', text: 'Markers' });
  toggleRow.append(btnScan, btnMk, btnRefresh);

  const side = h('div', { class: 'card' }, [
    h('h2', { text: 'Scanner' }), kvEl,
    h('h3', { text: 'Markers' }), mkEl,
    h('h3', { text: 'Scan range' }), rangeRow,
    h('h3', { text: 'View' }), toggleRow,
  ]);
  const info = h('div', { class: 'card wide note', 'data-detail': 'full' });
  el.append(plot, side, info);

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

  btnScan.addEventListener('click', () => {
    showScan = !showScan; btnScan.className = 'rbtn' + (showScan ? ' on' : ''); draw();
  });
  btnMk.addEventListener('click', () => {
    showMarkers = !showMarkers; btnMk.className = 'rbtn' + (showMarkers ? ' on' : ''); draw();
  });
  // Borrowed from the Control tab: forces the rover poll rather than waiting
  // for the next 2 s tick.
  btnRefresh.addEventListener('click', async () => {
    if (!api.call) return;
    btnRefresh.disabled = true; btnRefresh.textContent = 'Refreshing…';
    try { await api.call('control', 'refresh', {}); }
    catch (e) { /* the Control tab shows the reason */ }
    finally { btnRefresh.disabled = false; btnRefresh.textContent = 'Refresh rover'; }
  });

  // ---- ROS -------------------------------------------------------------
  const rosFresh = () => (Date.now() - rosAt) < 2000;

  function fromLaserScan(m) {
    const out = [];
    const rr = m.ranges || [], inten = m.intensities || [];
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
        const now = Date.now();
        rosAt = now; rosEverUp = true; source = 'ROS';
        rosTimes.push(now);
        while (rosTimes.length && now - rosTimes[0] > 3000) rosTimes.shift();
        const hz = rosTimes.length > 1
          ? (rosTimes.length - 1) / ((now - rosTimes[0]) / 1000) : 0;
        last = { ok: true, points: pts, count: pts.length, hz, rpm: hz * 60,
                 rays: (m.ranges || []).length, frames: null, crc_reject: null, age_s: 0 };
        lastErr = null;
        paintSide(); draw();
      });
    } catch (e) {
      if (!dead) { source = 'roslib: ' + (e.message || e); paintSide(); }
    }
  }

  // ---- drawing ---------------------------------------------------------
  const ctx = cv.getContext('2d');
  const cssVar = (n, f) => (getComputedStyle(document.documentElement)
    .getPropertyValue(n) || '').trim() || f;

  function sizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const r = plot.getBoundingClientRect();
    const w = Math.max(1, Math.round(r.width * dpr));
    const hh = Math.max(1, Math.round(r.height * dpr));
    if (cv.width !== w || cv.height !== hh) { cv.width = w; cv.height = hh; }
  }

  function draw() {
    if (dead) return;
    sizeCanvas();
    const W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);
    const line = cssVar('--line', '#2c2a52'), muted = cssVar('--muted', '#8d8cba');
    const accent = cssVar('--accent', '#a97bff'), accent2 = cssVar('--accent2', '#62a8ff');
    const warn = cssVar('--warn', '#ffcf5c'), ok = cssVar('--ok', '#4fe8b4');

    if (!arena) {
      ctx.fillStyle = muted; ctx.font = `${Math.round(W / 40)}px system-ui`;
      ctx.fillText('waiting for arena geometry…', 16, 28);
      return;
    }

    // The OUTER bounds are deliberately wider than the driveable arena - the
    // zones sit outside it - so the view is fitted to those, not to the arena.
    const ob = arena.obj_bounds;
    const fw = ob.x_max - ob.x_min, fh = ob.y_max - ob.y_min;
    const pad = Math.min(W, H) * 0.06;
    const s = Math.min((W - pad * 2) / fw, (H - pad * 2) / fh);
    const ox = (W - fw * s) / 2, oy = (H - fh * s) / 2;
    // Arena y grows AWAY from home; screen y grows downward, so y is flipped.
    const X = (mx) => ox + (mx - ob.x_min) * s;
    const Y = (my) => oy + (ob.y_max - my) * s;

    // outer object bounds - faint
    ctx.strokeStyle = line; ctx.lineWidth = Math.max(1, W / 900); ctx.setLineDash([6, 6]);
    ctx.strokeRect(X(ob.x_min), Y(ob.y_max), fw * s, fh * s);
    ctx.setLineDash([]);

    // the driveable arena
    ctx.strokeStyle = muted; ctx.lineWidth = Math.max(1.4, W / 620);
    ctx.strokeRect(X(0), Y(arena.h), arena.w * s, arena.h * s);

    // edge-keep inset: where the rover CENTRE may not go
    const ek = arena.edge_keep;
    ctx.strokeStyle = line; ctx.setLineDash([3, 5]);
    ctx.strokeRect(X(ek), Y(arena.h - ek), (arena.w - 2 * ek) * s, (arena.h - 2 * ek) * s);
    ctx.setLineDash([]);

    // 500 mm grid, for scale
    ctx.strokeStyle = line; ctx.globalAlpha = 0.5; ctx.lineWidth = 1;
    for (let gx = 0; gx <= arena.w; gx += 500) {
      ctx.beginPath(); ctx.moveTo(X(gx), Y(0)); ctx.lineTo(X(gx), Y(arena.h)); ctx.stroke();
    }
    for (let gy = 0; gy <= arena.h; gy += 500) {
      ctx.beginPath(); ctx.moveTo(X(0), Y(gy)); ctx.lineTo(X(arena.w), Y(gy)); ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // ---- the scan, from an ASSUMED pose ---------------------------------
    const home = arena.home;
    const pose = { x: home.x, y: home.y, hdg: home.heading_deg };
    if (showScan && last && last.points && last.points.length) {
      const stale = rosEverUp && !rosFresh();
      ctx.globalAlpha = stale ? 0.18 : 0.85;
      ctx.fillStyle = accent;
      const dot = Math.max(1.2, W / 560);
      for (const [angDeg, distMm, inten] of last.points) {
        if (!distMm || distMm > maxRange) continue;
        // scan angle is CCW from the rover's forward axis; add the heading.
        const a = (pose.hdg + angDeg) * Math.PI / 180;
        const px = pose.x + Math.cos(a) * distMm;
        const py = pose.y + Math.sin(a) * distMm;
        ctx.globalAlpha = (stale ? 0.18 : 0.35 + 0.6 * Math.min(1, (inten || 0) / 200));
        ctx.beginPath(); ctx.arc(X(px), Y(py), dot, 0, Math.PI * 2); ctx.fill();
      }
      ctx.globalAlpha = 1;
    }

    // ---- rover at the assumed pose --------------------------------------
    if (body) {
      const a = pose.hdg * Math.PI / 180;
      const ca = Math.cos(a), sa = Math.sin(a);
      const corners = [[body.front, body.left], [body.front, body.right],
                       [body.back, body.right], [body.back, body.left]];
      ctx.strokeStyle = accent2; ctx.lineWidth = Math.max(1.4, W / 620);
      ctx.beginPath();
      corners.forEach(([fx, fy], i) => {
        const px = pose.x + fx * ca - fy * sa, py = pose.y + fx * sa + fy * ca;
        i ? ctx.lineTo(X(px), Y(py)) : ctx.moveTo(X(px), Y(py));
      });
      ctx.closePath(); ctx.stroke();
    }
    ctx.fillStyle = accent2;
    ctx.beginPath(); ctx.arc(X(pose.x), Y(pose.y), Math.max(3, W / 260), 0, Math.PI * 2); ctx.fill();

    // ---- markers ---------------------------------------------------------
    if (showMarkers) {
      const fs = Math.max(10, Math.round(W / 62));
      ctx.font = `600 ${fs}px ui-monospace, monospace`;
      for (const m of markers) {
        const px = X(m.x), py = Y(m.y);
        const col = m.kind === 'water' ? accent2 : (m.measured ? ok : warn);
        const rad = (m.kind === 'zone' ? arena.zone_radius : 90) * s;
        ctx.strokeStyle = col; ctx.lineWidth = Math.max(1.4, W / 700);
        // Unmeasured markers are DASHED. A placeholder must not look like a
        // surveyed point on a map someone will drive from.
        ctx.setLineDash(m.measured ? [] : [5, 4]);
        ctx.beginPath(); ctx.arc(px, py, rad, 0, Math.PI * 2); ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(px - rad * 0.35, py); ctx.lineTo(px + rad * 0.35, py);
        ctx.moveTo(px, py - rad * 0.35); ctx.lineTo(px, py + rad * 0.35); ctx.stroke();
        ctx.fillStyle = col;
        const label = m.id + (m.measured ? '' : ' ?');
        ctx.fillText(label, px + rad + 4, py - 2);
      }
      // HOME
      ctx.strokeStyle = ok; ctx.lineWidth = Math.max(1.4, W / 700);
      const hr = Math.max(6, W / 130);
      ctx.beginPath(); ctx.rect(X(home.x) - hr, Y(home.y) - hr, hr * 2, hr * 2); ctx.stroke();
      ctx.fillStyle = ok; ctx.fillText('HOME', X(home.x) + hr + 4, Y(home.y) - 2);
    }

    // stale banner
    if (rosEverUp && !rosFresh()) {
      const age = ((Date.now() - rosAt) / 1000).toFixed(1);
      const fs2 = Math.max(13, Math.round(W / 28));
      ctx.font = `600 ${fs2}px system-ui, sans-serif`;
      const msg = `FEED STOPPED  ${age}s`;
      const tw = ctx.measureText(msg).width;
      ctx.fillStyle = 'rgba(0,0,0,.7)';
      ctx.fillRect(W / 2 - tw / 2 - 12, H / 2 - fs2, tw + 24, fs2 + 14);
      ctx.fillStyle = '#ff5c8a'; ctx.fillText(msg, W / 2 - tw / 2, H / 2 + 4);
    }
  }

  // ---- HTTP fallback ---------------------------------------------------
  async function tick() {
    if (dead || !scanUrl || rosFresh()) return;
    if (rosEverUp) return;
    if (Date.now() - mountedAt < 8000) return;
    try {
      const ac = new AbortController(); inflight = ac;
      const to = setTimeout(() => ac.abort(), 1500);
      const r = await fetch(scanUrl, { cache: 'no-store', signal: ac.signal });
      clearTimeout(to); inflight = null;
      if (!r.ok) { lastErr = 'HTTP ' + r.status; last = null; }
      else { last = await r.json(); lastErr = null; }
    } catch (e) {
      if (!dead) { lastErr = e.name === 'AbortError' ? 'timeout' : (e.message || String(e)); last = null; }
      inflight = null;
    }
    if (!dead) { paintSide(); draw(); }
  }

  function paintSide() {
    const s = last, okk = !!(s && s.ok), fresh = rosFresh();
    if (rosEverUp && !fresh) {
      overlay.className = 'stat bad';
      overlay.textContent = `FEED STOPPED — ${((Date.now() - rosAt) / 1000).toFixed(1)}s ago`;
    } else {
      overlay.className = 'stat ' + (okk ? 'ok' : 'bad');
      overlay.textContent = okk
        ? `${fresh ? 'ROS' : 'HTTP'}   ${s.count} pts` + (s.hz ? `   ${s.hz.toFixed(2)} Hz` : '')
        : ('no scan — ' + (lastErr || 'unknown'));
    }
    kvEl.replaceChildren();
    for (const [k, v] of [
      ['status', (rosEverUp && !fresh) ? 'FEED STOPPED' : (okk ? 'spinning' : 'no data')],
      ['source', fresh ? 'ROS 2 (rosbridge)' : (okk ? 'HTTP bridge' : DASH)],
      ['rays', okk && s.rays ? fmt(s.rays) : DASH],
      ['returns', okk ? fmt(s.count) : DASH],
      ['rate', okk ? fmt(s.hz, ' Hz', 2) : DASH],
      ['rpm', okk ? fmt(s.rpm, '', 0) : DASH],
    ]) kvEl.append(h('dt', { text: k }), h('dd', { text: v }));
  }

  function paintMarkers() {
    mkEl.replaceChildren();
    for (const m of markers) {
      mkEl.append(h('dt', { text: m.id + (m.measured ? '' : ' ?') }),
                  h('dd', { text: `${m.x.toFixed(0)}, ${m.y.toFixed(0)} mm`
                    + (m.measured ? '' : '  (placeholder)') }));
    }
    if (arena) {
      mkEl.append(h('dt', { text: 'HOME' }),
                  h('dd', { text: `${arena.home.x.toFixed(0)}, ${arena.home.y.toFixed(0)} mm` }));
    }
  }

  function update(full) {
    const st = ((full || {}).panels || {})[PANEL];
    if (!st) return;
    if (st.scan_url && st.scan_url !== scanUrl) { scanUrl = st.scan_url; tick(); }
    if (st.scan_topic) stTopic = st.scan_topic;
    if (st.scan_type) stType = st.scan_type;
    if (st.arena && !arena) { arena = st.arena; draw(); }
    if (st.markers && !markers.length) { markers = st.markers; paintMarkers(); draw(); }
    if (st.body_mm && !body) { body = st.body_mm; draw(); }
    if (!rosWired && st.rosbridge) { rosWired = true; wireRos(st.rosbridge); }

    const unmeasured = markers.filter((m) => !m.measured).map((m) => m.id);
    warnBox.textContent = unmeasured.length
      ? `pose ASSUMED = HOME (no localisation)\n${unmeasured.join(', ')} not measured`
      : 'pose ASSUMED = HOME (no localisation)';
    warnBox.style.whiteSpace = 'pre';

    if (!info.dataset.filled && st.lidar) {
      info.dataset.filled = '1';
      info.replaceChildren(h('div', { class: 'small', text:
        `${st.lidar.model} on the ${st.lidar.port}. M1 and M2 are the mission `
        + `stack's own constants, carried through verbatim. `
        + (unmeasured.length ? `${unmeasured.join(' and ')} are PLACEHOLDERS, drawn dashed with a "?" - `
          + `give me the real coordinates and they are a one-line change. ` : '')
        + `Rover 1 has no localisation, so the scan is drawn from an assumed `
        + `pose at HOME facing ${st.arena ? st.arena.home.heading_deg : 90}°; `
        + `it is NOT a tracked position.` }));
    }
  }

  timer = setInterval(() => {
    if (dead) return;
    const fresh = rosFresh();
    if (wasFresh !== fresh) { wasFresh = fresh; paintSide(); draw(); }
    else if (!fresh && rosEverUp) draw();
    tick();
  }, FETCH_MS);
  const onResize = () => draw();
  window.addEventListener('resize', onResize);
  if (api && api.state) update(api.state);
  paintSide(); draw();

  return {
    update,
    destroy() {
      dead = true;
      clearInterval(timer);
      window.removeEventListener('resize', onResize);
      try { if (inflight) inflight.abort(); } catch (e) { /* settled */ }
      inflight = null;
      if (unsub) { unsub(); unsub = null; }
      if (rosUrl) { release(rosUrl, onStatus); rosUrl = null; }
    },
  };
}
