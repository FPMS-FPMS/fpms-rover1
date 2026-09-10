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
import { buildGrid, plan, GRID_RES, ROUTE_CLEARANCE } from './planner.js';

const PANEL = 'lidar';
const FETCH_MS = 200;

/* OBSTACLE DETECTION
 * The zones and HOME are FIXED; obstacles move between runs, so they are
 * detected rather than configured. Method and constants are Rover 2's, from
 * INTERNATIONALS_ROVER2.py - a scan return is an obstacle if it is inside the
 * arena, not near a waypoint, and not near a wall. Everything left is
 * clustered.
 *
 * WAYPOINT_EXCLUDE is 150 mm there. A zone is 131 mm in radius, so 150 clears
 * the vase itself with 19 mm to spare - tight, deliberately: the point is to
 * still SEE an obstacle standing close to a zone.
 */
const WAYPOINT_EXCLUDE_MM = 150;
const WALL_MARGIN_MM = 30;      // returns this close to a wall ARE the wall
const CLUSTER_RADIUS_MM = 220;  // CLUSTER_RADIUS_MM
const MIN_CLUSTER_PTS = 3;      // fewer than this is noise, not an object
const MAX_CLUSTERS = 30;
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
  let rosUrl = null, rosWired = false, unsub = null, unsubPose = null, onStatus = null;
  let stTopic = '/scan', stType = 'sensor_msgs/msg/LaserScan';
  let poseTopic = '/rover/arena_pose', poseType = 'geometry_msgs/msg/PoseStamped';
  // LIVE pose in arena mm from odom_node. null until it arrives, and the panel
  // says which it is using - a map drawn from an assumed pose that is silently
  // wrong is worse than one that admits it does not know.
  let pose = null, poseAt = 0;
  let rosAt = 0, source = 'connecting';
  let rosTimes = [], rosEverUp = false, wasFresh = false;
  const mountedAt = Date.now();
  let last = null, lastErr = null, timer = null, inflight = null;
  let arena = null, markers = [], body = null;
  let showScan = true, showMarkers = true, showObs = true, showPlan = true;
  let obstacles = [];
  let route = null;
  // Rover 1's body is NOT measured. This is Rover 2's ROBOT_W/2 - the same
  // figure its own planner uses (ROBOT_RADIUS = ROBOT_W / 2 = 260/2).
  //
  // NOT the circumscribed radius of 239. That is the right number for a rover
  // ROTATING IN PLACE, but 239 + 90 clearance is 329 mm of inflation on a
  // 1000 mm arena, which leaves so little free space that the planner has to
  // relocate the start halfway across the field. Rover 2's own file already
  // flags its 90 mm clearance as "too tight for 230mm chassis", so treat both
  // numbers as provisional until Rover 1 is measured.
  let robotRadius = 130;
  let lastPlanAt = 0;
  // Which end of the arena the operator is standing at. The stored geometry
  // puts the zones at high y; someone facing the arena from the far side sees
  // them at the BOTTOM. This is a view preference, never a change to the data.
  // Default matches how the operator describes the arena: M1 bottom-left,
  // M2 bottom-right, i.e. the zones nearest the bottom of the screen.
  let flip = true;
  let maxRange = 4000;

  const el = h('div', { class: 'flid' });
  root.replaceChildren(h('style', { text: CSS }), el);

  const cv = h('canvas');
  const overlay = h('div', { class: 'stat', text: 'connecting…' });
  const warnBox = h('div', { class: 'warn' });
  const plot = h('div', { class: 'plotwrap' }, [cv, overlay, warnBox]);

  const kvEl = h('dl', { class: 'kv' });
  const mkEl = h('dl', { class: 'kv' });
  const obsEl = h('dl', { class: 'kv' });
  const rangeRow = h('div', { class: 'row' });
  const toggleRow = h('div', { class: 'row' });
  const btnRefresh = h('button', { class: 'rbtn', type: 'button', text: 'Refresh rover' });
  const btnScan = h('button', { class: 'rbtn on', type: 'button', text: 'Scan' });
  const btnMk = h('button', { class: 'rbtn on', type: 'button', text: 'Markers' });
  const btnFlip = h('button', { class: 'rbtn on', type: 'button', text: 'Flip view' });
  const btnObs = h('button', { class: 'rbtn on', type: 'button', text: 'Obstacles' });
  const btnPlan = h('button', { class: 'rbtn on', type: 'button', text: 'Route' });
  toggleRow.append(btnScan, btnMk, btnObs, btnPlan, btnFlip, btnRefresh);

  const side = h('div', { class: 'card' }, [
    h('h2', { text: 'Scanner' }), kvEl,
    h('h3', { text: 'Markers (fixed)' }), mkEl,
    h('h3', { text: 'Obstacles (detected)' }), obsEl,
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
  btnObs.addEventListener('click', () => {
    showObs = !showObs; btnObs.className = 'rbtn' + (showObs ? ' on' : ''); draw();
  });
  btnPlan.addEventListener('click', () => {
    showPlan = !showPlan; btnPlan.className = 'rbtn' + (showPlan ? ' on' : '');
    if (showPlan) replan();
    draw();
  });
  btnFlip.addEventListener('click', () => {
    flip = !flip; btnFlip.className = 'rbtn' + (flip ? ' on' : ''); draw();
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
  const poseFresh = () => pose && (Date.now() - poseAt) < 2000;

  /* The pose the drawing uses: live odometry when it is arriving, otherwise
   * the placement at HOME. Which one is in force is shown on the canvas. */
  function currentPose() {
    if (poseFresh()) return pose;
    if (!arena) return { x: 0, y: 0, hdg: 90, live: false };
    return { x: arena.home.x, y: arena.home.y, hdg: arena.home.heading_deg, live: false };
  }

  function quatYawDeg(q) {
    if (!q) return 0;
    // Planar: yaw straight out of the z/w terms.
    return Math.atan2(2 * (q.w * q.z), 1 - 2 * (q.z * q.z)) * 180 / Math.PI;
  }

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
      unsubPose = subscribe(ROSLIB, ros, poseTopic, poseType, (m) => {
        if (dead || !m.pose) return;
        pose = { x: m.pose.position.x, y: m.pose.position.y,
                 hdg: quatYawDeg(m.pose.orientation), live: true };
        poseAt = Date.now();
      });
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
        obstacles = detectObstacles(pts, currentPose());
        // Replanning is cheap on a 21x25 grid but not free; twice a second is
        // far faster than the rover can invalidate a route.
        if (showPlan && Date.now() - lastPlanAt > 500) replan();
        paintSide(); paintObstacles(); draw();
      });
    } catch (e) {
      if (!dead) { source = 'roslib: ' + (e.message || e); paintSide(); }
    }
  }

  /* Scan returns that are neither a wall nor a known fixture, grouped.
   * Positions inherit the ASSUMED pose - without localisation an obstacle's
   * arena coordinate is only as good as the guess that the rover is at HOME.
   * That is stated on the canvas; it is not a detail to bury. */
  function detectObstacles(points, pose) {
    if (!arena || !points || !points.length) return [];
    const kept = [];
    for (const [angDeg, distMm] of points) {
      if (!distMm || distMm > maxRange) continue;
      const a = (pose.hdg + angDeg) * Math.PI / 180;
      const wx = pose.x + Math.cos(a) * distMm;
      const wy = pose.y + Math.sin(a) * distMm;
      // outside the arena entirely
      if (wx < 0 || wx > arena.w || wy < 0 || wy > arena.h) continue;
      // the walls themselves
      if (wx < WALL_MARGIN_MM || wx > arena.w - WALL_MARGIN_MM ||
          wy < WALL_MARGIN_MM || wy > arena.h - WALL_MARGIN_MM) continue;
      // known fixtures: the zones, the lake and HOME are not obstacles
      let fixture = false;
      for (const m of markers) {
        if (Math.hypot(wx - m.x, wy - m.y) < WAYPOINT_EXCLUDE_MM) { fixture = true; break; }
      }
      if (!fixture && Math.hypot(wx - arena.home.x, wy - arena.home.y) < WAYPOINT_EXCLUDE_MM) {
        fixture = true;
      }
      if (fixture) continue;
      kept.push([wx, wy]);
    }
    // Greedy clustering against a running centroid - enough for a few objects
    // on a 1 x 1.2 m floor, and cheap enough to run every frame.
    const out = [];
    for (const [x, y] of kept) {
      let hit = null;
      for (const c of out) {
        if (Math.hypot(x - c.x, y - c.y) <= CLUSTER_RADIUS_MM) { hit = c; break; }
      }
      if (hit) {
        hit.n++; hit.sx += x; hit.sy += y;
        hit.x = hit.sx / hit.n; hit.y = hit.sy / hit.n;
        hit.minx = Math.min(hit.minx, x); hit.maxx = Math.max(hit.maxx, x);
        hit.miny = Math.min(hit.miny, y); hit.maxy = Math.max(hit.maxy, y);
      } else if (out.length < MAX_CLUSTERS) {
        out.push({ x, y, sx: x, sy: y, n: 1, minx: x, maxx: x, miny: y, maxy: y });
      }
    }
    return out.filter((c) => c.n >= MIN_CLUSTER_PTS).map((c) => ({
      x: c.x, y: c.y, n: c.n,
      w: Math.max(40, c.maxx - c.minx), d: Math.max(40, c.maxy - c.miny),
    }));
  }

  /* Plan to the midpoint of M1 and M2 - the lake, i.e. the refill point.
   * Rebuilt from the CURRENT obstacles each time, because they move. */
  function replan() {
    if (!arena || !markers.length) { route = null; return; }
    const m1 = markers.find((m) => m.id === 'M1');
    const m2 = markers.find((m) => m.id === 'M2');
    if (!m1 || !m2) { route = null; return; }
    const goal = { x: (m1.x + m2.x) / 2, y: (m1.y + m2.y) / 2 };
    const grid = buildGrid(arena, showObs ? obstacles : [], markers, robotRadius);
    route = plan(arena, grid, currentPose(), goal);
    route.goal = goal;
    lastPlanAt = Date.now();
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
    // ONLY the y axis flips. Mirroring x as well would swap left and right -
    // M2 would appear left of M1 - which is a different arena, not a different
    // viewpoint. Standing at the other end changes which end is near you; it
    // does not swap your hands.
    const X = (mx) => ox + (mx - ob.x_min) * s;
    const Y = (my) => flip ? oy + (my - ob.y_min) * s : oy + (ob.y_max - my) * s;
    // Which arena corner lands top-left depends on the flip, so a rectangle
    // must be built from BOTH mapped corners. Assuming Y(y_high) is the top
    // silently drew the arena off the bottom of the canvas once the view was
    // flipped - the box was there, just outside the viewport.
    const rect = (x0, y0, x1, y1) => {
      const sx0 = X(x0), sx1 = X(x1), sy0 = Y(y0), sy1 = Y(y1);
      return [Math.min(sx0, sx1), Math.min(sy0, sy1),
              Math.abs(sx1 - sx0), Math.abs(sy1 - sy0)];
    };

    // outer object bounds - faint
    ctx.strokeStyle = line; ctx.lineWidth = Math.max(1, W / 900); ctx.setLineDash([6, 6]);
    ctx.strokeRect(...rect(ob.x_min, ob.y_min, ob.x_max, ob.y_max));
    ctx.setLineDash([]);

    // the driveable arena
    ctx.strokeStyle = muted; ctx.lineWidth = Math.max(1.4, W / 620);
    ctx.strokeRect(...rect(0, 0, arena.w, arena.h));

    // edge-keep inset: where the rover CENTRE may not go
    const ek = arena.edge_keep;
    ctx.strokeStyle = line; ctx.setLineDash([3, 5]);
    ctx.strokeRect(...rect(ek, ek, arena.w - ek, arena.h - ek));
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
    const p = currentPose();
    if (showScan && last && last.points && last.points.length) {
      const stale = rosEverUp && !rosFresh();
      ctx.globalAlpha = stale ? 0.18 : 0.85;
      ctx.fillStyle = accent;
      const dot = Math.max(1.2, W / 560);
      for (const [angDeg, distMm, inten] of last.points) {
        if (!distMm || distMm > maxRange) continue;
        // scan angle is CCW from the rover's forward axis; add the heading.
        const a = (p.hdg + angDeg) * Math.PI / 180;
        const px = p.x + Math.cos(a) * distMm;
        const py = p.y + Math.sin(a) * distMm;
        ctx.globalAlpha = (stale ? 0.18 : 0.35 + 0.6 * Math.min(1, (inten || 0) / 200));
        ctx.beginPath(); ctx.arc(X(px), Y(py), dot, 0, Math.PI * 2); ctx.fill();
      }
      ctx.globalAlpha = 1;
    }

    // ---- rover at the assumed pose --------------------------------------
    if (body) {
      const a = p.hdg * Math.PI / 180;
      const ca = Math.cos(a), sa = Math.sin(a);
      const corners = [[body.front, body.left], [body.front, body.right],
                       [body.back, body.right], [body.back, body.left]];
      ctx.strokeStyle = accent2; ctx.lineWidth = Math.max(1.4, W / 620);
      ctx.beginPath();
      corners.forEach(([fx, fy], i) => {
        const px = p.x + fx * ca - fy * sa, py = p.y + fx * sa + fy * ca;
        i ? ctx.lineTo(X(px), Y(py)) : ctx.moveTo(X(px), Y(py));
      });
      ctx.closePath(); ctx.stroke();
    }
    ctx.fillStyle = accent2;
    ctx.beginPath(); ctx.arc(X(p.x), Y(p.y), Math.max(3, W / 260), 0, Math.PI * 2); ctx.fill();
    // heading tick, so which way it faces is unambiguous
    {
      const a2 = p.hdg * Math.PI / 180, L = Math.max(10, W / 34);
      ctx.strokeStyle = accent2; ctx.lineWidth = Math.max(1.6, W / 520);
      ctx.beginPath(); ctx.moveTo(X(p.x), Y(p.y));
      ctx.lineTo(X(p.x + Math.cos(a2) * L / s), Y(p.y + Math.sin(a2) * L / s));
      ctx.stroke();
    }

    // ---- detected obstacles ---------------------------------------------
    if (showObs && obstacles.length) {
      const bad = cssVar('--bad', '#ff5c8a');
      ctx.font = `600 ${Math.max(9, Math.round(W / 78))}px ui-monospace, monospace`;
      for (const o of obstacles) {
        const bw = o.w * s, bd = o.d * s;
        ctx.strokeStyle = bad; ctx.lineWidth = Math.max(1.3, W / 780);
        ctx.setLineDash([4, 3]);
        ctx.strokeRect(X(o.x) - bw / 2, Y(o.y) - bd / 2, bw, bd);
        ctx.setLineDash([]);
        ctx.globalAlpha = 0.14; ctx.fillStyle = bad;
        ctx.fillRect(X(o.x) - bw / 2, Y(o.y) - bd / 2, bw, bd);
        ctx.globalAlpha = 1;
      }
    }

    // ---- planned route ---------------------------------------------------
    if (showPlan && route && route.path && route.path.length > 1) {
      ctx.strokeStyle = cssVar('--accent2', '#62a8ff');
      ctx.lineWidth = Math.max(2, W / 380);
      ctx.setLineDash([]);
      ctx.beginPath();
      route.path.forEach((pt, i) => {
        i ? ctx.lineTo(X(pt.x), Y(pt.y)) : ctx.moveTo(X(pt.x), Y(pt.y));
      });
      ctx.stroke();
      // waypoints
      ctx.fillStyle = cssVar('--accent2', '#62a8ff');
      for (const pt of route.path) {
        ctx.beginPath(); ctx.arc(X(pt.x), Y(pt.y), Math.max(2, W / 420), 0, Math.PI * 2); ctx.fill();
      }
      // If the route had to start away from the rover, show that gap. An
      // unbroken line from the rover would claim a route it does not have.
      if (route.movedStartMm > GRID_RES) {
        const p0 = route.path[0];
        ctx.setLineDash([3, 4]);
        ctx.strokeStyle = cssVar('--bad', '#ff5c8a');
        ctx.lineWidth = Math.max(1.4, W / 620);
        ctx.beginPath();
        ctx.moveTo(X(p.x), Y(p.y)); ctx.lineTo(X(p0.x), Y(p0.y));
        ctx.stroke(); ctx.setLineDash([]);
      }

      // The last leg is the part the rover CANNOT drive - the goal is outside
      // the arena. Dashed, so it is obviously not a driveable segment.
      if (route.standoffMm > GRID_RES && route.goal) {
        const endp = route.path[route.path.length - 1];
        ctx.setLineDash([5, 5]);
        ctx.strokeStyle = cssVar('--warn', '#ffcf5c');
        ctx.lineWidth = Math.max(1.4, W / 620);
        ctx.beginPath();
        ctx.moveTo(X(endp.x), Y(endp.y));
        ctx.lineTo(X(route.goal.x), Y(route.goal.y));
        ctx.stroke(); ctx.setLineDash([]);
      }
    }

    // ---- markers ---------------------------------------------------------
    if (showMarkers) {
      const fs = Math.max(10, Math.round(W / 62));
      ctx.font = `600 ${fs}px ui-monospace, monospace`;
      for (const m of markers) {
        const px = X(m.x), py = Y(m.y);
        const col = m.kind === 'water' ? accent2 : (m.measured ? ok : warn);
        // A marker may carry its own radius - the lake does, because it sits
        // 215 mm from M1 and M2 and anything wider than 84 mm overlaps them.
        const radMm = m.radius || (m.kind === 'zone' ? arena.zone_radius : 90);
        const rad = radMm * s;
        ctx.strokeStyle = col; ctx.lineWidth = Math.max(1.4, W / 700);
        // Unmeasured markers are DASHED. A placeholder must not look like a
        // surveyed point on a map someone will drive from.
        ctx.setLineDash(m.measured ? [] : [5, 4]);
        ctx.beginPath(); ctx.arc(px, py, rad, 0, Math.PI * 2); ctx.stroke();
        if (m.kind === 'water') {           // the lake reads as a body, not a target
          ctx.globalAlpha = 0.18; ctx.fillStyle = col; ctx.fill(); ctx.globalAlpha = 1;
        }
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(px - rad * 0.35, py); ctx.lineTo(px + rad * 0.35, py);
        ctx.moveTo(px, py - rad * 0.35); ctx.lineTo(px, py + rad * 0.35); ctx.stroke();
        ctx.fillStyle = col;
        const label = m.id + (m.measured ? '' : ' ?');
        // Centred ABOVE the marker. To the right, a label sits between two
        // circles and reads as belonging to the wrong one - which it did:
        // "M1" appeared to label the lake.
        const lw = ctx.measureText(label).width;
        ctx.fillText(label, px - lw / 2, py - rad - Math.max(4, fs * 0.35));
      }
      // HOME
      ctx.strokeStyle = ok; ctx.lineWidth = Math.max(1.4, W / 700);
      const hr = Math.max(6, W / 130);
      ctx.beginPath(); ctx.rect(X(home.x) - hr, Y(home.y) - hr, hr * 2, hr * 2); ctx.stroke();
      const hlw = ctx.measureText('HOME').width;
      ctx.fillStyle = ok;
      ctx.fillText('HOME', X(home.x) - hlw / 2, Y(home.y) - hr - Math.max(4, fs * 0.35));
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
      ['obstacles', showObs ? String(obstacles.length) : 'hidden'],
      ['pose', poseFresh() ? 'live (odometry)' : 'assumed (HOME)'],
      ['position', poseFresh() ? `${pose.x.toFixed(0)}, ${pose.y.toFixed(0)} mm` : DASH],
      ['heading', poseFresh() ? `${pose.hdg.toFixed(0)}°` : DASH],
      ['route', !route ? DASH : (route.status === 'ok'
        ? `${route.path.length} legs, ${routeLenMm(route).toFixed(0)} mm` : route.status)],
      ['standoff', route && route.status === 'ok'
        ? `${route.standoffMm.toFixed(0)} mm from goal` : DASH],
      ['clearance', `${robotRadius} + ${ROUTE_CLEARANCE} mm (unmeasured)`],
      ['start moved', route && route.movedStartMm > 1
        ? `${route.movedStartMm.toFixed(0)} mm — rover is inside inflation` : 'no'],
    ]) kvEl.append(h('dt', { text: k }), h('dd', { text: v }));
  }

  function routeLenMm(r) {
    let d = 0;
    for (let i = 1; i < r.path.length; i++) {
      d += Math.hypot(r.path[i].x - r.path[i - 1].x, r.path[i].y - r.path[i - 1].y);
    }
    return d;
  }

  function paintObstacles() {
    obsEl.replaceChildren();
    if (!obstacles.length) {
      obsEl.append(h('dt', { text: '—' }), h('dd', { text: 'none detected' }));
      return;
    }
    obstacles.slice(0, 8).forEach((o, i) => {
      obsEl.append(h('dt', { text: '#' + (i + 1) }),
                   h('dd', { text: `${o.x.toFixed(0)}, ${o.y.toFixed(0)} mm  `
                     + `(${o.w.toFixed(0)}x${o.d.toFixed(0)}, ${o.n} pts)` }));
    });
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
    if (st.pose_topic) poseTopic = st.pose_topic;
    if (st.pose_type) poseType = st.pose_type;
    if (st.arena && !arena) { arena = st.arena; draw(); }
    if (st.markers && !markers.length) { markers = st.markers; paintMarkers(); draw(); }
    if (st.body_mm && !body) { body = st.body_mm; draw(); }
    if (!rosWired && st.rosbridge) { rosWired = true; wireRos(st.rosbridge); }

    const cp = currentPose();
    warnBox.textContent = cp.live
      ? `pose LIVE (odometry)  ${cp.x.toFixed(0)}, ${cp.y.toFixed(0)} mm  ${cp.hdg.toFixed(0)}°`
      : 'pose ASSUMED = HOME (odometry not arriving)';
    warnBox.style.color = cp.live ? '' : 'var(--warn)';
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
      if (unsubPose) { unsubPose(); unsubPose = null; }
      if (rosUrl) { release(rosUrl, onStatus); rosUrl = null; }
    },
  };
}
