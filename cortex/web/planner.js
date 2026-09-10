/* planner.js - grid A* over the arena, for Rover 1.
 *
 * CONSTANTS ARE ROVER 2'S, from INTERNATIONALS_ROVER2.py, not invented:
 *   GRID_RES 50 mm       - it carries the note "was 100: only ~6 cells across
 *                          a 650mm arena", i.e. 100 was too coarse to plan on.
 *   ROUTE_CLEARANCE 90   - and that file flags its own value as "too tight for
 *                          230mm chassis". It is exposed here so it can be
 *                          raised rather than inherited blindly.
 *
 * TWO FAILURE MODES THIS HANDLES ON PURPOSE
 *
 * 1. THE GOAL IS OUTSIDE THE DRIVEABLE ARENA. The zones sit beyond the arena
 *    by design - M1/M2 are at y=1357.5 against an arena 1200 deep - so a path
 *    to a zone centre does not exist and never will. Planning "fails" for a
 *    reason that is geometry, not search. So the planner approaches to a
 *    STANDOFF and reports the remaining distance instead of returning nothing.
 *
 * 2. GOAL INSIDE INFLATION. The Rover 2 notes record Nav2 STATUS_ABORTED
 *    failures from exactly this. Rather than fail, the goal is relocated to
 *    the nearest free cell and the relocation distance is reported, so the
 *    operator sees that the goal moved and by how much.
 */

export const GRID_RES = 50;          // mm per cell
export const ROUTE_CLEARANCE = 90;   // mm, on top of the robot radius

/** Build an occupancy grid over the arena. Returns {cols,rows,blocked,idx}. */
export function buildGrid(arena, obstacles, markers, robotRadius, opts = {}) {
  const clearance = opts.clearance == null ? ROUTE_CLEARANCE : opts.clearance;
  const inflate = robotRadius + clearance;
  const cols = Math.floor(arena.w / GRID_RES) + 1;
  const rows = Math.floor(arena.h / GRID_RES) + 1;
  const blocked = new Uint8Array(cols * rows);
  const idx = (c, r) => r * cols + c;
  const cx = (c) => c * GRID_RES;
  const cy = (r) => r * GRID_RES;

  // The rover CENTRE may not come within edge_keep of a wall, or part of the
  // body leaves the floor. That is a harder bound than the inflation.
  const ek = arena.edge_keep;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const x = cx(c), y = cy(r);
      if (x < ek || x > arena.w - ek || y < ek || y > arena.h - ek) blocked[idx(c, r)] = 1;
    }
  }

  const stamp = (px, py, radius) => {
    const rad = Math.ceil(radius / GRID_RES);
    const c0 = Math.round(px / GRID_RES), r0 = Math.round(py / GRID_RES);
    for (let dr = -rad; dr <= rad; dr++) {
      for (let dc = -rad; dc <= rad; dc++) {
        const c = c0 + dc, r = r0 + dr;
        if (c < 0 || r < 0 || c >= cols || r >= rows) continue;
        if (Math.hypot(dc * GRID_RES, dr * GRID_RES) <= radius) blocked[idx(c, r)] = 1;
      }
    }
  };

  // Detected obstacles: half their measured extent, plus inflation.
  for (const o of obstacles || []) {
    stamp(o.x, o.y, Math.max(o.w, o.d) / 2 + inflate);
  }
  // The vases ARE solid. The lake is NOT - it is somewhere the rover goes.
  for (const m of markers || []) {
    if (m.kind === 'zone') stamp(m.x, m.y, (arena.zone_radius || 131) + inflate);
  }
  return { cols, rows, blocked, idx, inflate };
}

function nearestFree(g, c0, r0) {
  if (!g.blocked[g.idx(c0, r0)]) return [c0, r0];
  for (let ring = 1; ring < Math.max(g.cols, g.rows); ring++) {
    for (let dr = -ring; dr <= ring; dr++) {
      for (let dc = -ring; dc <= ring; dc++) {
        if (Math.max(Math.abs(dc), Math.abs(dr)) !== ring) continue;
        const c = c0 + dc, r = r0 + dr;
        if (c < 0 || r < 0 || c >= g.cols || r >= g.rows) continue;
        if (!g.blocked[g.idx(c, r)]) return [c, r];
      }
    }
  }
  return null;
}

/**
 * A* from start to goal, both in arena mm.
 * Returns { path, status, note, movedGoalMm, standoffMm }.
 */
export function plan(arena, grid, startMm, goalMm) {
  const g = grid;
  const toCell = (p) => [Math.round(p.x / GRID_RES), Math.round(p.y / GRID_RES)];
  const toMm = (c, r) => ({ x: c * GRID_RES, y: r * GRID_RES });

  let [sc, sr] = toCell(startMm);
  sc = Math.max(0, Math.min(g.cols - 1, sc));
  sr = Math.max(0, Math.min(g.rows - 1, sr));

  // The rover may legitimately be standing inside inflation - it is a real
  // object, not a planning error. Free its own cell rather than refuse, but
  // REPORT how far the start had to move: a route that silently begins
  // somewhere other than the rover is a lie, and on a 1 m arena with heavy
  // inflation the relocation can be most of the field.
  const startFree = nearestFree(g, sc, sr);
  if (!startFree) return { path: [], status: 'no free cell near the rover' };
  const movedStart = Math.hypot((startFree[0] - sc) * GRID_RES, (startFree[1] - sr) * GRID_RES);
  [sc, sr] = startFree;

  // Goal outside the driveable arena is GEOMETRY, not search failure.
  const outside = goalMm.x < 0 || goalMm.x > arena.w || goalMm.y < 0 || goalMm.y > arena.h;
  let gc = Math.max(0, Math.min(g.cols - 1, Math.round(goalMm.x / GRID_RES)));
  let gr = Math.max(0, Math.min(g.rows - 1, Math.round(goalMm.y / GRID_RES)));
  const goalFree = nearestFree(g, gc, gr);
  if (!goalFree) return { path: [], status: 'no free cell near the goal' };
  const movedGoal = Math.hypot((goalFree[0] - gc) * GRID_RES, (goalFree[1] - gr) * GRID_RES);
  [gc, gr] = goalFree;

  const N = g.cols * g.rows;
  const gScore = new Float32Array(N).fill(Infinity);
  const came = new Int32Array(N).fill(-1);
  const open = [];
  const hEst = (c, r) => Math.hypot(c - gc, r - gr);
  const sIdx = g.idx(sc, sr);
  gScore[sIdx] = 0;
  open.push([hEst(sc, sr), sIdx]);
  const D = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
             [1, 1, Math.SQRT2], [1, -1, Math.SQRT2],
             [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  const gIdx = g.idx(gc, gr);
  let found = false;
  while (open.length) {
    open.sort((a, b) => a[0] - b[0]);          // small grid; a heap is overkill
    const [, cur] = open.shift();
    if (cur === gIdx) { found = true; break; }
    const cc = cur % g.cols, cr = (cur - cc) / g.cols;
    for (const [dc, dr, w] of D) {
      const nc = cc + dc, nr = cr + dr;
      if (nc < 0 || nr < 0 || nc >= g.cols || nr >= g.rows) continue;
      const ni = g.idx(nc, nr);
      if (g.blocked[ni]) continue;
      // No corner-cutting: a diagonal needs both orthogonals clear, or the
      // path clips the corner of an inflated obstacle.
      if (dc && dr && (g.blocked[g.idx(cc + dc, cr)] || g.blocked[g.idx(cc, cr + dr)])) continue;
      const t = gScore[cur] + w;
      if (t < gScore[ni]) {
        gScore[ni] = t; came[ni] = cur;
        open.push([t + hEst(nc, nr), ni]);
      }
    }
  }
  if (!found) return { path: [], status: 'no route', movedGoalMm: movedGoal };

  const cells = [];
  for (let i = gIdx; i !== -1; i = came[i]) {
    const c = i % g.cols, r = (i - c) / g.cols;
    cells.push(toMm(c, r));
    if (i === sIdx) break;
  }
  cells.reverse();

  const path = simplify(cells, g);
  const endMm = path[path.length - 1];
  const standoff = Math.hypot(endMm.x - goalMm.x, endMm.y - goalMm.y);
  return {
    path,
    status: 'ok',
    outsideArena: outside,
    movedGoalMm: movedGoal,
    movedStartMm: movedStart,
    standoffMm: standoff,
  };
}

/** Drop waypoints that a straight line already covers. */
function simplify(cells, g) {
  if (cells.length < 3) return cells;
  const clear = (a, b) => {
    const n = Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (GRID_RES / 2));
    for (let i = 0; i <= n; i++) {
      const x = a.x + (b.x - a.x) * i / n, y = a.y + (b.y - a.y) * i / n;
      const c = Math.round(x / GRID_RES), r = Math.round(y / GRID_RES);
      if (c < 0 || r < 0 || c >= g.cols || r >= g.rows) return false;
      if (g.blocked[g.idx(c, r)]) return false;
    }
    return true;
  };
  const out = [cells[0]];
  let i = 0;
  while (i < cells.length - 1) {
    let j = cells.length - 1;
    while (j > i + 1 && !clear(cells[i], cells[j])) j--;
    out.push(cells[j]); i = j;
  }
  return out;
}
