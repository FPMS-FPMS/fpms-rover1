/*
 * FPMS Cortex - the shell.
 *
 * Loads the panels, holds the one connection to the backend, and owns the
 * background. Panels know nothing about each other or about transport; they
 * get mount(root, api) and are told when state changes.
 *
 * WHY ONE CONNECTION AND NOT ONE PER PANEL
 * ----------------------------------------
 * Twelve panels each polling /api/state would be twelve HTTP round trips per
 * tick on a 2-core machine that is also running ROS. There is a single SSE
 * stream; the shell fans it out. A panel that is not visible is not updated at
 * all - update() is only called on the mounted panel - so a hidden 3D view
 * costs nothing.
 *
 * WHY ONE ANIMATION LOOP
 * ----------------------
 * Same argument, one layer down. `motion.js` owns the app's only
 * requestAnimationFrame loop; the shell subscribes to it and unsubscribes the
 * moment nothing is moving, so idle really is idle. Panel transitions, the
 * rail marker and the boot sequence are CSS transitions on transform and
 * opacity, which run on the compositor thread and therefore keep running
 * smoothly while this thread is parsing a 230 KB telemetry frame.
 *
 * THE CONTRACT PANELS IMPLEMENT
 *   export function mount(rootElement, api) -> { update(state), destroy() }
 *   api.call(panel, action, params) -> Promise<result>   (see call(), below)
 *   api.state                        -> latest full telemetry
 *   api.on(event, fn)                -> 'state' | 'online' | 'offline'
 *   api.motion                       -> the shared motion primitives
 */

import {
  frames, Sparkline, RingBuffer, rollNumber, cancelRoll,
  enter, exit, reducedMotion, onReducedMotion, clamp, setKids, setDisabled,
} from './motion.js';

import * as motion from './motion.js';

const PALETTE = {
  bg: '#090a16', panel: '#15132b', line: '#2c2a52',
  orange: '#a97bff', cyan: '#62a8ff', text: '#eaeaf8', muted: '#8d8cba',
  ok: '#4fe8b4', warn: '#ffcf5c', bad: '#ff5c8a',
};

/* Panels, in the order they appear in the rail. `needs` names the backend
 * panel key; a panel whose backend is absent is shown greyed rather than
 * hidden, so a missing collector is visible instead of silently dropping a
 * feature. */
const PANELS = [
  /* TRIMMED 2026-09-04: cut to Overview and Accel because the browser
   * was the CPU load, not the backend - cortex_core sits at
   * 1.5%. Seven modules drive requestAnimationFrame loops
   * (viz3d, camera, leds, motion, usb, glitchbg, app), and each
   * one that is registered is one the app can load and animate.
   * The panel FILES are untouched; this is the registry only.
   * Put them all back with:  sudo python3 trim_cortex.py --restore
   * Dropped: ai, viz3d, power, tasks, storage, motion, experimental, net, display, sensors, services, leds, audio, camera, usb, appearance */
  { id: 'overview',     label: 'Overview',    glyph: '◉', mod: null,                    needs: null },
  /* Rover 2 - merged in from the standalone ground-station dashboard so this
   * machine runs ONE application. They sit directly under Overview because
   * this is a ground station first and a laptop second: the robot is what
   * the operator opens the app to look at. */
  /* Rover 1 - the Orange Pi 5 Max. Added 2026-09-10. These three sit above
   * the Rover 2 group because Rover 1 is the machine currently being brought
   * up; reorder freely, the rail is built from this array alone.
   *   control  link, health and the device inventory, from fpms-rover-bridge
   *   lidar    D500 scan, straight from the rover over rosbridge (see the
   *            header of panel_lidar.js for why it does not come through
   *            panel state)
   *   cameras  the Thermal Master P1 over ROS, and the ground station's own
   *            webcam. Rover 1 has no camera fitted. */
  { id: 'control',      label: 'Control',     glyph: '◈', mod: './panel_control.js',   needs: 'control' },
  { id: 'lidar',        label: 'LiDAR',       glyph: '◎', mod: './panel_lidar.js',     needs: 'lidar' },
  { id: 'cameras',      label: 'Cameras',     glyph: '▣', mod: './panel_cameras.js',   needs: 'cameras' },
  { id: 'rover',        label: 'Rover',       glyph: '⬢', mod: './panel_rover.js',     needs: 'rover' },
  { id: 'mission',      label: 'Mission',     glyph: '▶', mod: './panel_mission.js',   needs: 'mission' },
  /* 3D Field removed 2026-09-10 at the operator's request. The panel and
   * its backend are LEFT IN PLACE - only the registry entry is gone, so
   * restoring it is one line rather than a recovery. */
  { id: 'accel',        label: 'Accel',       glyph: '⧗', mod: './panel_accel.js',      needs: 'accel' },
  /* CPU/power put BACK 2026-09-04: the trim above dropped it, but this is the
   * panel that shows package power, per-core clocks, governor, EPP and the
   * thermal ceiling - the thing you open when the machine feels slow. Its
   * backend is cheap; the animation cost the trim was chasing was in viz3d,
   * camera, leds and glitchbg, which stay out. */
  { id: 'power',        label: 'CPU',         glyph: '⚡', mod: './panel_power.js',      needs: 'power' },
];

/* How long the intro is allowed to linger once data has arrived. The intro
 * never blocks input (it is pointer-events:none over the stage only), so this
 * is purely so a fast backend does not produce a one-frame flash of the
 * wordmark. If the first frame takes longer than this - it usually does - the
 * intro ends the instant that frame lands. */
const BOOT_MIN_MS = 350;
/* No frame by now and the intro stops pretending: it stalls, in red, and says
 * what it is waiting for. */
const BOOT_STALL_MS = 1800;
/* And by now it gets out of the way entirely, leaving the banner to carry the
 * bad news, because a permanent splash screen is not a diagnosis. */
const BOOT_GIVEUP_MS = 5200;
/* Do not flap the offline chrome on a single dropped SSE beat. */
const OFFLINE_GRACE_MS = 900;
/* Telemetry history retained for the sparklines, in samples. The stream runs
 * at 1 Hz idle / 4 Hz with a subscriber, so 90 samples is 22 s - 90 s of
 * history: enough to see a spike arrive and decay, small enough that the
 * whole set is 4 * 90 * 4 bytes. */
const HIST = 90;

/* Thresholds that mean something on THIS part, not generic ones. The i7-7500U
 * throttles at 95 °C and under a sustained load with the 25 W firmware limit
 * the package sits in the high 80s, so 86 is "bad" and 92 is "about to lose
 * clock". */
function levelCpu(v) { return v == null ? 'idle' : v >= 90 ? 'bad' : v >= 70 ? 'warn' : 'ok'; }
function levelTemp(v) {
  if (v == null) return 'idle';
  return v >= 92 ? 'crit' : v >= 86 ? 'bad' : v >= 70 ? 'warn' : 'ok';
}
function levelMem(v) { return v == null ? 'idle' : v >= 92 ? 'bad' : v >= 80 ? 'warn' : 'ok'; }
/* PSI is "how long something was stalled waiting", so any sustained non-zero
 * is worth seeing; 0-5 is normal noise on this machine. */
function levelPsi(v) { return v == null ? 'idle' : v >= 50 ? 'bad' : v >= 20 ? 'warn' : v >= 5 ? 'ok' : 'idle'; }

const LEVEL_CLASSES = ['lv-idle', 'lv-ok', 'lv-warn', 'lv-bad', 'lv-crit'];

function setLevel(el, lv) {
  if (!el) return;
  const want = 'lv-' + lv;
  if (el.dataset.lv === want) return;      // no class churn at telemetry rate
  el.dataset.lv = want;
  el.classList.remove(...LEVEL_CLASSES);
  el.classList.add(want);
}

class Cortex {
  constructor() {
    /* null, NOT {}. An empty object is truthy and looks like a valid payload,
     * so a panel mounted before the first SSE frame would render 'no devices'
     * rather than 'waiting'. A USB panel painted an empty bus forever because
     * of exactly this. */
    this.state = null;
    this.hasState = false;
    this.listeners = { state: [], online: [], offline: [] };
    this.current = null;
    this.currentId = null;
    this.modules = new Map();
    this.bg = null;
    this.online = false;
    this.lastTick = 0;
    this.lastFrameWall = 0;

    /* Panels get the motion primitives through the api object they already
     * receive, so nothing has to import a second module or reach for a
     * global to stay in step with the shell's loop. */
    this.motion = motion;
    this.frames = frames;
    this.reducedMotion = reducedMotion;

    /* One ring buffer per figure, filled by the shell, read by every
     * sparkline that wants it. A panel mounted five minutes in gets the whole
     * history immediately instead of starting from a blank trace. */
    this.hist = {
      cpu: new RingBuffer(HIST),
      temp: new RingBuffer(HIST),
      mem: new RingBuffer(HIST),
      psi: new RingBuffer(HIST),
    };
    this.sparks = [];          // every live Sparkline, redrawn once per frame

    this.bootStart = 0;
    this.bootEl = null;
    this.bootState = 'link';
    this.offlineSince = 0;
    this.alerting = false;
  }

  /* ------------------------------------------------------------ transport */

  /**
   * Run a panel action.
   *
   * WHY THIS UNWRAPS, AND WHY THE OUTER `ok` IS NOT THE ANSWER
   * ---------------------------------------------------------
   * /api/action answers with an envelope: `{ok, panel, action, result}`. That
   * outer `ok` is the DISPATCH's ok - "a panel by that name existed and an
   * action by that name ran". It is true for a refusal, because a refusal is
   * a successful dispatch. The action's own verdict lives in `result.ok`
   * beside its `error`. A panel that checks only the outer `ok` therefore
   * renders every refusal as a success, which is exactly the bug that shipped
   * in the tasks panel and which storage and net each grew their own
   * unwrapping code to avoid.
   *
   * So the shell does it once, here:
   *   - resolves to `result` (or the whole body when there is no `result`)
   *   - THROWS when either ok is false, with the message the backend gave
   *
   * The thrown Error carries `.result`, `.envelope` and `.detail` so a caller
   * that wants the refusal's structure still has it.
   */
  async call(panel, action, params = {}) {
    const r = await fetch('/api/action', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ panel, action, params }),
    });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const body = await r.json();
    const result = (body && body.result !== undefined) ? body.result : body;

    const outerBad = body && body.ok === false;
    const innerBad = result && typeof result === 'object' && result.ok === false;
    if (outerBad || innerBad) {
      const src = innerBad ? result : body;
      const msg = src.error || src.message || src.detail
        || (outerBad ? `${panel}.${action} was not dispatched`
                     : `${panel}.${action} refused`);
      const err = new Error(String(msg));
      err.result = result;
      err.envelope = body;
      err.detail = src.detail || null;
      err.panel = panel;
      err.action = action;
      throw err;
    }
    return result;
  }

  on(ev, fn) { (this.listeners[ev] ||= []).push(fn); }
  emit(ev, arg) { (this.listeners[ev] || []).forEach((f) => { try { f(arg); } catch (e) { console.error(e); } }); }

  connect() {
    /* SSE, not a websocket: the backend is stdlib Python and SSE is a plain
     * HTTP response it can write forever. EventSource also reconnects on its
     * own, which is the behaviour we want when the backend restarts. */
    const es = new EventSource('/api/stream');
    es.onopen = () => { this.setOnline(true); this.bootStatus('link'); };
    es.onerror = () => { this.setOnline(false); };
    /* Applied by both frame types. MEASURED: a full snapshot is ~220 KB and
     * ~70% of it does not change between ticks, so the backend sends only the
     * panels that moved - a median 9.1 KB delta, 23.9x less data. The browser
     * had to parse every one of those 220 KB to update the single panel it is
     * displaying, which is why the dashboard's Chrome processes were the
     * largest CPU consumer on this machine. */
    const applyFrame = (s, first) => {
      this.hasState = true;
      this.lastTick = performance.now();
      this.lastFrameWall = Date.now();
      this.setOnline(true);
      if (first) {
        document.body.classList.remove('nodata');
        this.endBoot();
      }
      this.emit('state', s);
      /* Only the visible panel is updated. A hidden panel that kept redrawing
       * would cost exactly as much as a visible one. */
      if (this.current && this.current.update) {
        try { this.current.update(s); } catch (e) { console.error('panel update', e); }
      }
      this.updateChrome(s);
    };

    const onState = (ev) => {
      let s;
      try { s = JSON.parse(ev.data); } catch { return; }
      this.state = s;
      this.lastSeq = (typeof s.seq === 'number') ? s.seq : null;
      const first = !this.hasState;
      applyFrame(s, first);
    };

    /* A delta carries only the panels that changed, plus base_seq: the seq it
     * must be applied ON TOP OF. If that does not match what we hold, we have
     * missed a frame and merging would silently produce a state that never
     * existed - so drop the connection instead and take a fresh full snapshot.
     * Being briefly disconnected is recoverable; showing a blend of two ticks
     * as if it were live is not. */
    const onDelta = (ev) => {
      let d;
      try { d = JSON.parse(ev.data); } catch { this.resync('delta parse failed'); return; }
      if (!this.hasState || !this.state) { this.resync('delta before any snapshot'); return; }
      if (typeof d.base_seq === 'number' && this.lastSeq !== null
          && d.base_seq !== this.lastSeq) {
        this.resync('delta base ' + d.base_seq + ' != held ' + this.lastSeq);
        return;
      }
      const merged = Object.assign({}, this.state, d);
      merged.panels = Object.assign({}, this.state.panels || {}, d.panels || {});
      if (Array.isArray(d.dropped)) for (const n of d.dropped) delete merged.panels[n];
      delete merged.delta;
      delete merged.dropped;
      delete merged.base_seq;
      this.state = merged;
      this.lastSeq = (typeof d.seq === 'number') ? d.seq : null;
      applyFrame(merged, false);
    };

    /* The backend labels its frames `event: state`. EventSource fires
     * onmessage ONLY for unnamed events, so a named stream needs an
     * explicit listener. Bind both: onmessage covers a future unnamed
     * frame, the listener covers the one the backend actually sends. */
    es.onmessage = onState;
    es.addEventListener('state', onState);
    es.addEventListener('delta', onDelta);
    this.es = es;
  }

  /* Drop the stream and let connect() take a fresh full snapshot. Used only
   * when a delta cannot be trusted; the backend always sends a full frame to
   * a newly connected client. */
  resync(why) {
    console.warn('cortex: resync -', why);
    try { if (this.es) this.es.close(); } catch (e) { /* already closed */ }
    this.es = null;
    this.lastSeq = null;
    setTimeout(() => { if (!this.es) this.connect(); }, 250);
  }

  setOnline(v) {
    if (v === this.online) return;
    this.online = v;
    document.body.classList.toggle('offline', !v);
    this.offlineSince = v ? 0 : performance.now();
    const dot = document.getElementById('linkdot');
    if (dot) {
      dot.style.background = v ? PALETTE.ok : PALETTE.bad;
      dot.title = v ? 'backend connected' : 'backend not responding';
    }
    /* Sparklines keep their data but stop claiming to be live. A flat trace
     * from a dead stream and a flat trace from a quiet machine must not look
     * the same. */
    for (const s of this.sparks) s.setStale(!v);
    /* The background answers too: a lost link drops the field to a cold, low
     * idle instead of glitching happily as if nothing had happened. */
    if (this.bg && this.bg.setFault) this.bg.setFault(!v);
    if (this.bg && this.bg.pulse) this.bg.pulse(v ? 0.30 : 0.55);
    if (!v) this.bootStatus('lost');
    this.refreshAlert();
    this.emit(v ? 'online' : 'offline');
  }

  /* ---------------------------------------------------- connection banner */

  /* One strip that carries the two states the operator must never confuse
   * with "everything is fine": nothing has arrived yet, and it used to arrive
   * and stopped. It slides in on transform only; the stage gains padding at
   * the same time so the banner covers no content. That padding write is a
   * layout pass, which is why it is bound to a connection change (rare) and
   * never to a telemetry frame. */
  refreshAlert() {
    const body = document.body;
    const nodata = !this.hasState;
    const offlineFor = this.offlineSince ? performance.now() - this.offlineSince : 0;
    const bootFor = this.bootStart ? performance.now() - this.bootStart : 0;

    const want = (!this.online && offlineFor > OFFLINE_GRACE_MS)
              || (nodata && bootFor > BOOT_STALL_MS);
    if (want !== this.alerting) {
      this.alerting = want;
      body.classList.toggle('alerting', want);
    }
    if (!want) return;

    const msg = document.getElementById('alert-msg');
    const age = document.getElementById('alert-age');
    if (msg) {
      msg.textContent = nodata
        ? 'no telemetry — the backend has not sent a frame'
        : 'backend offline — figures below are the last known values';
    }
    if (age) {
      age.textContent = this.lastFrameWall
        ? 'last frame ' + fmtAge((Date.now() - this.lastFrameWall) / 1000) + ' ago'
        : 'never connected';
    }
  }

  /* ------------------------------------------------------- boot sequence */

  /* An honest intro. It covers the STAGE only - the rail and the title bar
   * are live and clickable from the first paint - and it is pointer-events:
   * none, so "never delays interaction" is enforced by the layout rather than
   * promised in a comment. It ends when data lands, stalls in red when data
   * does not, and removes itself either way. */
  startBoot() {
    this.bootEl = document.getElementById('boot');
    this.bootStart = performance.now();
    if (!this.bootEl) return;
    this._bootStall = setTimeout(() => {
      if (this.hasState) return;
      this.bootStatus('stalled');
      this.refreshAlert();
    }, BOOT_STALL_MS);
    this._bootGiveUp = setTimeout(() => {
      if (this.hasState) return;
      this.endBoot(true);
    }, BOOT_GIVEUP_MS);
  }

  bootStatus(kind) {
    const el = this.bootEl;
    if (!el || this.bootState === kind) return;
    this.bootState = kind;
    const st = document.getElementById('boot-status');
    const sub = document.getElementById('boot-sub');
    if (kind === 'link') {
      if (st) st.textContent = 'link up · awaiting first frame';
      if (sub) sub.textContent = 'the sampler is measuring; the first frame carries every panel';
    } else if (kind === 'stalled') {
      el.classList.add('stalled');
      if (st) st.textContent = 'backend not responding';
      if (sub) sub.textContent = 'no frame from cortex_core. The shell is usable; every figure is unknown, not zero.';
    } else if (kind === 'lost') {
      if (!this.hasState) {
        el.classList.add('stalled');
        if (st) st.textContent = 'no link to the backend';
        if (sub) sub.textContent = 'the stream would not open. Check that cortex_core.py is running.';
      }
    }
  }

  endBoot(giveUp = false) {
    const el = this.bootEl;
    if (!el) return;
    this.bootEl = null;
    clearTimeout(this._bootStall);
    clearTimeout(this._bootGiveUp);
    const held = performance.now() - this.bootStart;
    /* Only ever holds when the backend beat the intro, which it rarely does.
     * It never holds input - the overlay does not take pointer events. */
    const wait = giveUp || reducedMotion() ? 0 : Math.max(0, BOOT_MIN_MS - held);
    setTimeout(() => {
      el.classList.add('gone');
      /* The identity resolves INTO the app: the background gets a kick at the
       * same moment so the two reads as one event rather than a fade over a
       * static picture. */
      if (this.bg && this.bg.pulse && !giveUp) this.bg.pulse(0.45);
      setTimeout(() => el.remove(), reducedMotion() ? 0 : 220);
    }, wait);
  }

  /* --------------------------------------------------------------- chrome */

  /* Registers a sparkline so the shell redraws it when a frame arrives.
   * Returns an unregister function - panels MUST call it in destroy() or the
   * shell keeps drawing into a detached canvas. */
  addSpark(spark) {
    this.sparks.push(spark);
    spark.setStale(!this.online);
    spark.render();
    return () => {
      const i = this.sparks.indexOf(spark);
      if (i >= 0) this.sparks.splice(i, 1);
    };
  }

  buildHudSparks() {
    const mk = (id, buf, line, fill, baseline) => {
      const c = document.getElementById(id);
      if (!c) return;
      const s = new Sparkline(c, {
        data: buf, width: 46, height: 14, min: 0, max: 100,
        line, fill, baseline,
      });
      this.sparks.push(s);
    };
    mk('spark-cpu', this.hist.cpu, PALETTE.cyan, 'rgba(98,168,255,0.16)', null);
    /* The package trace is scaled 30-100 °C with a rule at 95: the throttle
     * point is on the picture, so "close to it" is something you can see
     * rather than something you have to remember. */
    const t = document.getElementById('spark-temp');
    if (t) {
      this.sparks.push(new Sparkline(t, {
        data: this.hist.temp, width: 46, height: 14, min: 30, max: 100,
        line: PALETTE.orange, fill: 'rgba(169,123,255,0.14)', baseline: 95,
      }));
    }
    mk('spark-mem', this.hist.mem, PALETTE.cyan, 'rgba(98,168,255,0.16)', null);
    mk('spark-psi', this.hist.psi, PALETTE.warn, 'rgba(255,207,92,0.14)', null);
  }

  updateChrome(s) {
    if (!s) return;
    const p = s.panels || {};
    const cpu = s.cpu || {};
    const th = s.thermal || {};
    const mem = s.memory || {};
    const t = th.package_c ?? th.package ?? null;
    const c = cpu.usage_pct ?? null;
    const m = mem.used_pct ?? null;

    /* PSI drives the background. Utilisation says a core is busy; PSI says
     * something is STALLED WAITING, which is what actually feels bad - so
     * that is what makes the glitch escalate. */
    const psi = (p.experimental && p.experimental.psi) || p.psi;
    const psiCpu = psi ? (psi.cpu?.some?.avg10 ?? 0) : null;
    const psiIo = psi ? (psi.io?.some?.avg10 ?? 0) : null;
    let pressure = 0;
    if (psi) {
      const mm = psi.memory?.some?.avg10 ?? 0;
      pressure = Math.min(1, (psiCpu + psiIo * 1.5 + mm * 3) / 60);
    }

    /* History first, then one render pass over every registered sparkline.
     * Each canvas is 46x14 CSS px and is drawn at the telemetry rate (1-4 Hz),
     * never from rAF: a trace that redraws 60 times a second to show a number
     * that changes four times a second is 56 wasted draws. */
    this.hist.cpu.push(c ?? 0);
    this.hist.temp.push(t ?? 0);
    this.hist.mem.push(m ?? 0);
    this.hist.psi.push(psiCpu ?? 0);
    for (const sp of this.sparks) sp.render();

    /* Figures roll rather than snap. rollNumber writes textContent only when
     * the formatted string changes and the elements are tabular-nums, so a
     * digit change never reflows a neighbour. */
    const eCpu = document.getElementById('hud-cpu');
    const eTemp = document.getElementById('hud-temp');
    const eMem = document.getElementById('hud-mem');
    const ePsi = document.getElementById('hud-psi');

    rollNumber(eCpu, c, { format: (v) => Math.round(v) + '%' });
    rollNumber(eTemp, t, { format: (v) => Math.round(v) + '°' });
    rollNumber(eMem, m, { format: (v) => Math.round(v) + '%' });
    /* Two numbers in one slot: roll the one that moves, print the other. */
    rollNumber(ePsi, psiCpu, {
      format: (v) => (Math.round(v * 10) / 10) + ' / '
        + (psiIo == null ? '--' : (Math.round(psiIo * 10) / 10)),
    });

    setLevel(eCpu, levelCpu(c));
    setLevel(eTemp, levelTemp(t));
    setLevel(eMem, levelMem(m));
    setLevel(ePsi, levelPsi(psiCpu));
    if (eTemp) {
      eTemp.title = t == null ? 'package temperature unavailable'
        : `package ${t.toFixed(1)} °C — this part throttles at 95 °C`;
    }

    /* NOT ANIMATED, deliberately: the link dot used to get a WAAPI pulse on
     * every frame. Four new compositor animations a second on an element with
     * a box-shadow measured as real GPU time on the HD 620 for information
     * the dot's colour and the banner already carry. The dot is steady when
     * live and has a CSS state animation only when the link is seeking or
     * lost - which are the states worth looking at. */

    if (this.bg && this.bg.setIntensity) this.bg.setIntensity(0.25 + 0.75 * pressure);
    if (this.bg && this.bg.setLoad) this.bg.setLoad({ cpu: c ?? 0, temp: t ?? 0 });
  }

  /* -------------------------------------------------------------- routing */

  /* Moves the single rail marker. transform only: no width, no top, no
   * per-item background repaint. offsetTop is read once per switch (a forced
   * layout, ~0.05 ms) and never per frame. */
  moveMarker(id) {
    const mark = document.getElementById('railmark');
    const item = document.querySelector(`.rail-item[data-id="${CSS.escape(id)}"]`);
    if (!mark) return;
    if (!item) { mark.classList.remove('on'); return; }
    const h = item.offsetHeight - 12;
    mark.style.height = Math.max(8, h) + 'px';
    mark.style.transform = `translateY(${item.offsetTop + 6}px)`;
    mark.classList.add('on');
  }

  async show(id) {
    if (id === this.currentId) return;
    const def = PANELS.find((p) => p.id === id);
    if (!def) return;

    /* Rapid clicks down the rail must not interleave two mounts into the same
     * stage. Only the newest wins. */
    const token = (this._navToken = (this._navToken || 0) + 1);

    const root = document.getElementById('stage');

    if (this.current && this.current.destroy) {
      try { this.current.destroy(); } catch (e) { console.error(e); }
    }
    this.current = null;
    this.currentId = id;

    document.querySelectorAll('.rail-item').forEach((el) => {
      el.classList.toggle('active', el.dataset.id === id);
      el.setAttribute('aria-current', el.dataset.id === id ? 'page' : 'false');
    });
    this.moveMarker(id);
    /* The background acknowledges the switch. It is one scalar; the shader's
     * cost is flat in energy, so a kick cannot also be a frame-time spike. */
    if (this.bg && this.bg.pulse) this.bg.pulse(0.28);

    /* The stage leaves while the module is being imported, so the transition
     * and the import overlap instead of queueing. */
    const leaving = exit(root, { cls: 'is-leaving', ms: 110 });

    let mod = this.modules.get(def.mod);
    let loadError = null;
    if (def.mod && !mod) {
      try {
        mod = await import(def.mod);
        this.modules.set(def.mod, mod);
      } catch (e) {
        /* A panel that fails to load must not take the shell down. Say which
         * one and why, on screen, rather than leaving a blank pane. */
        loadError = e.message;
      }
    }
    await leaving;
    if (token !== this._navToken) return;

    root.replaceChildren();
    root.scrollTop = 0;

    /* Tell the backend which panel is in front. It splits a fixed slice of one
     * core between fourteen collectors, and without this it splits it evenly -
     * which gave the task manager a 25 s refresh while thirteen panels nobody
     * was looking at spent the rest. Fire-and-forget: a failed hint costs a
     * slower refresh, never a broken panel, so it must not be awaited and must
     * not throw into the navigation path. */
    const backend = PANELS.find((d) => d.id === id);
    fetch('/api/focus?panel=' + encodeURIComponent((backend && backend.needs) || ''),
          { cache: 'no-store' }).catch(() => {});

    if (loadError) {
      root.appendChild(this.errorCard(def.label, loadError));
    } else if (id === 'overview') {
      this.mountOverview(root);
    } else {
      const mount = mod.mount || (mod.default && mod.default.mount);
      if (typeof mount !== 'function') {
        root.appendChild(this.errorCard(def.label,
          'module loaded but exports no mount() - it is incomplete'));
      } else {
        try {
          /* viz3d takes OPTIONS, not the api - it is a renderer driven by
           * update(state), so handing it `this` would have it reading Cortex
           * properties by coincidence. It gets a purpose-built object with
           * the action table; every other panel gets the api. */
          /* viz3d takes OPTIONS, not the api. The appearance panel needs one
           * option too: it will not emit an <img> for a wallpaper unless the
           * host tells it how to turn an absolute path into a URL, and
           * without that every tile rendered as a filename and no picture.
           * The backend serves these from /api/file against an allowlist
           * built from the panel's own state. */
          let arg = this;
          if (id === 'viz3d') arg = { actions: viz3dActions(this) };
          else if (id === 'appearance') arg = appearanceBridge(this);
          this.current = mount(root, arg) || {};
          /* Only push state a panel can trust. Before the first frame arrives
           * it gets nothing and shows its own waiting state. */
          if (this.current.update && this.hasState) this.current.update(this.state);
        } catch (e) {
          root.replaceChildren(this.errorCard(def.label, e.message));
        }
      }
    }

    enter(root, { cls: 'is-entering', ms: 150 });
  }

  errorCard(label, msg) {
    const d = document.createElement('div');
    d.className = 'card';
    const h = document.createElement('h2');
    h.textContent = label + ' could not load';
    const p = document.createElement('p');
    p.className = 'muted';
    p.textContent = msg;                       // textContent: never innerHTML
    d.append(h, p);
    return d;
  }

  /* ------------------------------------------------------------- overview */

  mountOverview(root) {
    const grid = document.createElement('div');
    grid.className = 'grid';

    /* WAS BROKEN, NOW FIXED: this read `st.panels.cpu.total_pct`. `st.panels`
     * is truthy and has no `cpu` key, so every card on the landing page
     * rendered '--' forever while the identical figures in the title bar were
     * correct. The telemetry keys are top level: cpu.usage_pct,
     * thermal.package_c, memory.used_pct/used_mb. */
    const specs = [
      {
        key: 'cpu', title: 'CPU', id: 'ov-cpu',
        sub: 'load across 4 threads',
        pick: (s) => s.cpu?.usage_pct ?? null,
        fmt: (v) => Math.round(v) + '%',
        level: levelCpu, buf: this.hist.cpu,
        line: PALETTE.cyan, fill: 'rgba(98,168,255,0.14)', min: 0, max: 100, base: null,
      },
      {
        key: 'temp', title: 'Package', id: 'ov-temp',
        sub: 'this part throttles at 95 °C',
        pick: (s) => s.thermal?.package_c ?? null,
        fmt: (v) => Math.round(v) + '°C',
        level: levelTemp, buf: this.hist.temp,
        line: PALETTE.orange, fill: 'rgba(169,123,255,0.13)', min: 30, max: 100, base: 95,
      },
      {
        key: 'mem', title: 'Memory', id: 'ov-mem',
        sub: '11.6 GB total, zram 4 GB zstd — trace is % used',
        pick: (s) => (s.memory?.used_mb != null ? s.memory.used_mb / 1024 : null),
        fmt: (v) => v.toFixed(1) + ' GB',
        /* The figure is GB because that is what you compare against a
         * process list; the colour comes from the percentage the kernel
         * actually reports, not from a hard-coded total. */
        level: (v, st) => levelMem(st?.memory?.used_pct ?? null),
        buf: this.hist.mem,
        line: PALETTE.cyan, fill: 'rgba(98,168,255,0.14)', min: 0, max: 100, base: null,
      },
      {
        key: 'psi', title: 'Stall pressure', id: 'ov-psi',
        sub: 'PSI cpu / io — waiting, not just busy',
        pick: (s) => {
          const p = s.panels || {};
          const psi = (p.experimental && p.experimental.psi) || p.psi;
          return psi ? (psi.cpu?.some?.avg10 ?? 0) : null;
        },
        fmt: (v) => v.toFixed(1) + '%',
        level: levelPsi, buf: this.hist.psi,
        line: PALETTE.warn, fill: 'rgba(255,207,92,0.13)', min: 0, max: null, base: null,
      },
    ];

    const views = [];
    const unregs = [];
    for (const sp of specs) {
      const c = document.createElement('div');
      c.className = 'card stat';
      const h = document.createElement('div'); h.className = 'stat-label'; h.textContent = sp.title;
      const v = document.createElement('div'); v.className = 'stat-value'; v.id = sp.id;
      v.textContent = '--';
      const canvas = document.createElement('canvas');
      canvas.className = 'stat-spark';
      const s = document.createElement('div'); s.className = 'muted small'; s.textContent = sp.sub;
      setKids(c, [h, v, canvas, s]);
      grid.appendChild(c);
      views.push({ sp, v, canvas });
    }
    root.appendChild(grid);

    /* The canvas backing store needs a real CSS width, which only exists once
     * the card is in the document. One layout read per card, at mount. */
    for (const view of views) {
      const w = view.canvas.clientWidth || 220;
      const spark = new Sparkline(view.canvas, {
        data: view.sp.buf, width: w, height: 38,
        min: view.sp.min, max: view.sp.max,
        line: view.sp.line, fill: view.sp.fill, baseline: view.sp.base,
      });
      view.spark = spark;
      unregs.push(this.addSpark(spark));
    }

    /* Window resizes are rare and layout-bound; re-measure then, never on a
     * telemetry frame. */
    const onResize = () => {
      for (const view of views) {
        view.spark.resize(view.canvas.clientWidth || 220, 38);
        view.spark.render();
      }
    };
    window.addEventListener('resize', onResize);

    this.current = {
      update: (st) => {
        if (!st) return;
        for (const view of views) {
          const val = view.sp.pick(st);
          rollNumber(view.v, val, { format: view.sp.fmt, tau: 0.14 });
          setLevel(view.v, view.sp.level(val, st));
        }
      },
      destroy: () => {
        window.removeEventListener('resize', onResize);
        for (const u of unregs) u();
        for (const view of views) cancelRoll(view.v);
      },
    };
  }

  /* ----------------------------------------------------------------- boot */
  async start() {
    this.startBoot();

    const rail = document.getElementById('rail');
    const mark = document.getElementById('railmark');
    for (const p of PANELS) {
      const el = document.createElement('button');
      el.className = 'rail-item';
      el.dataset.id = p.id;
      el.type = 'button';
      el.title = p.label;
      const g = document.createElement('span'); g.className = 'glyph'; g.textContent = p.glyph;
      const l = document.createElement('span'); l.className = 'label'; l.textContent = p.label;
      el.append(g, l);
      el.addEventListener('click', () => { location.hash = '#' + p.id; });
      rail.appendChild(el);
    }
    /* Keep the marker as the rail's last child so it paints over nothing. */
    if (mark) rail.appendChild(mark);

    this.buildHudSparks();

    /* The background is decoration, so a failure here must never stop the
     * control panel from working. */
    try {
      /* glitchbg DISABLED 2026-09-02 by operator request. It is a full-screen
       * WebGL quad; with it running Chrome's GPU process measured 7.9% of a
       * core on a 2-physical-core machine whose iGPU is also used for ternary
       * compute. Decoration is not worth a scheduling slice here.
       *
       * NOT deleted - opt back in from the browser console with:
       *   localStorage.setItem('fpms.glitchbg','1'); location.reload();
       * The existing catch turns this into the painted CSS fallback. */
      if (localStorage.getItem('fpms.glitchbg') !== '1') {
        this.bg = null;
        throw new Error('glitchbg off by default (fpms.glitchbg)');
      }
      const bgmod = await import('./glitchbg.js');
      const create = bgmod.createGlitchBackground || bgmod.default;
      const canvas = document.getElementById('bg');
      if (create && canvas) this.bg = create(canvas) || null;
      /* Put the shader's wordmark cycle at the point where it materialises,
       * so the FPMS identity resolves behind the intro instead of arriving 12
       * seconds later to an operator who has moved on. Free: it sets a float.
       * If the context failed, `bg` is still an object with a painted CSS
       * fallback and these are no-ops. */
      if (this.bg && this.bg.markPhase) this.bg.markPhase(0.06);

      /* Feed the background from the storage-resident field instead of making
       * the shader recompute it. Opt-in and failure-proof: if the engine is
       * absent, the endpoint 503s or a fetch fails, the feed reports not-ready
       * and the shader computes the noise exactly as it always did.
       *
       * Only every 8th phase crosses the network - the GPU interpolates
       * between the two keyframes it holds, which is one mix() and free. */
      if (this.bg && this.bg.attachFeed && this.bg.gl) {
        import('./tcorefeed.js')
          .then((m) => this.bg.attachFeed(m.createFeed(this.bg.gl,
                                                       { stride: 8, phasesPerSecond: 12 })))
          .catch(() => {});
      }
    } catch (e) {
      console.warn('glitch background unavailable:', e.message);
      document.getElementById('bg')?.remove();
    }

    /* A live accessibility setting, not one read once at load. */
    onReducedMotion((on) => {
      if (this.bg && this.bg.setCalm) this.bg.setCalm(on);
    });
    if (this.bg && this.bg.setCalm && reducedMotion()) this.bg.setCalm(true);

    this.connect();

    /* Hash routing. Deep links are useful on their own - you can leave the
     * Cortex open on Power - and they are the only way to drive the app from
     * a headless browser, which is how every panel gets screenshot-tested. */
    const fromHash = () => (location.hash || '').replace(/^#\/?/, '') || 'overview';
    window.addEventListener('hashchange', () => this.show(fromHash()));
    await this.show(fromHash());
    window.addEventListener('resize', () => this.moveMarker(this.currentId));

    /* If the backend goes quiet, say so rather than showing stale numbers as
     * if they were live. One second, so the banner's age counter is honest;
     * it does no DOM work at all unless the banner is up. */
    setInterval(() => {
      if (this.lastTick && performance.now() - this.lastTick > 6000) this.setOnline(false);
      if (this.alerting || !this.online || !this.hasState) this.refreshAlert();
    }, 1000);
  }
}

/* ---- what the 3D model can do -------------------------------------------
 * The model knows where the parts are. It does not know what a power limit is,
 * and it must not learn: viz3d.js is a renderer, and the moment it contains a
 * profile name it is a renderer that has to be edited when the backend changes.
 *
 * So the shell hands it a table of {label, hint, run} per part, and every
 * `run` is an ordinary panel action - the same call the Power or Storage panel
 * would make, going through the same validators and the same helper. Clicking
 * the heatpipe on the model and clicking Balanced on the Power panel are the
 * same action; there is no second path into the machine.
 *
 * `run` must THROW on refusal so the menu can print it. api.call resolves to
 * the envelope, so a refusal arrives as result.ok === false and would
 * otherwise render as a success.
 */
function viz3dActions(cortex) {
  const act = async (panel, action, params) => {
    const body = await cortex.call(panel, action, params || {});
    if (!body || body.ok === false) throw new Error((body && body.error) || 'action failed');
    const r = body.result === undefined ? body : body.result;
    if (r && r.ok === false) throw new Error(r.error || 'refused');
    return r;
  };
  const profile = (name, hint) => ({
    label: name[0].toUpperCase() + name.slice(1),
    hint,
    run: () => act('power', 'apply_profile', { profile: name })
      .then((r) => ({ message: 'profile ' + name + ' applied' + (r && r.declared && r.declared.length
        ? ' (' + r.declared.length + ' settings this uid cannot write)' : '') })),
  });

  return {
    /* The heatpipe and exhaust IS the thermal story on this chassis, so it is
     * where the power profiles live. There is no fan control to offer. */
    therm: [
      profile('balanced', 'the default: PL1 at the part\u2019s 15 W nominal'),
      profile('battery', 'PL1 down, turbo off, quietest and coolest'),
      profile('speed', 'PL1 to 20 W and turbo on - needs confirming on the Power panel'),
    ],
    cpu: [
      profile('balanced', 'PL1 15 W, turbo on'),
      { label: 'Open Tasks', hint: 'the process table and the cgroup governor',
        run: async () => { cortex.show('tasks'); return { message: 'Tasks' }; } },
    ],
    /* The stick is the slowest thing in the machine, so its menu is the two
     * things that actually change that, in the order you would try them. */
    stor: [
      { label: 'Check the link', hint: 'USB2 or a UAS fallback is a 5-10x loss no cache recovers',
        run: () => act('storage', 'transport', {})
          .then((r) => ({ message: r && r.summary ? r.summary : 'transport checked' })) },
      { label: 'Measure it', hint: 'random reads against an existing file, caches dropped first',
        run: () => act('storage', 'measure', {})
          .then((r) => ({ message: r && r.iops != null ? Math.round(r.iops) + ' IOPS' : 'measured' })) },
      { label: 'Open Storage', hint: 'warm, pin, zram and the page cache',
        run: async () => { cortex.show('storage'); return { message: 'Storage' }; } },
    ],
    mem: [
      { label: 'Open Storage', hint: 'zram, page cache and what is pinned',
        run: async () => { cortex.show('storage'); return { message: 'Storage' }; } },
      { label: 'Open Tasks', hint: 'per-app memory limits - no root needed for these',
        run: async () => { cortex.show('tasks'); return { message: 'Tasks' }; } },
    ],
    batt: [
      profile('battery', 'the profile that actually extends this'),
      { label: 'Open Power', hint: 'battery health, cycle count and the profiles in full',
        run: async () => { cortex.show('power'); return { message: 'Power' }; } },
    ],
    /* gpu deliberately has NO actions. Both GPUs are blacklisted and kernel
     * lockdown blocks the module; a menu here would be a menu of things that
     * cannot work, and the tooltip already says so. */
  };
}

/* ---- quick controls ------------------------------------------------------
 * Volume, brightness and power profile, on every screen, at all times.
 *
 * These are not a second implementation of anything: each one sends the same
 * action the owning panel sends, through the same validators and the same
 * helper. What they add is that you do not have to be on that panel, and do
 * not have to read anything, to use them.
 *
 * TWO RULES, BOTH LEARNED FROM SLIDERS THAT LIED:
 *
 * 1. A slider you are dragging is NOT updated from telemetry. The panels
 *    refresh on their own schedule; letting a frame land mid-drag snaps the
 *    thumb back under the finger and the value you were setting is lost.
 * 2. The value shown after a write is the one the KERNEL read back, never the
 *    one that was requested. Backlight and volume writes fail silently on this
 *    machine often enough that echoing the request is how a control comes to
 *    show a number that is not true.
 */
function initQuick(cortex) {
  const $ = (id) => document.getElementById(id);
  const vol = $('q-vol'), volV = $('q-vol-v'), mute = $('q-mute');
  const bri = $('q-bri'), briV = $('q-bri-v');
  const volGrp = $('q-vol-grp'), briGrp = $('q-bri-grp'), powGrp = $('q-pow-grp');
  if (!vol || !bri) return;

  let dragging = null;          // which slider the pointer owns right now
  let sinkId = null, muted = false, briWritable = false;

  const send = async (panel, action, params) => {
    const body = await cortex.call(panel, action, params || {});
    if (!body || body.ok === false) throw new Error((body && body.error) || 'action failed');
    const r = body.result === undefined ? body : body.result;
    if (r && r.ok === false) throw new Error(r.error || 'refused');
    return r;
  };

  /* Commit on release and on keyboard change, not on every pixel of a drag:
   * each write is a round trip to a helper that re-reads sysfs, and a drag
   * fires dozens of input events. */
  const onCommit = (el, fn) => {
    el.addEventListener('pointerdown', () => { dragging = el; });
    const done = () => { if (dragging === el) { dragging = null; fn(el.value); } };
    el.addEventListener('pointerup', done);
    el.addEventListener('pointercancel', done);
    el.addEventListener('change', () => { if (dragging !== el) fn(el.value); });
    el.addEventListener('input', () => {
      /* the readout tracks the thumb even though the write has not happened */
      const v = el.value;
      (el === vol ? volV : briV).textContent = v + '%';
    });
  };

  onCommit(vol, async (v) => {
    if (sinkId == null) return;
    try {
      const r = await send('audio', 'set_volume', { id: sinkId, volume: Number(v) / 100 });
      /* set_volume answers with `volume` as the SLIDER SCALE FLOAT (0..1.5),
       * not the object the state frame carries under the same key. Reading
       * r.volume.percent here got undefined and quietly left the readout on
       * whatever the drag had painted - which looks exactly like success. */
      const back = (r && typeof r.volume === 'number') ? r.volume * 100 : null;
      if (back != null) { vol.value = Math.round(back); volV.textContent = Math.round(back) + '%'; }
    } catch (e) { volV.textContent = '!'; volV.title = String(e.message || e); }
  });

  onCommit(bri, async (v) => {
    try {
      const r = await send('display', 'brightness', { percent: Number(v) });
      /* The panel re-reads /sys after the write and returns that as `current`.
       * There is no top-level `percent` on the success path - the whole point
       * of this action is that the number comes back from the kernel rather
       * than being echoed from the request. */
      const pct = r && (r.current && r.current.percent);
      if (pct != null) { bri.value = Math.round(pct); briV.textContent = Math.round(pct) + '%'; }
    } catch (e) { briV.textContent = '!'; briV.title = String(e.message || e); }
  });

  mute.addEventListener('click', async () => {
    if (sinkId == null) return;
    try {
      const r = await send('audio', 'set_mute', { id: sinkId, mute: !muted });
      if (r && r.volume) applyMute(!!r.volume.mute);
    } catch (e) { mute.title = String(e.message || e); }
  });

  function applyMute(m) {
    muted = m;
    mute.classList.toggle('off', m);
    mute.textContent = m ? '\u25cc' : '\u25c9';
    mute.title = m ? 'muted - click to unmute' : 'click to mute';
  }

  for (const b of powGrp ? powGrp.querySelectorAll('.q-p') : []) {
    b.addEventListener('click', async () => {
      const name = b.dataset.profile;
      const was = b.textContent;
      b.disabled = true; b.textContent = '...';
      try {
        await send('power', 'apply_profile', { profile: name });
      } catch (e) {
        /* Speed refuses without an explicit thermal confirmation, by design.
         * Send the operator to the panel that can take it rather than
         * inventing a confirm the backend did not ask this control for. */
        b.title = String(e.message || e);
        cortex.show('power');
      } finally { b.disabled = false; b.textContent = was; }
    });
  }

  cortex.on('state', (st) => {
    const P = (st && st.panels) || {};

    const a = P.audio;
    if (a && Array.isArray(a.sinks)) {
      const sink = a.sinks.find((x) => x.is_default) || a.sinks[0];
      if (sink) {
        sinkId = sink.id;
        applyMute(!!(sink.volume && sink.volume.mute));
        if (dragging !== vol && sink.volume && sink.volume.percent != null) {
          vol.value = Math.round(sink.volume.percent);
          volV.textContent = Math.round(sink.volume.percent) + '%';
        }
        volGrp.classList.remove('gone');
      } else { volGrp.classList.add('gone'); }
    }

    const d = P.display;
    if (d && d.backlight) {
      const bl = d.backlight;
      /* writable_by_us is false on this machine - the write goes through the
       * helper - so the slider is live when EITHER route exists. */
      briWritable = !!(bl.writable_by_us || (d.helper && d.helper.present));
      bri.disabled = !briWritable;
      bri.min = String(Math.max(1, Math.round(bl.floor_pct || 5)));
      if (dragging !== bri && bl.percent != null) {
        bri.value = Math.round(bl.percent);
        briV.textContent = Math.round(bl.percent) + '%';
      }
      bri.title = briWritable ? 'screen brightness'
        : 'no backlight write available: cortex-helper is not answering';
      briGrp.classList.remove('gone');
    }

    const pw = P.power;
    if (pw && powGrp) {
      const cur = pw.active_profile || pw.profile || (pw.current && pw.current.profile);
      for (const b of powGrp.querySelectorAll('.q-p')) {
        b.classList.toggle('on', b.dataset.profile === cur);
      }
    }
  });
}

/* ---- the appearance panel's bridge --------------------------------------
 * This panel was written to run standalone as well as inside the shell, so it
 * documents its own host contract - `getState()`, `call(action, params)`,
 * `subscribe(cb)` - and that is NOT the Cortex's shape: `cortex.call` takes
 * the panel name first. Handing it `this` gave it a two-argument call that
 * was really a three-argument one, so every action went to a panel named
 * after the action. An adapter is the fix; there is no version of passing
 * `this` that is correct.
 *
 * It also refuses to emit an <img> for a wallpaper unless the host says how
 * to turn an absolute path into a URL - deliberately, so that a host with no
 * file route shows names rather than a grid of broken images. This one has a
 * file route, so it gets one.
 */
function appearanceBridge(cortex) {
  return {
    imageUrl: (path) => '/api/file?path=' + encodeURIComponent(path),
    async getState() {
      const st = cortex.state;
      return (st && st.panels) ? st.panels.appearance : null;
    },
    async call(action, params) {
      const body = await cortex.call('appearance', action, params || {});
      if (!body || body.ok === false) throw new Error((body && body.error) || 'action failed');
      const res = body.result === undefined ? body : body.result;
      if (res && res.ok === false) throw new Error(res.error || 'action failed');
      return res;
    },
    subscribe(cb) {
      const fn = (st) => {
        const p = st && st.panels && st.panels.appearance;
        if (p) cb(p);
      };
      cortex.on('state', fn);
      /* cortex.on() returns nothing, but this contract's subscribe() must
       * return the unsubscribe - the panel calls it on destroy, and an
       * undefined there is a TypeError every time you leave the panel, plus a
       * listener that keeps firing into a dead DOM. */
      return () => {
        const list = cortex.listeners.state;
        if (!list) return;
        const i = list.indexOf(fn);
        if (i >= 0) list.splice(i, 1);
      };
    },
  };
}

/* ---- density ----------------------------------------------------------- */
/* SIMPLE is the default, deliberately. The panels were written to teach - most
 * of them carry a paragraph explaining why a control refuses, or what the
 * kernel does with a value - and all of that is worth reading exactly once.
 * Every day after that it is noise between you and the slider, so the app
 * opens with the explanations folded away and the controls at the top.
 *
 * The choice is per-machine and sticky: localStorage, not a query parameter,
 * because whoever set this up is the same person opening it at a venue. A
 * browser that refuses storage (private window, cleared data) still gets a
 * working toggle - it just forgets, which is better than throwing. */
const DENSITY_KEY = 'fpms.cortex.density';

function readDensity() {
  try {
    const v = window.localStorage.getItem(DENSITY_KEY);
    if (v === 'simple' || v === 'full') return v;
  } catch (_) { /* storage denied; the default is still correct */ }
  return 'simple';
}

function applyDensity(mode) {
  const m = mode === 'full' ? 'full' : 'simple';
  document.body.setAttribute('data-density', m);
  const btn = document.getElementById('density');
  if (btn) {
    btn.innerHTML = 'MODE <b></b>';
    btn.lastChild.textContent = m === 'full' ? 'FULL' : 'SIMPLE';
    btn.title = m === 'full'
      ? 'showing every explanation - click for controls only'
      : 'controls only - click to show the explanations';
    btn.setAttribute('aria-pressed', String(m === 'full'));
  }
  try { window.localStorage.setItem(DENSITY_KEY, m); } catch (_) {}
  /* Panels that lay themselves out from measured height (the sparklines, the
   * 3D canvas) need to know the box changed. One event, no panel API. */
  window.dispatchEvent(new CustomEvent('cortex:density', { detail: { mode: m } }));
  return m;
}

function initDensity() {
  let mode = applyDensity(readDensity());
  const btn = document.getElementById('density');
  if (btn) btn.addEventListener('click', () => {
    mode = applyDensity(mode === 'full' ? 'simple' : 'full');
  });
  /* One key, because reaching for the mouse to read a footnote is the thing
   * this is trying to avoid. Not while typing into a field. */
  window.addEventListener('keydown', (e) => {
    if (e.key !== 'd' && e.key !== 'D') return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    const t = e.target;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
    e.preventDefault();
    mode = applyDensity(mode === 'full' ? 'simple' : 'full');
  });
}

function fmtAge(sec) {
  if (!Number.isFinite(sec) || sec < 0) return '—';
  if (sec < 60) return Math.round(sec) + ' s';
  if (sec < 3600) return Math.floor(sec / 60) + ' min ' + Math.round(sec % 60) + ' s';
  return Math.floor(sec / 3600) + ' h ' + Math.floor((sec % 3600) / 60) + ' min';
}

const cortex = new Cortex();
if (typeof window !== 'undefined') window.__cortex = cortex;
window.addEventListener('DOMContentLoaded', () => {
  initDensity();
  initQuick(cortex);
  /* The border light is REMOVED. It was a ring the operator did not want, and
   * every element on screen costs something to composite even when its own
   * animation is free. The moving background (glitchbg) stays - that is the
   * one the operator asked to keep. */
  cortex.start();
});
export default cortex;
export { PALETTE, PANELS, setKids, setDisabled, clamp, applyDensity, readDensity };
