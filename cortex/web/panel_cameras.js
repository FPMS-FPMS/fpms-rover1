/* panel_cameras - CAMERAS panel: thermal and regular.
 *
 * Two feeds are described here and exactly one of them can produce an image
 * today. That asymmetry is shown honestly rather than papered over:
 *
 *   regular   the LAPTOP's built-in webcam (/dev/video0 on the ground
 *             station). Rover 1 has no camera fitted - its only video nodes
 *             are the RK3588 hardware codecs. The panel says so rather than
 *             letting the obvious assumption stand.
 *
 *   thermal   a Thermal Master P1 IS plugged into the rover, but no kernel
 *             driver binds it (vendor class 255/240, not UVC), so there is no
 *             frame to fetch. This panel renders an explicit "no driver"
 *             state - never a placeholder image, a spinner that never
 *             resolves, or a frozen last frame, any of which could be
 *             mistaken for a live feed.
 *
 * Frames arrive as base64 through api.call, not from an <img src> pointed at
 * a stream: cortex_core serves only its own endpoints and panels cannot add
 * HTTP routes, so the action API is the door that exists. The camera is opened
 * lazily by the backend and released when this panel is destroyed.
 *
 * mount(root, api) -> { update(state), destroy() }
 */
const PANEL = 'cameras';

const FRAME_MS = 320;          // ~3 fps. The source is 5 fps; faster only burns CPU.

const CSS = `
.fcam{display:grid;gap:var(--s3,14px);grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}
.fcam .wide{grid-column:1/-1}
.fcam .feed{position:relative;background:#000;border:1px solid var(--line);
  border-radius:10px;overflow:hidden;aspect-ratio:4/3;display:grid;place-items:center}
.fcam .feed img{width:100%;height:100%;object-fit:contain;display:block}
.fcam .badge{position:absolute;left:10px;top:10px;font:500 11px/1.4 ui-monospace,monospace;
  background:rgba(0,0,0,.55);padding:4px 8px;border-radius:6px;color:var(--muted)}
.fcam .badge.live{color:var(--ok)}
.fcam .badge.bad{color:var(--bad)}
.fcam .empty{padding:22px;text-align:center;color:var(--muted);max-width:44ch}
.fcam .empty .hd{color:var(--warn);font:600 13px/1.4 system-ui;letter-spacing:.06em;
  text-transform:uppercase;margin-bottom:8px}
.fcam .empty .bd{font-size:12px;line-height:1.65}
.fcam .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;align-items:baseline;
  margin-top:10px}
.fcam .kv dt{color:var(--muted);font-size:12px}
.fcam .kv dd{margin:0;color:var(--text);font-variant-numeric:tabular-nums;font-size:12px}
.fcam .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:10px}
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
  const el = h('div', { class: 'fcam' });
  root.replaceChildren(h('style', { text: CSS }), el);

  // ---- thermal (rover) -------------------------------------------------
  const thermBadge = h('div', { class: 'badge bad', text: 'no driver' });
  const thermBody = h('div', { class: 'empty' });
  const thermFeed = h('div', { class: 'feed' }, [thermBody, thermBadge]);
  const thermKv = h('dl', { class: 'kv' });
  const thermCard = h('div', { class: 'card' }, [
    h('h2', { text: 'Thermal — rover' }), thermFeed, thermKv,
  ]);

  // ---- regular (ground station) ---------------------------------------
  const img = h('img', { alt: 'live camera frame' });
  const camBadge = h('div', { class: 'badge', text: 'starting…' });
  const camEmpty = h('div', { class: 'empty' });
  const camFeed = h('div', { class: 'feed' }, [img, camBadge]);
  const camKv = h('dl', { class: 'kv' });
  const btnPause = h('button', { class: 'btn', type: 'button', text: 'Pause' });
  const camCard = h('div', { class: 'card' }, [
    h('h2', { text: 'Regular — ground station' }), camFeed, camKv,
    h('div', { class: 'row' }, [btnPause]),
  ]);

  const note = h('div', { class: 'card wide note', 'data-detail': 'full' });
  el.append(thermCard, camCard, note);

  let dead = false;
  let paused = false;
  let timer = null;
  let busy = false;
  let frames = 0;
  let lastOk = 0;
  let lastErr = null;
  let started = false;

  function kv(dl, pairs) {
    dl.replaceChildren();
    for (const [k, v] of pairs) dl.append(h('dt', { text: k }), h('dd', { text: v }));
  }

  btnPause.addEventListener('click', async () => {
    paused = !paused;
    btnPause.textContent = paused ? 'Resume' : 'Pause';
    if (paused) {
      camBadge.className = 'badge';
      camBadge.textContent = 'paused';
      // Hand the device back straight away rather than waiting for the
      // backend watchdog - something else may want the webcam.
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
        camEmpty.remove();
        frames++;
        lastOk = Date.now();
        lastErr = null;
        started = true;
        camBadge.className = 'badge live';
        camBadge.textContent = `live  ${(r.bytes / 1024).toFixed(0)} KB`;
      }
    } catch (e) {
      if (dead) return;
      lastErr = (e && e.message) || String(e);
      camBadge.className = 'badge bad';
      camBadge.textContent = 'no frame';
      if (!started) {
        img.style.display = 'none';
        if (!camEmpty.isConnected) camFeed.append(camEmpty);
        camEmpty.replaceChildren(
          h('div', { class: 'hd', text: 'no frame' }),
          h('div', { class: 'bd', text: lastErr }),
        );
      }
    } finally {
      busy = false;
    }
  }

  function update(full) {
    const st = ((full || {}).panels || {})[PANEL];
    if (!st) return;

    // -- thermal: always an explicit, explained non-feed ------------------
    const t = st.thermal || {};
    thermBody.replaceChildren(
      h('div', { class: 'hd', text: t.present ? 'present — no driver' : 'not detected' }),
      h('div', {
        class: 'bd',
        text: t.present
          ? (t.why || 'no kernel driver binds this device')
          : (t.reachable === false
            ? ('rover not reachable: ' + (t.error || 'unknown'))
            : 'the thermal camera is not on the rover’s USB bus'),
      }),
    );
    thermBadge.className = 'badge ' + (t.present ? 'bad' : '');
    thermBadge.textContent = t.present ? 'no driver' : 'absent';
    kv(thermKv, [
      ['device', 'Thermal Master P1'],
      ['where', t.host || DASH],
      ['on the bus', t.present ? 'yes' : 'no'],
      ['usable', 'no'],
    ]);

    // -- regular ----------------------------------------------------------
    const c = st.regular || {};
    const cap = c.capture || {};
    kv(camKv, [
      ['device', c.device || DASH],
      ['where', c.where || DASH],
      ['size', c.size ? c.size.join(' × ') : DASH],
      ['capture', cap.running ? 'running' : (paused ? 'paused' : 'idle')],
      ['source fps', cap.fps != null ? String(cap.fps) : DASH],
      ['frames shown', String(frames)],
    ]);

    if (!note.dataset.filled && c.note) {
      note.dataset.filled = '1';
      note.replaceChildren(h('div', {
        class: 'small',
        text: c.note + ' The camera is opened only while this tab is open and '
            + 'released as soon as you leave it, because /dev/video0 is '
            + 'exclusive - holding it would stop anything else using the webcam.',
      }));
    }
  }

  timer = setInterval(grab, FRAME_MS);
  grab();

  if (api && api.state) update(api.state);

  return {
    update,
    destroy() {
      dead = true;
      clearInterval(timer);
      // Release /dev/video0 now. The backend watchdog would get there in ~6 s,
      // but leaving the webcam held after the tab is gone is a surprise.
      try { if (api.call) api.call(PANEL, 'stop', {}); } catch (e) { /* best effort */ }
    },
  };
}
