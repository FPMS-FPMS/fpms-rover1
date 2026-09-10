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
  const thermBadge = h('div', { class: 'badge bad', text: 'connecting' });
  const thermBody = h('div', { class: 'empty' });
  const thermImg = h('img', { alt: 'live thermal frame' });
  thermImg.style.display = 'none';
  // Nearest-neighbour: 160x120 blown up. Smoothing invents gradients that the
  // sensor did not measure, which on a thermal image reads as real structure.
  thermImg.style.imageRendering = 'pixelated';
  const thermFeed = h('div', { class: 'feed' }, [thermImg, thermBody, thermBadge]);
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
  // Thermal arrives over rosbridge, not through panel state: a 12 KB PNG in
  // the SSE snapshot would be broadcast to every client on every tick.
  let tws = null, twired = false, tretry = null;
  let tAt = 0, tFrames = 0, tStats = null, tTopics = null;
  let tObjUrl = null;
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

  const tFresh = () => (Date.now() - tAt) < 3000;

  function b64ToBlob(b64, mime) {
    const bin = atob(b64);
    const u8 = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    return new Blob([u8], { type: mime });
  }

  function connectThermal(url, topics) {
    if (dead) return;
    try { tws = new WebSocket(url); } catch (e) { retryThermal(url, topics); return; }
    tws.onopen = () => {
      if (dead) return;
      tws.send(JSON.stringify({ op: 'subscribe', topic: topics.image_topic,
                                type: topics.image_type }));
      tws.send(JSON.stringify({ op: 'subscribe', topic: topics.stats_topic,
                                type: topics.stats_type }));
    };
    tws.onmessage = (ev) => {
      if (dead) return;
      let m;
      try { m = JSON.parse(ev.data); } catch (e) { return; }
      if (!m || !m.msg) return;
      if (m.topic === topics.image_topic && m.msg.data) {
        // Object URLs, not data: URLs - a 12 KB base64 string per frame at
        // 8 fps churns far more memory as a string than as a Blob.
        const blob = b64ToBlob(m.msg.data, 'image/' + (m.msg.format || 'png'));
        const next = URL.createObjectURL(blob);
        const prev = tObjUrl;
        tObjUrl = next;
        thermImg.src = next;
        thermImg.style.display = '';
        thermBody.style.display = 'none';
        if (prev) setTimeout(() => URL.revokeObjectURL(prev), 250);
        tAt = Date.now(); tFrames++;
      } else if (m.topic === topics.stats_topic && m.msg.data) {
        tStats = m.msg.data;
        tAt = Date.now();
      }
      paintThermal();
    };
    tws.onclose = () => { if (!dead) retryThermal(url, topics); };
    tws.onerror = () => { try { tws.close(); } catch (e) { /* closing */ } };
  }

  function retryThermal(url, topics) {
    clearTimeout(tretry);
    tretry = setTimeout(() => { if (!dead) connectThermal(url, topics); }, 3000);
  }

  function paintThermal() {
    const live = tFresh();
    thermBadge.className = 'badge ' + (live ? 'live' : 'bad');
    if (live && tStats) {
      thermBadge.textContent = `live  ${tStats[0].toFixed(1)}–${tStats[1].toFixed(1)} °C`;
    } else if (live) {
      thermBadge.textContent = 'live';
    } else {
      thermBadge.textContent = 'no frames';
    }
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

    // -- thermal ----------------------------------------------------------
    const t = st.thermal || {};
    if (!twired && t.usable && t.rosbridge && t.image_topic) {
      twired = true;
      tTopics = t;
      connectThermal(t.rosbridge, t);
    }
    if (!tFresh()) {
      thermImg.style.display = 'none';
      thermBody.style.display = '';
      thermBody.replaceChildren(
        h('div', { class: 'hd', text: t.usable ? 'waiting for frames' : 'unavailable' }),
        h('div', {
          class: 'bd',
          text: t.usable
            ? ('subscribing to ' + (t.image_topic || '') + ' on ' + (t.rosbridge || ''))
            : (t.note || 'no driver binds this device'),
        }),
      );
    }
    paintThermal();
    kv(thermKv, [
      ['device', t.device || 'Thermal Master P1'],
      ['where', t.host || DASH],
      ['size', t.size ? t.size.join(' × ') : DASH],
      ['transport', t.usable ? 'ROS 2 → rosbridge' : DASH],
      ['topic', t.image_topic || DASH],
      ['frames shown', String(tFrames)],
      ['range now', tStats ? `${tStats[0].toFixed(1)} – ${tStats[1].toFixed(1)} °C` : DASH],
      ['centre', tStats ? `${tStats[3].toFixed(1)} °C` : DASH],
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
      clearTimeout(tretry);
      try { if (tws) tws.close(); } catch (e) { /* already gone */ }
      tws = null;
      // Object URLs are not garbage collected on their own.
      if (tObjUrl) { try { URL.revokeObjectURL(tObjUrl); } catch (e) { /* gone */ } tObjUrl = null; }
      // Release /dev/video0 now. The backend watchdog would get there in ~6 s,
      // but leaving the webcam held after the tab is gone is a surprise.
      try { if (api.call) api.call(PANEL, 'stop', {}); } catch (e) { /* best effort */ }
    },
  };
}
