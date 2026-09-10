/* panel_control - CONTROL panel for Rover 1.
 *
 * The link, the health and the device inventory of the Orange Pi 5 Max.
 * Everything shown here is measured on the rover and arrives via
 * panel_control.py; nothing is inferred and nothing is remembered across a
 * dropout. When the rover is down every field goes to an explicit dash rather
 * than holding its last value, because a stale reading that looks live is
 * worse than no reading.
 *
 * There are no drive controls here on purpose. The MCU on the CH340 has not
 * been identified and its protocol is unknown; a button that writes to an
 * unknown serial port can move a motor.
 *
 * mount(root, api) -> { update(state), destroy() }
 */
const PANEL = 'control';

const CSS = `
.fctl{display:grid;gap:var(--s3,14px);grid-template-columns:repeat(auto-fill,minmax(260px,1fr))}
.fctl .wide{grid-column:1/-1}
.fctl .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;align-items:baseline}
.fctl .kv dt{color:var(--muted);font-size:12px}
.fctl .kv dd{margin:0;font-variant-numeric:tabular-nums;color:var(--text)}
.fctl .big{font:600 26px/1.1 system-ui;font-variant-numeric:tabular-nums}
.fctl .dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:7px;
  vertical-align:middle;background:var(--muted)}
.fctl .dot.ok{background:var(--ok)} .fctl .dot.bad{background:var(--bad)}
.fctl .dot.warn{background:var(--warn)}
.fctl .devs{display:grid;gap:6px}
.fctl .dev{display:grid;grid-template-columns:1fr auto;gap:8px;align-items:center;
  padding:7px 9px;border:1px solid var(--line);border-radius:8px;background:var(--panel2)}
.fctl .dev .nm{font-size:13px;color:var(--text)}
.fctl .dev .pathx{font-size:11px;color:var(--muted);word-break:break-all}
.fctl .tag{font-size:11px;padding:2px 7px;border-radius:999px;border:1px solid var(--line);
  color:var(--muted);white-space:nowrap}
.fctl .tag.ok{color:var(--ok);border-color:var(--ok)}
.fctl .tag.bad{color:var(--bad);border-color:var(--bad)}
.fctl .tag.warn{color:var(--warn);border-color:var(--warn)}
.fctl .temps{display:flex;flex-wrap:wrap;gap:6px}
.fctl .tchip{font-size:11px;padding:3px 8px;border-radius:6px;background:var(--panel2);
  border:1px solid var(--line);color:var(--muted);font-variant-numeric:tabular-nums}
.fctl .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
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

function uptime(s) {
  if (s === null || s === undefined) return DASH;
  const d = Math.floor(s / 86400), hh = Math.floor(s % 86400 / 3600);
  const mm = Math.floor(s % 3600 / 60), ss = Math.floor(s % 60);
  if (d) return `${d}d ${hh}h`;
  if (hh) return `${hh}h ${mm}m`;
  if (mm) return `${mm}m ${ss}s`;
  return `${ss}s`;
}

export function mount(root, api = {}) {
  const el = h('div', { class: 'fctl' });
  root.replaceChildren(h('style', { text: CSS }), el);

  // ---- link ------------------------------------------------------------
  const linkDot = h('span', { class: 'dot' });
  const linkTxt = h('span', { text: 'checking…' });
  const linkBig = h('div', { class: 'big' });
  const linkKv = h('dl', { class: 'kv' });
  const btnRefresh = h('button', { class: 'btn', type: 'button', text: 'Refresh now' });
  const linkCard = h('div', { class: 'card' }, [
    h('h2', { text: 'Rover 1 link' }),
    h('div', { class: 'row' }, [linkDot, linkTxt]),
    linkBig, linkKv,
    h('div', { class: 'row', style: 'margin-top:10px' }, [btnRefresh]),
  ]);

  // ---- health ----------------------------------------------------------
  const healthKv = h('dl', { class: 'kv' });
  const temps = h('div', { class: 'temps' });
  const healthCard = h('div', { class: 'card' }, [
    h('h2', { text: 'Health' }), healthKv,
    h('div', { style: 'margin-top:10px' }, [temps]),
  ]);

  // ---- lidar summary ---------------------------------------------------
  const lidKv = h('dl', { class: 'kv' });
  const lidCard = h('div', { class: 'card' }, [h('h2', { text: 'Scanner' }), lidKv]);

  // ---- devices ---------------------------------------------------------
  const devs = h('div', { class: 'devs' });
  const devCard = h('div', { class: 'card wide' }, [
    h('h2', { text: 'Devices on the rover' }), devs,
  ]);

  const noteCard = h('div', { class: 'card wide note', 'data-detail': 'full' }, [
    h('div', {
      class: 'small',
      text: 'No drive controls are shown here. The MCU on the CH340 bridge has '
          + 'not been identified and its serial protocol is unknown - a control '
          + 'that writes to it could move a motor. Controls appear here once the '
          + 'protocol is known.',
    }),
  ]);

  el.append(linkCard, healthCard, lidCard, devCard, noteCard);

  function kv(dl, pairs) {
    dl.replaceChildren();
    for (const [k, v] of pairs) {
      dl.append(h('dt', { text: k }), h('dd', { text: v }));
    }
  }

  function devRow(name, present, usable, path, why) {
    let cls = 'tag bad', label = 'absent';
    if (present && usable === false) { cls = 'tag warn'; label = 'no driver'; }
    else if (present) { cls = 'tag ok'; label = 'present'; }
    const left = h('div', {}, [h('div', { class: 'nm', text: name })]);
    if (path) left.append(h('div', { class: 'pathx mono', text: path }));
    if (why) left.append(h('div', { class: 'pathx', 'data-detail': 'full', text: why }));
    return h('div', { class: 'dev' }, [left, h('span', { class: cls, text: label })]);
  }

  btnRefresh.addEventListener('click', async () => {
    if (!api.call) return;
    btnRefresh.disabled = true;
    btnRefresh.textContent = 'Refreshing…';
    try { await api.call(PANEL, 'refresh', {}); }
    catch (e) { linkTxt.textContent = 'refresh failed: ' + (e.message || e); }
    finally { btnRefresh.disabled = false; btnRefresh.textContent = 'Refresh now'; }
  });

  function update(full) {
    const st = ((full || {}).panels || {})[PANEL];
    if (!st) return;

    const up = !!st.up;
    linkDot.className = 'dot ' + (up ? 'ok' : 'bad');
    linkTxt.textContent = up ? 'reachable' : 'not answering';
    linkBig.textContent = up ? fmt(st.rtt_ms, ' ms', 1) : DASH;
    kv(linkKv, [
      ['host', st.host || DASH],
      ['bridge', st.bridge || DASH],
      ['data age', up ? fmt(st.age_s, ' s', 1) : DASH],
      ['last error', st.error ? String(st.error).slice(0, 60) : 'none'],
    ]);

    const w = st.wifi || {};
    kv(healthKv, [
      ['uptime', up ? uptime(st.uptime_s) : DASH],
      ['load', up && st.load ? st.load.map((x) => x.toFixed(2)).join('  ') : DASH],
      ['memory free', up && st.mem_mb ? fmt(st.mem_mb.MemAvailable, ' MB') + ' of '
        + fmt(st.mem_mb.MemTotal, ' MB') : DASH],
      ['wifi signal', up ? fmt(w.signal_dbm, ' dBm') : DASH],
      ['rover ip', up ? (w.ip || DASH) : DASH],
      ['hottest', up ? fmt(st.temp_max_c, ' °C', 1) : DASH],
    ]);

    temps.replaceChildren();
    if (up && st.temps_c) {
      for (const [k, v] of Object.entries(st.temps_c)) {
        temps.append(h('span', {
          class: 'tchip', text: k.replace(/-thermal$/, '') + ' ' + v.toFixed(1) + '°',
        }));
      }
    }

    const L = st.lidar || {};
    kv(lidKv, [
      ['status', up ? (L.ok ? 'spinning' : (L.error || 'no data')) : DASH],
      ['rate', up ? fmt(L.hz, ' Hz', 2) : DASH],
      ['rpm', up ? fmt(L.rpm, '', 1) : DASH],
      ['points now', up ? fmt(L.count) : DASH],
      ['frames read', up ? fmt(L.frames) : DASH],
      ['CRC rejects', up ? fmt(L.crc_reject) : DASH],
    ]);

    devs.replaceChildren();
    const d = st.devices || {};
    if (!up) {
      devs.append(h('div', { class: 'muted small', text: 'rover not answering' }));
    } else {
      if (d.lidar) devs.append(devRow('LiDAR — D500', d.lidar.present, true, d.lidar.path));
      if (d.mcu) devs.append(devRow('MCU — CH340 bridge', d.mcu.present, true, d.mcu.path));
      if (d.thermal_cam) {
        devs.append(devRow('Thermal camera — Thermal Master P1',
          d.thermal_cam.present, false, 'usb ' + (d.thermal_cam.usb_id || ''),
          d.thermal_cam.note));
      }
      if (d.camera) devs.append(devRow('Regular camera', d.camera.present, false, null, d.camera.note));
    }

    noteCard.style.display = st.note ? '' : '';
  }

  if (api && api.state) update(api.state);
  return { update, destroy() {} };
}
