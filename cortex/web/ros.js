/* ros.js - the one ROS connection the Cortex uses.
 *
 * WHY roslibjs AND NOT A RAW WEBSOCKET
 * A browser cannot speak DDS - there is no socket type for it and no way to
 * join a DDS domain from a page. rosbridge is the ROS-native bridge for
 * exactly this, and roslibjs is the official ROS client that speaks it. Using
 * the library rather than hand-rolled JSON gets the real protocol: proper
 * subscribe/unsubscribe accounting, service calls, param access, and
 * reconnection - all of which hand-written code has to reinvent badly.
 *
 * ONE CONNECTION, SHARED
 * Three panels each opening their own socket meant three TCP connections and
 * three copies of every message on a 2-core ground station. rosbridge also
 * counts clients, so it looked like three operators. This hands every panel
 * the SAME ROSLIB.Ros, reference-counted, and only tears it down when the
 * last panel lets go.
 *
 * roslib.min.js is vendored in vendor/ deliberately: at a venue the ground
 * station is an isolated hotspot with no internet, and a CDN reference would
 * simply fail there.
 */

let loading = null;

/** Load roslib.min.js once. It is a classic script, not an ES module. */
function loadRoslib() {
  if (window.ROSLIB) return Promise.resolve(window.ROSLIB);
  if (loading) return loading;
  loading = new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = './vendor/roslib.min.js';
    s.async = true;
    s.onload = () => window.ROSLIB ? resolve(window.ROSLIB)
                                   : reject(new Error('roslib loaded but ROSLIB is undefined'));
    s.onerror = () => reject(new Error('could not load vendor/roslib.min.js'));
    document.head.appendChild(s);
  });
  return loading;
}

const conns = new Map();   // url -> { ros, refs, listeners }

/**
 * Get the shared connection for `url`.
 * onStatus(connected, detail) fires on every state change, including the
 * current state immediately, so a panel mounted while already connected does
 * not sit there looking dead.
 */
export async function connect(url, onStatus) {
  const ROSLIB = await loadRoslib();
  let c = conns.get(url);
  if (!c) {
    const ros = new ROSLIB.Ros({ url });
    c = { ros, refs: 0, connected: false, listeners: new Set() };
    const fan = (state, detail) => {
      c.connected = state;
      for (const fn of c.listeners) {
        try { fn(state, detail); } catch (e) { console.error('ros status cb', e); }
      }
    };
    ros.on('connection', () => fan(true, 'connected'));
    // roslibjs reconnects on its own; surface the reason rather than swallowing it.
    ros.on('error', (e) => fan(false, 'error: ' + (e && e.message ? e.message : 'unknown')));
    ros.on('close', () => fan(false, 'closed'));
    conns.set(url, c);
  }
  c.refs++;
  if (onStatus) {
    c.listeners.add(onStatus);
    onStatus(c.connected, c.connected ? 'connected' : 'connecting');
  }
  return { ROSLIB, ros: c.ros };
}

/** Release a panel's hold. The socket closes only when the last one lets go. */
export function release(url, onStatus) {
  const c = conns.get(url);
  if (!c) return;
  if (onStatus) c.listeners.delete(onStatus);
  c.refs = Math.max(0, c.refs - 1);
  if (c.refs === 0) {
    try { c.ros.close(); } catch (e) { /* already gone */ }
    conns.delete(url);
  }
}

/** Subscribe, returning a function that unsubscribes. */
export function subscribe(ROSLIB, ros, name, messageType, cb) {
  const topic = new ROSLIB.Topic({ ros, name, messageType });
  topic.subscribe(cb);
  return () => { try { topic.unsubscribe(); } catch (e) { /* already gone */ } };
}
