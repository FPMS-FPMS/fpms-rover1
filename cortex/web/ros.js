/* ros.js - the one ROS connection the Cortex uses.
 *
 * WHY roslibjs AND NOT A RAW WEBSOCKET
 * A browser cannot speak DDS - there is no socket type for it. rosbridge is
 * the ROS-native bridge for exactly this, and roslibjs is the official client.
 *
 * ONE CONNECTION, SHARED, AND IT RECONNECTS
 * Three panels each opening their own socket meant three TCP connections and
 * three copies of every message on a 2-core ground station. This hands every
 * panel the SAME ROSLIB.Ros, reference counted.
 *
 * ROSLIB.Ros DOES NOT RESUBSCRIBE BY ITSELF. When the socket drops - and it
 * does; rosbridge logs "WebSocketClosedError: Tried to write to a closed
 * websocket" - roslibjs neither reconnects nor replays subscriptions. A panel
 * that subscribed once then goes silent forever, while its last message stays
 * on screen looking live. So subscriptions are registered HERE and replayed on
 * every reconnect, and the socket is retried with backoff.
 *
 * roslib.min.js is vendored in vendor/ deliberately: at a venue the ground
 * station is an isolated hotspot with no internet.
 */

let loading = null;

function loadRoslib() {
  if (window.ROSLIB) return Promise.resolve(window.ROSLIB);
  if (loading) return loading;
  loading = new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = './vendor/roslib.min.js';
    s.async = true;
    // Clear the cache on failure. Caching a REJECTED promise means one bad
    // first paint poisons every later connect() with no retry.
    const fail = (msg) => { loading = null; reject(new Error(msg)); };
    s.onload = () => window.ROSLIB ? resolve(window.ROSLIB) : fail('ROSLIB undefined after load');
    s.onerror = () => fail('could not load vendor/roslib.min.js');
    document.head.appendChild(s);
  });
  return loading;
}

const conns = new Map();   // url -> connection record

function fanStatus(c, state, detail) {
  c.connected = state;
  for (const fn of c.listeners) {
    try { fn(state, detail); } catch (e) { console.error('ros status cb', e); }
  }
}

function resubscribe(c) {
  for (const sub of c.subs) {
    try { if (sub.topic) sub.topic.unsubscribe(); } catch (e) { /* stale handle */ }
    try {
      sub.topic = new c.ROSLIB.Topic({ ros: c.ros, name: sub.name, messageType: sub.type });
      sub.topic.subscribe(sub.cb);
    } catch (e) {
      console.error('ros resubscribe', sub.name, e);
    }
  }
}

function scheduleReconnect(c, url) {
  if (c.retry || c.closed) return;
  c.backoff = Math.min((c.backoff || 1000) * 2, 15000);
  c.retry = setTimeout(() => {
    c.retry = null;
    if (c.closed) return;
    try { c.ros.connect(url); } catch (e) { scheduleReconnect(c, url); }
  }, c.backoff);
}

export async function connect(url, onStatus) {
  const ROSLIB = await loadRoslib();
  let c = conns.get(url);
  if (!c) {
    const ros = new ROSLIB.Ros({ url });
    c = { ROSLIB, ros, refs: 0, connected: false, listeners: new Set(),
          subs: new Set(), retry: null, backoff: 1000, closed: false, closeTimer: null };
    ros.on('connection', () => {
      c.backoff = 1000;
      // Replay every subscription: the bridge has no memory of them.
      resubscribe(c);
      fanStatus(c, true, 'connected');
    });
    ros.on('error', (e) => fanStatus(c, false, 'error: ' + ((e && e.message) || 'unknown')));
    ros.on('close', () => {
      fanStatus(c, false, 'closed');
      scheduleReconnect(c, url);
    });
    conns.set(url, c);
  }
  c.refs++;
  if (c.closeTimer) { clearTimeout(c.closeTimer); c.closeTimer = null; }
  c.closed = false;
  if (onStatus) {
    c.listeners.add(onStatus);
    onStatus(c.connected, c.connected ? 'connected' : 'connecting');
  }
  return { ROSLIB, ros: c.ros, _conn: c };
}

export function release(url, onStatus) {
  const c = conns.get(url);
  if (!c) return;
  if (onStatus) c.listeners.delete(onStatus);
  c.refs = Math.max(0, c.refs - 1);
  if (c.refs === 0) {
    // Linger: only one panel is mounted at a time, so closing instantly means
    // every tab switch drops and rebuilds the connection.
    c.closeTimer = setTimeout(() => {
      if (c.refs !== 0) return;
      c.closed = true;
      if (c.retry) { clearTimeout(c.retry); c.retry = null; }
      for (const sub of c.subs) {
        try { if (sub.topic) sub.topic.unsubscribe(); } catch (e) { /* gone */ }
      }
      c.subs.clear();
      try { c.ros.close(); } catch (e) { /* already gone */ }
      conns.delete(url);
    }, 15000);
  }
}

/**
 * Subscribe. The subscription is REGISTERED so it survives a reconnect.
 * Returns a function that unsubscribes and deregisters it.
 */
export function subscribe(ROSLIB, ros, name, messageType, cb) {
  let c = null;
  for (const rec of conns.values()) if (rec.ros === ros) { c = rec; break; }
  const sub = { name, type: messageType, cb, topic: null };
  if (c) {
    c.subs.add(sub);
    if (c.connected) {
      sub.topic = new ROSLIB.Topic({ ros, name, messageType });
      sub.topic.subscribe(cb);
    }
    // If not connected yet, resubscribe() on 'connection' will create it.
  } else {
    sub.topic = new ROSLIB.Topic({ ros, name, messageType });
    sub.topic.subscribe(cb);
  }
  return () => {
    try { if (sub.topic) sub.topic.unsubscribe(); } catch (e) { /* gone */ }
    sub.topic = null;
    if (c) c.subs.delete(sub);
  };
}
