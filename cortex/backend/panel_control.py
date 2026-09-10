#!/usr/bin/env python3
"""panel_control - CONTROL panel backend: the live picture of Rover 1.

Rover 1 is the Orange Pi 5 Max (fpms1). It publishes its own sensors over
plain HTTP from fpms-rover-bridge on port 8781; this panel is a thin, cached
view of that.

WHY A BACKGROUND THREAD AND NOT A BLOCKING CACHE
------------------------------------------------
panel_rover.py polls inline and lets the registry throttle it. That is correct
for a panel whose data is cheap to be stale about, but the registry derives a
panel's refresh interval from how long state() takes, so an inline network
poll makes the panel refresh LESS often exactly when the link is slow - the
moment you most want to see it. Here state() only ever reads memory, so its
measured cost is ~0 and the cadence stays high; a separate daemon thread owns
the network and its deadlines.

"Rover unreachable" is a normal operating state, not an error. There is no
remote power-on.

Nothing in this file writes to the rover. The MCU protocol on the CH340 is not
known yet, and writing to an unknown MCU can move a motor.
"""

import json
import threading
import time
import urllib.request

ROVER_HOST = "fpms1.local"
BRIDGE = "http://%s:8781" % ROVER_HOST

# The HTTP bridge runs in health-only mode and never opens the scanner, so the
# live sensor values come from ROS. Host facts (temperatures, uptime, the USB
# inventory) still come over HTTP because nothing publishes them to ROS yet.
ROSBRIDGE_WS = "ws://%s:9090" % ROVER_HOST
TOPICS = {
    "battery": ("/rover/battery", "sensor_msgs/msg/BatteryState"),
    "scan": ("/scan", "sensor_msgs/msg/LaserScan"),
    "imu": ("/imu/data_raw", "sensor_msgs/msg/Imu"),
    "attitude": ("/rover/attitude", "geometry_msgs/msg/Vector3Stamped"),
}

POLL_INTERVAL = 2.0
HTTP_TIMEOUT = 3.0
STALE_AFTER = 8.0          # older than this and we stop calling the link up



# ---------------------------------------------------------------------------
# Resolve the rover to an IP, once, and hand the frontend an IP-based URL.
#
# WHY NOT JUST GIVE THE BROWSER "fpms1.local":
# Chrome uses its own async DNS resolver, which on Linux does NOT go through
# nsswitch - so an mDNS name that resolves perfectly from the shell can fail
# inside the page, and every ROS-fed value (scan, battery, attitude, thermal)
# goes dark at once with no error the operator can see. Resolving here, in
# Python, uses the system resolver where mDNS does work.
# It also removes an mDNS lookup from every poll; that was costing ~550 ms.
# ---------------------------------------------------------------------------
_RESOLVED = {"ip": None, "at": 0.0}
_RESOLVE_TTL = 300.0
_RESOLVING = {"busy": False}


def _resolve_bg():
    """Refresh the cached IP off the panel thread."""
    try:
        import socket
        ip = socket.getaddrinfo(ROVER_HOST, None, socket.AF_INET)[0][4][0]
        _RESOLVED["ip"], _RESOLVED["at"] = ip, time.time()
    except Exception:                                    # noqa: BLE001
        pass
    finally:
        _RESOLVING["busy"] = False


def rover_ip(default=ROVER_HOST):
    """Current IP for the rover. NEVER BLOCKS.

    state() cost drives how often the registry runs a panel, so a blocking
    getaddrinfo here is self-defeating: the mDNS lookup measured ~47 ms, which
    pushed the refresh interval out to 76 SECONDS. So a stale cache is
    refreshed on a background thread and the old value is returned meanwhile.
    """
    now = time.time()
    if _RESOLVED["ip"] and (now - _RESOLVED["at"]) < _RESOLVE_TTL:
        return _RESOLVED["ip"]
    if not _RESOLVING["busy"]:
        _RESOLVING["busy"] = True
        threading.Thread(target=_resolve_bg, daemon=True).start()
    return _RESOLVED["ip"] or default

def _json_get(url, secs=HTTP_TIMEOUT):
    try:
        with urllib.request.urlopen(url, timeout=secs) as r:
            return json.loads(r.read().decode("utf-8", "replace")), None
    except Exception as exc:                      # noqa: BLE001 - down is normal
        return None, "%s: %s" % (type(exc).__name__, exc)


class _Poller:
    """Owns the network. state() never waits on it."""

    def __init__(self):
        self.lock = threading.Lock()
        self.health = None
        self.error = "not polled yet"
        self.at = 0.0
        self.rtt_ms = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            t0 = time.time()
            data, err = _json_get(BRIDGE + "/health")
            dt = (time.time() - t0) * 1000.0
            with self.lock:
                if data is not None:
                    self.health, self.error, self.at = data, None, time.time()
                    self.rtt_ms = round(dt, 1)
                else:
                    self.error = err
                    self.rtt_ms = None
            time.sleep(POLL_INTERVAL)

    def get(self):
        with self.lock:
            age = time.time() - self.at if self.at else None
            fresh = age is not None and age < STALE_AFTER
            return {
                "health": self.health if fresh else None,
                "error": self.error,
                "age_s": round(age, 1) if age is not None else None,
                "rtt_ms": self.rtt_ms,
                "fresh": fresh,
            }


POLLER = _Poller()


def state() -> dict:
    snap = POLLER.get()
    h = snap["health"] or {}
    dev = h.get("devices") or {}
    lid = h.get("lidar") or {}

    return {
        "host": ROVER_HOST,
        "bridge": BRIDGE,
        "ip": rover_ip(),
        "rosbridge": "ws://%s:9090" % rover_ip(),
        "topics": {k: {"topic": t, "type": ty} for k, (t, ty) in TOPICS.items()},
        "up": bool(snap["fresh"]),
        "error": snap["error"],
        "age_s": snap["age_s"],
        "rtt_ms": snap["rtt_ms"],
        # Everything below is None when the rover is down. The UI must render
        # "unknown", never a remembered value presented as current.
        "uptime_s": h.get("uptime_s"),
        "load": h.get("load"),
        "mem_mb": h.get("mem_mb"),
        "temps_c": h.get("temps_c"),
        "temp_max_c": h.get("temp_max_c"),
        "wifi": h.get("wifi"),
        "devices": dev,
        "lidar": {
            "ok": lid.get("ok"),
            "rpm": lid.get("rpm"),
            "hz": lid.get("hz"),
            "count": lid.get("count"),
            "frames": lid.get("frames"),
            "crc_reject": lid.get("crc_reject"),
            "error": lid.get("error"),
        },
        "note": None if snap["fresh"] else
                "rover unreachable - normal when it is powered off. There is "
                "no remote power-on; use the physical switch.",
    }


def act_refresh(params):
    """Force a poll now rather than waiting for the next tick."""
    data, err = _json_get(BRIDGE + "/health")
    if data is None:
        return {"ok": False, "error": "rover did not answer", "detail": err}
    with POLLER.lock:
        POLLER.health, POLLER.error, POLLER.at = data, None, time.time()
    return {"ok": True, "host": ROVER_HOST}


def actions() -> dict:
    return {"refresh": act_refresh}
