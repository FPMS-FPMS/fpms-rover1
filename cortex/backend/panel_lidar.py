#!/usr/bin/env python3
"""panel_lidar - LIDAR panel backend for Rover 1.

This backend deliberately does almost nothing, and that is the design.

WHY THE SCAN DOES NOT COME THROUGH HERE
----------------------------------------
A full 360-point scan is ~6 KB of JSON at 10 Hz. Panel state is merged into
the SSE snapshot that goes to every attached client on every tick, and this
codebase has already been burned by exactly that: the panels dict once made up
96% of a 211 KB frame and cost 11x more CPU per viewer (see the SAMPLE_HZ note
in cortex_core.py). Routing the scan through here would rebuild that problem.

So the browser fetches the scan STRAIGHT from the rover's bridge, the same
reasoning panel_rover.js uses for rosbridge: the rover already speaks a
protocol the page can consume, and a proxy hop would add latency and a copy
for no gain. This panel's whole job is to hand the frontend the URL, so the
address lives in exactly one place.

The scanner is an LDROBOT D500 on the CP2102 bridge. Frame format was
confirmed against live hardware on 2026-09-10 by CRC: 419 frames, 0 rejects.
"""

import threading
import time

ROVER_HOST = "fpms1.local"
BRIDGE = "http://%s:8781" % ROVER_HOST
# ROS 2 Humble is the primary path; the HTTP bridge above is the fallback.
# Only ONE of them may own the scanner's serial port at a time - the two
# systemd units declare Conflicts= on each other for exactly that reason - so
# the frontend tries rosbridge first and falls back if it goes quiet.
ROSBRIDGE_WS = "ws://%s:9090" % ROVER_HOST
SCAN_TOPIC = "/scan"
SCAN_TYPE = "sensor_msgs/msg/LaserScan"

# The scanner's own geometry and limits, so the plot is drawn to scale rather
# than normalised to whatever the largest return happened to be.
LIDAR = {
    "model": "LDROBOT D500 (LD19 protocol)",
    "port": "CP2102 USB-UART bridge",
    "baud": 230400,
    "range_min_mm": 20,
    "range_max_mm": 12000,
    "nominal_hz": 10,
    "points_per_rev": 360,
}

# ---------------------------------------------------------------------------
# ARENA AND MARKERS
#
# TAKEN VERBATIM from panel_field.py, which took them verbatim from the mission
# stack (FPMSM2PERFECT/fpms_rover2/geometry.py). That file carries the warning
# and it applies here: "DO NOT re-measure these into something else. Every
# failure of the 2026-09-02 session traced to a zone constant that had drifted."
#
# So M1 and M2 below are NOT retyped from a tape measure - they are the same
# numbers the working mission stack drives to.
# ---------------------------------------------------------------------------
ARENA = {
    "w": 1000.0,                 # ARENA_W_MM
    "h": 1200.0,                 # ARENA_H_MM
    "home": {"x": 842.5, "y": 157.5, "heading_deg": 90.0},
    "zone_radius": 131.0,        # the vases are 263 mm across
    "edge_keep": 150.0,          # the rover CENTRE may not go nearer an edge
    # Where an object may BE is deliberately wider than where the rover may
    # DRIVE - the zones sit outside the driveable arena on purpose. This is the
    # faint outer box, and it is what makes the view wider than the arena.
    "obj_bounds": {"x_min": -100.0, "x_max": 1100.0,
                   "y_min": -100.0, "y_max": 1700.0},
}

MARKERS = [
    # M1 and M2: measured, from the mission stack. Do not edit.
    {"id": "M1", "x": 412.5, "y": 1357.5, "kind": "zone", "measured": True},
    {"id": "M2", "x": 842.5, "y": 1357.5, "kind": "zone", "measured": True},
    # M3: REQUESTED, NOT MEASURED. Placed by continuing the M1->M2 spacing
    # exactly - they are 430 mm apart at y=1357.5, so M3 sits 430 mm left of
    # M1 at x = -17.5. That is OUTSIDE the driveable arena, which is correct
    # for a zone: the geometry deliberately puts zones beyond the arena and
    # obj_bounds runs to x = -100.
    #
    # An earlier placeholder at x=157.5 was wrong and it showed on the map:
    # zone_radius is 131 mm, so two zones need 262 mm between centres, and
    # 412.5-157.5 = 255 made M3 overlap M1. Any replacement must clear 262 mm.
    {"id": "M3", "x": -17.5, "y": 1357.5, "kind": "zone", "measured": False},
    # Water refill station. ALSO NOT MEASURED - Rover 1 carries no water
    # hardware that has been identified, and no refill point is recorded
    # anywhere in the mission stack.
    {"id": "WATER", "x": 157.5, "y": 157.5, "kind": "water", "measured": False},
]

# Rover 1's footprint is NOT yet measured. Rover 2's is (see panel_rover.py
# BODY_MM), but that is a different chassis and reusing its numbers here would
# draw a body outline that is confidently wrong. Left null on purpose; the
# frontend draws no footprint until this is filled in.
BODY_MM = None



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

def state() -> dict:
    ip = rover_ip()
    return {
        "host": ROVER_HOST,
        "ip": ip,
        # IP, not the .local name - see rover_ip() for why.
        "rosbridge": "ws://%s:9090" % ip,
        "scan_topic": SCAN_TOPIC,
        "scan_type": SCAN_TYPE,
        "scan_url": "http://%s:8781/scan" % ip,
        "health_url": "http://%s:8781/health" % ip,
        "lidar": LIDAR,
        "arena": ARENA,
        "markers": MARKERS,
        "body_mm": BODY_MM,
        "body_note": "Rover 1's chassis has not been measured yet, so no "
                     "footprint is drawn. Rover 2's numbers are a different "
                     "chassis and would be wrong here.",
    }
