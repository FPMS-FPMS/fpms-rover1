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
# Live pose in ARENA MILLIMETRES from odom_node. The map is fixed; the rover
# moves on it. Before this existed the panel drew everything from an assumed
# pose at HOME, which was only true until the rover moved.
POSE_TOPIC = "/rover/arena_pose"
POSE_TYPE = "geometry_msgs/msg/PoseStamped"

# Rover 2. It runs its own stack on 192.168.137.18 and, when it is up and
# publishing an arena pose of its own, this is where Rover 1 would read it.
# Until then the field is None and the UI says Rover 2 is unknown rather than
# drawing it somewhere plausible.
ROVER2 = {
    "host": "192.168.137.18",
    "pose_topic": "/rover/arena_pose",
    "pose_type": "geometry_msgs/msg/PoseStamped",
    "rosbridge": "ws://192.168.137.18:9090",
    # Rover 2 stands bottom-RIGHT, at the mission stack's HOME. Used only as a
    # fallback: a LIVE pose from its own rosbridge always wins, because a
    # station is where it was put, not where it is once it moves.
    "station": {"x": 842.5, "y": 157.5, "heading_deg": 90.0},
    "note": "Rover 2 has not been reachable this session, so its station is "
            "used. A live pose from its own rosbridge overrides it.",
}

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
    # Operator geometry, 2026-09-10: "M3 is 300 mm to the left of M1" AND
    # "M1 should be centred between M2 and M3". Solving both together with M2
    # fixed gives an even 300 mm pitch:
    #     M1 = (M2 + M3)/2  and  M3 = M1 - 300   =>   M1 = M2 - 300
    # 300 mm clears the 262 mm two-zone minimum (zone_radius 131) by 38 mm.
    #
    # The mission stack was moved to match on 2026-09-10 - geometry.py in BOTH
    # rover-backup trees, and panel_field.py - so Rover 2 drives to the same M1
    # this panel draws. The divergence that existed for one commit is closed.
    #
    # STILL DIVERGENT, and it predates this change: INTERNATIONALS_ROVER2.py
    # works in a DIFFERENT frame entirely - its TARGET_COORDS put m1 at
    # (90, 1110) and m2 at (880, 1075), 820 mm apart against this frame's 300.
    # It derives them from zone fractions rather than storing coordinates, so
    # it cannot simply be set to 542.5; the derivation would have to change.
    {"id": "M3", "x": 242.5, "y": 1357.5, "kind": "zone", "measured": True},
    {"id": "M1", "x": 542.5, "y": 1357.5, "kind": "zone", "measured": True},
    {"id": "M2", "x": 842.5, "y": 1357.5, "kind": "zone", "measured": True},
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
        "pose_topic": POSE_TOPIC,
        "pose_type": POSE_TYPE,
        "rover2": ROVER2,
        # Rover 2's mission controls, surfaced here at the operator's request.
        # They act on ROVER 2 over ssh, not on Rover 1 - the panel labels them
        # so nobody presses "start" expecting Rover 1 to move.
        "mission_panel": "mission",
        "mission_actions": ["start", "stop", "restart", "shutdown"],
        "mission_target": "192.168.137.18 (Rover 2)",
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
