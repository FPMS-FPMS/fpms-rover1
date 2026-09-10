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

# Rover 1's footprint is NOT yet measured. Rover 2's is (see panel_rover.py
# BODY_MM), but that is a different chassis and reusing its numbers here would
# draw a body outline that is confidently wrong. Left null on purpose; the
# frontend draws no footprint until this is filled in.
BODY_MM = None


def state() -> dict:
    return {
        "host": ROVER_HOST,
        "rosbridge": ROSBRIDGE_WS,
        "scan_topic": SCAN_TOPIC,
        "scan_type": SCAN_TYPE,
        "scan_url": BRIDGE + "/scan",
        "health_url": BRIDGE + "/health",
        "lidar": LIDAR,
        "body_mm": BODY_MM,
        "body_note": "Rover 1's chassis has not been measured yet, so no "
                     "footprint is drawn. Rover 2's numbers are a different "
                     "chassis and would be wrong here.",
    }
