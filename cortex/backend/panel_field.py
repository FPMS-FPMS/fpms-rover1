#!/usr/bin/env python3
"""panel_field - 3D FIELD panel backend for the FPMS app.

An RViz/Foxglove-style world view: the live LiDAR, the rover, the arena, and a
set of objects the operator places by hand - trees, obstacles, zones - so the
scan can be read against what is actually out there.

WHAT THIS BACKEND IS FOR
------------------------
Only two things:

  1. PERSISTENCE. The scene the operator builds has to survive a panel switch,
     an app restart and a reboot, so it lives in a file rather than in the
     page. Everything else about the scene is the browser's business.
  2. THE FRAME DEFINITIONS. The arena geometry and the chassis dimensions are
     measured facts about this robot, and they belong next to the other
     measured facts rather than duplicated in JavaScript.

It does NOT touch the LiDAR. The scan reaches the browser over rosbridge
(ws://<rover>:9090) exactly as it does for panel_rover, and is transformed
into arena coordinates client-side using the pose the rover already publishes.
Routing point clouds through here would add a hop and a JSON re-encode at
10 Hz for no gain.

COORDINATE FRAME - stated once, here, so the JS cannot drift from it
-------------------------------------------------------------------
World is the ARENA frame, millimetres:

    +X right, +Y forward, +Z up.  Heading 90 deg = +Y.

That is the frame `~/run-live` publishes pose and obstacles in, so a scene
built here lines up with the View Planner's picture without a second
convention to reconcile.

The scanner reports in its own frame (x forward, y left, angle 0 dead ahead).
The transform from scan to world is:

    world = rover_xy + R(heading) * (SCANNER_OFFSET + scan_point)

SCANNER_OFFSET is not zero and that matters: the scanner sits 200 mm behind
the front tip of a 320 mm chassis, i.e. 40 mm FORWARD of the chassis centre.
Treating the scanner as the rover's centre puts every returned point 40 mm too
close, which is the same order as the docking tolerance this robot works to.
"""

import json
import os
import tempfile
import time

STATE_DIR = os.path.expanduser("~/.local/share/fpms")
SCENE_PATH = os.path.join(STATE_DIR, "field-scene.json")

# ---------------------------------------------------------------- geometry
# TAKEN VERBATIM FROM THE MISSION STACK, not re-measured here.
#   ~/rover-backup/FPMSM2PERFECT/fpms_rover2/geometry.py
#   ~/rover-backup/fpms_rover2-M2-HOME-26MM-20260904/fpms_rover2/geometry.py
# Those two agree exactly. geometry.py carries this warning and it applies to
# this file too: "DO NOT re-measure these into something else. Every failure of
# the 2026-09-02 session traced to a zone constant that had drifted."
#
# NOTE the older ~/FPMS/INTERNATIONALS_ROVER2.py (phase6) uses
# LIDAR_TO_FRONT = 160, which DISAGREES with the 200 used by the stack that
# actually works. The working stack wins; the discrepancy is flagged in the UI.
ARENA = {
    "w": 1000.0,                       # ARENA_W_MM
    "h": 1200.0,                       # ARENA_H_MM
    "home": {"x": 842.5, "y": 157.5, "heading_deg": 90.0},   # HOME_XY / HOME_HEADING_DEG
    # Zone centres. "M2 is exactly 1000mm in front of the TIP of the rover";
    # the tip is LIDAR_TO_FRONT ahead of the scanner, so
    #   y = HOME_y + 1000 + 200 = 1357.5
    # M1 moved 412.5 -> 542.5 on 2026-09-10 by instruction, in step with
    # geometry.py. Three zones now, at an even 300 mm pitch: M3 242.5,
    # M1 542.5, M2 842.5. Not a re-measurement - a deliberate change.
    "zones": {"M1": {"x": 542.5, "y": 1357.5},
              "M2": {"x": 842.5, "y": 1357.5}},
    "zone_radius": 131.0,              # ZONE_RADIUS_MM - the vases are 263mm across
    # The rover CENTRE may not go closer than this to an edge, so no part of
    # the BODY leaves the floor. Larger than the half-width because a turn
    # sweeps an arc.
    "edge_keep": 150.0,                # EDGE_KEEP_MM = BODY_HALF_MM + 20
    # Where the rover may DRIVE and where an object may plausibly BE are
    # deliberately different bounds - the zones sit outside the driveable
    # arena on purpose. Drawn as a faint outer box.
    "obj_bounds": {"x_min": -100.0, "x_max": 1100.0,
                   "y_min": -100.0, "y_max": 1700.0},
}

# Chassis, millimetres, measured FROM THE SCANNER - which is the origin of the
# rover frame in geometry.py (footprint_m() is built around the LiDAR at 0,0).
#
# THIS IS WHY THERE IS NO SCANNER OFFSET. The pose the stack publishes IS the
# scanner position, not the chassis centre. An earlier version of this panel
# assumed the centre and added 40 mm forward, which biased every transformed
# scan point by 40 mm - the same order as this robot's docking tolerance.
BODY = {"front": 200.0,                # LIDAR_TO_FRONT_MM
        "back": -120.0,                # -LIDAR_TO_BACK_MM
        "left": 130.0, "right": -130.0,
        "half_width": 130.0,
        "length": 320.0, "width": 260.0,       # BODY_LENGTH_MM / BODY_WIDTH_MM
        "circumscribed": 239.0,                # CIRCUMSCRIBED_MM
        "pivot_radius": 143.0,                 # PIVOT_RADIUS_MM
        "pose_is_scanner": True}

# What the operator may place. The browser draws these; the list lives here so
# the two cannot disagree about what a valid object is.
KINDS = {
    "tree":     {"label": "Tree",     "default": {"radius": 150, "height": 900}},
    "obstacle": {"label": "Obstacle", "default": {"w": 200, "d": 200, "height": 300}},
    "zone":     {"label": "Zone",     "default": {"w": 400, "d": 400}},
    "post":     {"label": "Post",     "default": {"radius": 40, "height": 600}},
}

MAX_OBJECTS = 400          # a scene, not a point cloud dump
MAX_BYTES = 1 << 20        # 1 MiB of JSON is already an unreasonable scene


def _empty():
    return {"version": 1, "objects": [], "saved": None}


def _read():
    try:
        with open(SCENE_PATH) as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        return _empty()
    if not isinstance(d, dict) or not isinstance(d.get("objects"), list):
        return _empty()
    return d


def _write(scene):
    """Write the scene atomically.

    A half-written scene file is worse than no scene: the panel would load a
    truncated JSON, fall back to empty, and the operator's layout would be
    silently gone. Write to a temp file in the same directory and rename.
    """
    os.makedirs(STATE_DIR, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=STATE_DIR, prefix=".field-", suffix=".json")
    try:
        with os.fdopen(fd, "w") as fh:
            json.dump(scene, fh, separators=(",", ":"))
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, SCENE_PATH)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _valid(obj):
    if not isinstance(obj, dict):
        return False
    if obj.get("kind") not in KINDS:
        return False
    for key in ("x", "y"):
        try:
            float(obj.get(key))
        except (TypeError, ValueError):
            return False
    return True


def state() -> dict:
    scene = _read()
    return {
        "arena": ARENA,
        "body": BODY,
        "kinds": KINDS,
        "max_objects": MAX_OBJECTS,
        "scene": scene,
        "scene_path": SCENE_PATH,
        "count": len(scene.get("objects") or []),
        # Repeated from panel_rover so this panel is self-sufficient: it opens
        # its own socket rather than depending on another panel being mounted.
        "rosbridge": "ws://192.168.137.18:9090",
        "note": ("world frame is the ARENA: +X right, +Y forward, +Z up, "
                 "heading 90 deg = +Y, millimetres. Constants are taken "
                 "verbatim from FPMSM2PERFECT/fpms_rover2/geometry.py. The "
                 "published pose IS the scanner, so no offset is applied."),
    }


def act_save(params):
    """Replace the whole scene. The browser owns the layout; this stores it."""
    objs = params.get("objects")
    if not isinstance(objs, list):
        return {"ok": False, "error": "objects must be a list"}
    if len(objs) > MAX_OBJECTS:
        return {"ok": False,
                "error": "too many objects (%d), limit is %d"
                         % (len(objs), MAX_OBJECTS)}
    clean = [o for o in objs if _valid(o)]
    dropped = len(objs) - len(clean)
    # A save is a WHOLE-SCENE replace, so dropping invalid entries silently is
    # how an operator's layout disappears: send one malformed object and the
    # file is rewritten without the rest. If everything was rejected but the
    # caller sent something, that is a bug on the wire, not an empty scene -
    # refuse and leave the stored scene alone.
    # An EMPTY save is how a hand-built layout disappears without anyone
    # touching "Clear all": one stray interaction that leaves the panel with
    # no objects, one debounced save, and the file is gone. Emptying the scene
    # is what act_clear is for, and that asks for a token. So a save may
    # shrink a scene but never empty one unless it says so explicitly.
    if not objs and not params.get("allow_empty"):
        return {"ok": False,
                "error": "refusing to save an empty scene - use the clear "
                         "action if you really mean to wipe it",
                "detail": "pass allow_empty:true to override"}
    if objs and not clean:
        return {"ok": False,
                "error": "every object was rejected (%d sent) - refusing to "
                         "overwrite the saved scene with nothing" % len(objs),
                "kinds": sorted(KINDS)}
    scene = {"version": 1, "objects": clean, "saved": time.time()}
    blob = json.dumps(scene, separators=(",", ":"))
    if len(blob) > MAX_BYTES:
        return {"ok": False, "error": "scene is larger than 1 MiB"}
    try:
        _write(scene)
    except OSError as exc:
        return {"ok": False, "error": "could not write scene: %s" % exc}
    return {"ok": True, "count": len(clean), "dropped": dropped,
            "path": SCENE_PATH}


def act_clear(params):
    """Wipe the scene. Requires a token - it is not undoable."""
    if str(params.get("confirm", "")) != "CLEAR":
        return {"ok": False, "error": "confirmation required",
                "detail": "clearing the scene cannot be undone"}
    try:
        _write(_empty())
    except OSError as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "count": 0}


def actions() -> dict:
    return {"save": act_save, "clear": act_clear}
