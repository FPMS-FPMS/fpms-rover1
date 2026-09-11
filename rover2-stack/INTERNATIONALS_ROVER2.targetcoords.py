"""Excerpt from ~/FPMS/INTERNATIONALS_ROVER2.py - the reframed TARGET_COORDS
block only. The full file is 243 KB and is not vendored here; this is kept so
the change is reviewable and restorable.
"""

# Fixed target coordinates in mm (Tesla: known map, known destinations)
#
# REFRAMED 2026-09-10 to match geometry.py / panel_field.py. This file used to
# derive its zones from fraction constants (ZONE_FRAC 0.10, MARGIN_FRAC 0.04),
# which put m1 at (90, 1110) and m2 at (910, 1110) - 820 mm apart, against the
# mission stack's 300 mm pitch at y = 1357.5. The two stacks disagreed about
# where the zones ARE, which is a worse problem than either value being wrong.
#
# Now both stacks share one definition. Previous values, for the record:
#     m1 (90.0, 1110.0)    m2 (880.0, 1075.0)    home (842.5, 157.5)
#
# THE ZONE CENTRES ARE NOT DRIVEABLE. They sit at y = 1357.5, outside the
# 1200 mm arena, exactly as geometry.py has them - the zones are beyond the
# floor on purpose. So the drive targets are DERIVED as the nearest legal pose
# inside the arena, never typed in, which is the same principle this block
# always used. The old hand-patched m2 of (880, 1075) existed because the zone
# centre (910, 1110) put the body through the wall; the derivation below makes
# that impossible rather than relying on someone noticing.
ZONE_CENTRES = {
    "m3": (242.5, 1357.5),
    "m1": (542.5, 1357.5),
    "m2": (842.5, 1357.5),
}
# Body clearance from a wall. ROBOT_RADIUS is half the 230 mm width; the extra
# 10 mm keeps the body off the wall rather than touching it.
_TARGET_MARGIN = ROBOT_RADIUS + 10.0


def _legal_pose(cx, cy):
    """Nearest pose inside the arena that keeps the whole body on the floor."""
    m = _TARGET_MARGIN
    return {"x": min(max(cx, ARENA_X_MIN + m), ARENA_X_MAX - m),
            "y": min(max(cy, ARENA_Y_MIN + m), ARENA_Y_MAX - m)}


TARGET_COORDS = {k: _legal_pose(*v) for k, v in ZONE_CENTRES.items()}
TARGET_COORDS["home"] = {"x": 842.5, "y": 157.5}
