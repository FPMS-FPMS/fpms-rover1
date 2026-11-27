#!/usr/bin/env python3
"""Rover 1 chassis geometry. MEASURED by the team 2026-10-09 with a tape.

Every number here replaces a guess, and two of those guesses were wrong in
ways that mattered:

  - Clearance margin was a flat 250 mm in all four directions. The real body
    is ASYMMETRIC and mostly SMALLER, so the guess was over-conservative: it
    refused turns and legs that would have fitted. The arc turn aborted at
    "216 mm ahead" when the body front was only 80 mm past the scanner, i.e.
    there was 136 mm of real room left.

  - Track width was BACK-DERIVED from a stall test as 418-501 mm. It is 115.
    Off by 4x, which inverted the diagnosis: the wheels were not rolling as
    the rotation required, they were slipping 3.6-4.4x.

Never derive a chassis dimension from the behaviour you are trying to explain.
"""

# -- body extents from the LiDAR's rotation centre, mm ---------------------
LIDAR_TO_FRONT = 80.0
LIDAR_TO_BACK = 170.0
LIDAR_TO_LEFT = 120.0
LIDAR_TO_RIGHT = 120.0

# The scanner is NOT central: 80 forward vs 170 back. So the body reaches
# 90 mm further behind the scanner than in front of it, and a reverse leg
# needs correspondingly more room than a forward one.
BODY_LENGTH = LIDAR_TO_FRONT + LIDAR_TO_BACK      # 250
BODY_WIDTH = LIDAR_TO_LEFT + LIDAR_TO_RIGHT       # 240

# Worst-case radius swept about the scanner, used for turn clearance: the
# corner furthest from the rotation centre.
import math
TURN_SWEEP_RADIUS = math.hypot(max(LIDAR_TO_FRONT, LIDAR_TO_BACK),
                               max(LIDAR_TO_LEFT, LIDAR_TO_RIGHT))   # ~208

# -- drive geometry, mm ----------------------------------------------------
WHEELBASE = 195.0          # front axle centre -> rear axle centre
TRACK = 115.0              # left contact centre -> right contact centre
WHEEL_DIAMETER = 80.0
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER     # 251.3

MASS_KG = 0.400
TYRES = 'Yahboom spiky off-road soft rubber'

# WHY THIS CHASSIS WILL NOT SPIN IN PLACE.
# Skid-steer yaw requires scrubbing all four tyres sideways. The resistance
# scales with how far the front/rear tyres sit from the yaw centre relative to
# the track, i.e. with WHEELBASE/TRACK. At or below 1.0 a skid-steer turns
# readily; this chassis is 1.70. Add high-grip spiky rubber and only 400 g of
# normal force to work with, and a full-duty spin manages ~0.28 deg/s.
# That is geometry. No duty setting fixes it - see
# [[rover1-cannot-turn-in-place]] and use arc turns.
SCRUB_RATIO = WHEELBASE / TRACK                   # 1.70

# A pure differential-drive turn radius, for REFERENCE ONLY. The chassis does
# not obey it: at outer 26 / inner +13 this predicts 172 mm and the measured
# radius was ~4200 mm, 24x wider, because scrub resists yaw. Quote the
# measured radius, never this.
def ideal_radius_mm(outer_pwm: float, inner_pwm: float) -> float:
    d = outer_pwm - inner_pwm
    if abs(d) < 1e-9:
        return float('inf')
    return (TRACK / 2.0) * (outer_pwm + inner_pwm) / d


# -- counts per mm: STILL CONTESTED ---------------------------------------
# Two readings disagree by 25% and the wheel diameter cannot break the tie
# without the encoder spec:
#     4.968 counts/mm -> 1248.6 counts per wheel revolution  (from "980 mm")
#     6.220 counts/mm -> 1563.3 counts per wheel revolution  (from "~200 mm")
# Neither is obviously the gearbox figure without a datasheet.
#
# THE DECIDER needs no tape and no driving: lift the wheels clear of the
# ground, mark one, hand-rotate it exactly 10 turns, read the encoder delta.
#     counts_per_mm = (delta / 10) / 251.33
# Off the ground there is no slip, no surface and no tape error, which is why
# this settles what two ground measurements could not.
COUNTS_PER_REV_IF_4968 = 4.968 * WHEEL_CIRCUMFERENCE
COUNTS_PER_REV_IF_6220 = 6.220 * WHEEL_CIRCUMFERENCE
