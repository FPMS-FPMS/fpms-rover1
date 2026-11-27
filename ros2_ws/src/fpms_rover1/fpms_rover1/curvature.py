#!/usr/bin/env python3
"""Curvature as the primitive. The base layer for continuous, fluid motion.

WHY CURVATURE AND NOT AN ANGLE
An angle is a discrete event: "turn 90 degrees" is a thing you start, finish
and stop. Curvature is a continuous quantity you can hold, ramp and blend, so
a path becomes a curvature PROFILE over distance rather than a queue of legs.
That difference is the whole reason this module exists: the operator does not
want rigid turns, and you cannot get fluid motion by sequencing discrete ones
however finely you chop them. The stop between legs is the problem, not the
leg length.

    kappa (k) = 1/R, signed. k > 0 curves LEFT, k = 0 is straight.
    omega = v * k

THE CHASSIS DOES NOT OBEY DIFFERENTIAL-DRIVE KINEMATICS
Measured 2026-10-09: outer 26 / inner +13 predicts a 172 mm radius from the
115 mm track. The measured radius was ~4200 mm. TWENTY-FOUR TIMES wider,
because skid-steer scrub resists yaw (wheelbase/track = 1.70, and you want
<= 1.0). So duty -> curvature MUST be a measured table, never a formula.
`ideal_kappa` exists only to show how far off the model is.

WHY IT REFUSES WHEN UNCALIBRATED
An empty table returns None, and callers must handle it. This project has
already been burned by the alternative: a counts-per-mm constant derived from
an assumption reported "travelled 200.4 mm of 200.0 mm" while the rover drove
1200 mm. Precise nonsense is worse than a refusal, because a refusal gets
fixed and nonsense gets trusted.
"""
from __future__ import annotations

import bisect
import math
from typing import List, Optional, Tuple

from fpms_rover1 import geometry as G

# A curvature this small is a straight line for our purposes: 1/8000 mm^-1 is
# an 8 m radius, wider than the arena is long.
STRAIGHT_KAPPA = 1.0 / 8000.0


def ideal_kappa(outer_pwm: float, inner_pwm: float) -> float:
    """Differential-drive curvature. REFERENCE ONLY - measured 24x off."""
    s = outer_pwm + inner_pwm
    if abs(s) < 1e-9:
        return math.inf
    return 2.0 * (outer_pwm - inner_pwm) / (G.TRACK * s)


class CurvatureMap:
    """Measured duty-split -> achieved curvature, with interpolation.

    A sample is (split, kappa_per_mm) where split is the NORMALISED duty
    difference, (outer - inner) / (outer + inner). Normalising matters: it
    makes the table reusable across speeds instead of valid at one duty pair.
    """

    def __init__(self) -> None:
        self._splits: List[float] = []
        self._kappas: List[float] = []

    def add(self, outer_pwm: float, inner_pwm: float, measured_kappa: float) -> None:
        s = outer_pwm + inner_pwm
        if abs(s) < 1e-9:
            raise ValueError('outer + inner must be non-zero to normalise')
        split = (outer_pwm - inner_pwm) / s
        i = bisect.insort_left and bisect.bisect_left(self._splits, split)
        self._splits.insert(i, split)
        self._kappas.insert(i, measured_kappa)

    @property
    def calibrated(self) -> bool:
        return len(self._splits) >= 2

    def kappa_for_split(self, split: float) -> Optional[float]:
        """Interpolate. None if there is not enough measured data."""
        if not self.calibrated:
            return None
        if split <= self._splits[0]:
            return self._kappas[0]
        if split >= self._splits[-1]:
            return self._kappas[-1]
        i = bisect.bisect_left(self._splits, split)
        x0, x1 = self._splits[i - 1], self._splits[i]
        y0, y1 = self._kappas[i - 1], self._kappas[i]
        t = (split - x0) / (x1 - x0)
        return y0 + t * (y1 - y0)

    def split_for_kappa(self, kappa: float) -> Optional[float]:
        """Invert the table. None if uncalibrated, or kappa is unreachable.

        Unreachable is reported rather than clipped silently: a caller asking
        for a 100 mm radius on a chassis that can only manage 400 needs to
        know, not to be quietly given 400 and left to wonder why it missed.
        """
        if not self.calibrated:
            return None
        pairs = sorted(zip(self._kappas, self._splits))
        ks = [k for k, _ in pairs]
        if kappa < ks[0] or kappa > ks[-1]:
            return None
        i = bisect.bisect_left(ks, kappa)
        if i == 0:
            return pairs[0][1]
        k0, s0 = pairs[i - 1]
        k1, s1 = pairs[i]
        if abs(k1 - k0) < 1e-12:
            return s0
        t = (kappa - k0) / (k1 - k0)
        return s0 + t * (s1 - s0)

    def duty_for(self, base_pwm: float, kappa: float) -> Optional[Tuple[int, int]]:
        """(left_pwm, right_pwm) for a signed curvature, or None.

        base_pwm is the MEAN duty, so speed and curvature are independent
        knobs - which is what lets a path hold its speed through a bend
        instead of slowing for every corner.
        """
        split = self.split_for_kappa(abs(kappa))
        if split is None:
            return None
        d = base_pwm * split
        outer, inner = base_pwm + d, base_pwm - d
        # kappa > 0 curves LEFT, so the LEFT side is the inner (slower) one.
        left, right = (inner, outer) if kappa > 0 else (outer, inner)
        return int(round(left)), int(round(right))

    def samples(self):
        return list(zip(self._splits, self._kappas))


class Slew:
    """Rate limiter. This is what makes the motion fluid rather than stepped.

    A STEP in curvature IS a rigid turn - the wheels jump to a new differential
    and the chassis snaps. Ramping curvature at a bounded rate is what reads as
    a purposeful curve. So the limiter is not a nicety bolted on top; it is the
    mechanism by which the motion stops being rigid.

    Limits per SECOND, applied against real elapsed time, so a late control
    cycle cannot smuggle through a bigger jump than it earned.
    """

    def __init__(self, max_rate: float, initial: float = 0.0) -> None:
        self.max_rate = float(max_rate)
        self.value = float(initial)

    def step(self, target: float, dt: float) -> float:
        if dt <= 0.0:
            return self.value
        lim = self.max_rate * dt
        d = target - self.value
        if d > lim:
            d = lim
        elif d < -lim:
            d = -lim
        self.value += d
        return self.value

    def reset(self, value: float = 0.0) -> None:
        self.value = float(value)


def radius_mm(kappa: float) -> float:
    return math.inf if abs(kappa) < STRAIGHT_KAPPA else 1.0 / abs(kappa)


def sweep_mm(kappa: float, angle_deg: float) -> float:
    """Path length to turn through an angle at a curvature.

    The number that caught my arc turn out: at the measured ~4200 mm radius a
    90 degree turn needs ~6.6 m of travel, which does not exist in a 1.2 m
    arena. Check this BEFORE committing to a curve, not against a wall.
    """
    if abs(kappa) < STRAIGHT_KAPPA:
        return math.inf
    return abs(math.radians(angle_deg)) / abs(kappa)


# ---------------------------------------------------------------------------
# ONE FLUID S-CURVE
#
# The team's existing S is three discrete legs: turn +theta, drive L, turn
# -theta. It works and it is NOT what we want here - every join is a stop, and
# a stop is the thing that makes motion rigid.
#
# A fluid S is ONE curvature profile: hold +k, ramp through 0, hold -k. The
# heading is restored by the SHAPE, exactly as the team's note argues, but
# nothing ever stops. Two equal opposite arcs of radius R through angle theta:
#
#     lateral = 2R(1 - cos theta)      sideways reach
#     along   = 2R sin theta           forward cost
#     path    = 2R theta               wheel distance travelled
#     net heading change = 0           by construction
#
# WHY AN S AND NOT A TURN-TOWARDS: the team simulated 400 trials per case on
# the measured chassis model. Turning to face the goal leaves 87 mm of error
# and cannot do better, because the chassis arcs while it turns and spends
# more position than it recovers. The S leaves 19 mm. Correct the ALONG
# component first - a straight line cannot make the rover crooked, and it
# shortens the S.
# ---------------------------------------------------------------------------

class SCurve:
    """A planned S: lateral offset with no net heading change.

    This is a PLAN, not a promise. Slew-limiting the curvature reversal
    rounds off the join, which costs a little lateral reach, and the chassis
    does not track a plan exactly anyway. Drive it closed-loop against
    measured pose and treat these numbers as the opening bid.
    """

    def __init__(self, lateral_mm: float, radius_mm: float):
        self.lateral = float(lateral_mm)
        self.radius = abs(float(radius_mm))
        self.sign = 1.0 if lateral_mm >= 0 else -1.0
        # d <= 4R or the arcs cannot reach that far sideways at this radius.
        self.reach_limit = 4.0 * self.radius
        self.feasible = abs(self.lateral) <= self.reach_limit
        if self.feasible and self.radius > 0.0:
            c = 1.0 - abs(self.lateral) / (2.0 * self.radius)
            c = max(-1.0, min(1.0, c))
            self.theta = math.acos(c)
        else:
            self.theta = float('nan')

    @property
    def along_mm(self) -> float:
        return 2.0 * self.radius * math.sin(self.theta) if self.feasible else float('nan')

    @property
    def path_mm(self) -> float:
        return 2.0 * self.radius * self.theta if self.feasible else float('nan')

    @property
    def peak_kappa(self) -> float:
        return self.sign / self.radius if self.radius > 0 else 0.0

    def phases(self):
        """[(kappa, path_mm), ...] - the profile to drive, in order.

        Two phases, one curvature reversal. The reversal is where the slew
        limiter earns its keep: a step from +k to -k is two rigid turns back
        to back, a ramp through zero is a single continuous S.
        """
        if not self.feasible:
            return []
        half = self.path_mm / 2.0
        return [(self.peak_kappa, half), (-self.peak_kappa, half)]

    def describe(self) -> str:
        if not self.feasible:
            return (f'S of {self.lateral:+.0f} mm NOT reachable at R '
                    f'{self.radius:.0f} mm (max {self.reach_limit:.0f} mm)')
        return (f'S {self.lateral:+.0f} mm lateral at R {self.radius:.0f} mm: '
                f'theta {math.degrees(self.theta):.1f} deg, '
                f'along {self.along_mm:.0f} mm, path {self.path_mm:.0f} mm')
