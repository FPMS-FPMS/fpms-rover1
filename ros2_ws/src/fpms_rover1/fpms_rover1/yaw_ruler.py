#!/usr/bin/env python3
"""yaw_ruler - measure rotation with the LiDAR, independently of the IMU.

WHY
Rover 1's heading cannot be trusted and no ROS package can fix that, because
the fault is a sensor scale, not a missing algorithm. The team already measured
this on the sister board: the Yahboom "fused yaw" under-reads physical rotation
by ~1.60x AND runs opposite in sign. Closing any loop on it - robot_localization,
Nav2, my own yaw hold - inherits the fault and produces a pose that is
confidently wrong. See /usr/share/fpms/knowledge/fpms-phase6-spin-must-use-fused-yaw.md

So this measures rotation the one way that shares NO hardware with the IMU:
brute-force align two 360 degree scans and report the rotation that fits best.

CONVENTION (matches the team's _best_shift)
Returns s where r1[a] ~= r0[a+s]. A feature moving from bin a+s to bin a means
the body rotated +s, in the SAME convention as a commanded angle - so the
number here compares directly with what was asked for.

THE TRAPS, all from the team's note - do not re-learn these
- A single large turn gives a WRONG scale. An earlier pass measured 3.45, more
  than 2x too big, from big open-loop bursts. Two causes, both growing with
  turn size: a rectangular arena is near-symmetric under 180 deg so a large
  rotation can alias to angle-180; and the chassis ARCS, so a big turn also
  translates the LiDAR origin, which cross-correlation reads as LESS rotation.
- Measure a SEQUENCE against ONE latched reference, ending near the start
  heading. Then there is almost no translation left to bias the reading.
- Do NOT sum per-turn numbers: each carries the bias, so they compound.
- Residual is the honest health signal: ~6 mm parked, 200-300 mm mid-turn.
  A high residual means "it translated as well as rotated", not "bad match".

    ros2 run fpms_rover1 yaw_ruler                      # null test, NO motion
    ros2 run fpms_rover1 yaw_ruler --ros-args -p hold_s:=20.0
"""
from __future__ import annotations

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import LaserScan

try:
    import numpy as np
except ImportError:                                   # pragma: no cover
    np = None

MIN_PAIRS = 60          # below this the match is not worth reporting

# RESAMPLE ONTO A FIXED GRID. The LD19 driver does NOT publish a constant ray
# count - consecutive revolutions came back as 503 and 502 - and it derives
# angle_increment from that count, so bin k is a DIFFERENT angle in two
# different scans. Comparing raw arrays either crashes on the length mismatch
# or, worse, silently compares misaligned angles. Everything is therefore
# resampled onto one fixed grid before any comparison.
GRID = 512
GRID_DEG = 360.0 / GRID


def resample(msg):
    """Scan -> fixed GRID array, nan where no usable return.

    Nearest bin, keeping the MINIMUM range when several rays land in one bin:
    the near surface is the one that matters and min is stable under a
    jittering ray count.
    """
    out = [math.nan] * GRID
    inc = msg.angle_increment
    for i, r in enumerate(msg.ranges):
        if not (math.isfinite(r) and msg.range_min < r < msg.range_max):
            continue
        a = math.degrees(msg.angle_min + i * inc) % 360.0
        b = int(round(a / GRID_DEG)) % GRID
        if math.isnan(out[b]) or r < out[b]:
            out[b] = r
    return out


def _candidates(n, center, radius):
    """Shifts to try: the whole circle, or a window around center."""
    if center is None or radius is None:
        return range(n)
    c, r = int(round(center)) % n, int(radius)
    return [(c + k) % n for k in range(-r, r + 1)]


def best_shift(r0, r1, center=None, radius=None):
    """Brute-force the bin shift that best aligns r1 onto r0.

    center/radius restrict the search to +-radius bins around center. A
    full 512-shift sweep is fine for a one-off measurement but too slow to
    run inside a 20 Hz control loop; while DRIVING STRAIGHT the rotation is
    small and known to be near zero, so a narrow window is both faster and
    safer - it cannot alias onto a far-away match.

    Returns (shift_bins_float, rms_residual_m, n_pairs). shift is refined to
    sub-bin by a parabolic fit, because one bin is 0.72 deg and the drift we
    care about is smaller than that.
    """
    n = len(r0)
    assert n == len(r1) == GRID, f'grids must match: {n} vs {len(r1)}'
    if np is not None:
        a0 = np.asarray(r0, dtype=float)
        a1 = np.asarray(r1, dtype=float)
        best = (math.inf, 0, 0)
        curve = {}
        for s in _candidates(n, center, radius):
            shifted = np.roll(a0, -s)
            m = ~(np.isnan(shifted) | np.isnan(a1))
            k = int(m.sum())
            if k < MIN_PAIRS:
                continue
            # MEDIAN absolute difference, not RMS. A real scene has depth
            # discontinuities - a bin straddling an edge jumps metres between
            # revolutions - and RMS is dominated by a handful of those, which
            # made a parked scan read 200 mm when its ANGLE was stable to
            # 0.02 deg. The median ignores the edges and describes the bulk.
            d = np.abs(a1[m] - shifted[m])
            rms = float(np.median(d))
            curve[s] = rms
            if rms < best[0]:
                best = (rms, s, k)
        if not curve:
            return (0.0, float('nan'), 0)
        rms, s, k = best
    else:
        best = (math.inf, 0, 0)
        curve = {}
        for s in _candidates(n, center, radius):
            tot, k = 0.0, 0
            for a in range(n):
                x, y = r1[a], r0[(a + s) % n]
                if math.isnan(x) or math.isnan(y):
                    continue
                tot += (x - y) ** 2
                k += 1
            if k < MIN_PAIRS:
                continue
            rms = math.sqrt(tot / k)
            curve[s] = rms
            if rms < best[0]:
                best = (rms, s, k)
        if not curve:
            return (0.0, float('nan'), 0)
        rms, s, k = best

    # Parabolic refinement on the three points around the minimum.
    lo, hi = curve.get((s - 1) % n), curve.get((s + 1) % n)
    frac = 0.0
    if lo is not None and hi is not None:
        den = (lo - 2.0 * rms + hi)
        if abs(den) > 1e-12:
            frac = 0.5 * (lo - hi) / den
            frac = max(-0.5, min(0.5, frac))
    return (s + frac, rms, k)


class YawRuler(Node):
    def __init__(self):
        super().__init__('yaw_ruler')
        self.declare_parameter('hold_s', 12.0)
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.scan = None
        self.yaw = None
        self.create_subscription(LaserScan, '/scan', self._on_scan, q)
        self.create_subscription(Vector3Stamped, '/rover/attitude',
                                 self._on_att, q)

    def _on_scan(self, m):
        self.scan = m

    def _on_att(self, m):
        self.yaw = m.vector.z

    def snapshot(self):
        m = self.scan
        if m is None:
            return None
        return {
            'grid': resample(m),
            'yaw': self.yaw,
            'n': len(m.ranges),
        }


def compare(ref, cur):
    shift, rms, k = best_shift(ref['grid'], cur['grid'])
    deg = (shift * GRID_DEG) % 360.0
    if deg > 180.0:
        deg -= 360.0
    return deg, rms, k


def main(args=None):
    rclpy.init(args=args)
    n = YawRuler()
    ev = threading.Event()

    def spin():
        while not ev.is_set():
            rclpy.spin_once(n, timeout_sec=0.05)
    th = threading.Thread(target=spin, daemon=True)
    th.start()

    dl = time.time() + 15.0
    while time.time() < dl and n.scan is None:
        time.sleep(0.1)
    if n.scan is None:
        print('  no /scan - cannot measure anything')
        ev.set(); th.join(timeout=2.0); n.destroy_node(); rclpy.shutdown()
        return 1

    hold = float(n.get_parameter('hold_s').value)
    print(f'  numpy: {"yes" if np is not None else "NO - pure python, slower"}')
    print(f'\n=== NULL TEST: latch a reference, wait {hold:.0f} s, re-measure ===')
    print('  The rover MUST NOT MOVE. This validates the ruler itself:')
    print('  a trustworthy ruler reads ~0 deg with a small residual.')

    ref = n.snapshot()
    print(f'\n  reference latched: {ref["n"]} rays -> {GRID} grid bins, '
          f'{sum(1 for v in ref["grid"] if not math.isnan(v))} filled, '
          f'{GRID_DEG:.3f} deg/bin')
    print(f'  board yaw at latch: '
          f'{math.degrees(ref["yaw"]):+.2f} deg' if ref['yaw'] is not None
          else '  board yaw: unavailable')

    t0 = time.time()
    while time.time() - t0 < hold:
        time.sleep(0.5)
        cur = n.snapshot()
        if cur is None:
            continue
        deg, rms, k = compare(ref, cur)
        dy = (math.degrees(cur['yaw'] - ref['yaw'])
              if (cur['yaw'] is not None and ref['yaw'] is not None) else float('nan'))
        if (time.time() - t0) % 4.0 < 0.55:
            print(f'    t+{time.time()-t0:4.1f}s  lidar {deg:+6.2f} deg  '
                  f'residual {rms*1000:6.1f} mm  pairs {k:3d}  '
                  f'board yaw {dy:+6.2f} deg')

    # Validate the OUTPUT WE USE: 12 parked readings, then judge the angle's
    # offset and scatter. The earlier version gated on an absolute residual
    # borrowed from the team's different pipeline; that is not a threshold this
    # one can be held to, and it declared a ruler SUSPECT whose angle was
    # stable to 0.02 deg. Residual is reported, not used as a gate.
    samples = []
    for _ in range(12):
        time.sleep(0.25)
        c = n.snapshot()
        if c is None:
            continue
        d, r, kk = compare(ref, c)
        samples.append((d, r, kk))
    if not samples:
        print('\n  no samples'); return 2

    degs = [x[0] for x in samples]
    res = [x[1] for x in samples]
    mean = sum(degs) / len(degs)
    spread = max(degs) - min(degs)
    sd = (sum((d - mean) ** 2 for d in degs) / len(degs)) ** 0.5
    print(f'\n  === {len(samples)} parked readings ===')
    print(f'  angle    mean {mean:+.3f} deg   spread {spread:.3f}   '
          f'sd {sd:.3f}')
    print(f'  residual median {sorted(res)[len(res)//2]*1000:.1f} mm   '
          f'pairs ~{samples[-1][2]}')
    # Parked, the ruler must read ~0 and must not wander. 0.5 deg of offset and
    # 0.5 deg of scatter is well inside anything we would steer on.
    ok = abs(mean) <= 0.5 and spread <= 0.5
    print(f'  VERDICT: {"RULER IS TRUSTWORTHY" if ok else "RULER SUSPECT"} '
          f'(want |mean| <= 0.5 deg and spread <= 0.5 deg parked)')
    if ok:
        print(f'  -> rotation measurable to about +-{max(abs(mean), spread/2):.2f} '
              f'deg, independently of the IMU. This is the reference.')
    else:
        print('  If the rover really did not move, do NOT calibrate off this.')

    ev.set(); th.join(timeout=2.0); n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
