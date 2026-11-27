#!/usr/bin/env python3
"""motion_seq - drive forward, then back, on ROS actions, with LiDAR gating.

WHY THIS EXISTS
Every motion test so far has been an ad-hoc script. This is the installed,
repeatable version: one command, both legs, a live clearance check before and
during each leg, and a report that puts the two directions side by side.
Forward and reverse on this chassis are NOT symmetric - reverse ran 18% slower
at the same duty and drifted 3x further before heading hold - so a test that
does both and prints them together is the one worth keeping.

WHAT IT REFUSES TO DO
- Drive a leg whose travel plus margin exceeds the measured clearance in that
  direction. It reports and skips that leg rather than trying a shorter one:
  silently driving something other than what was asked is worse than stopping.
- Drive without encoder data, or without a scan. Both are checked first.
- Trust a clearance reading taken before the previous leg. Each leg re-reads.

    ros2 run fpms_rover1 motion_seq
    ros2 run fpms_rover1 motion_seq --ros-args -p forward_mm:=500.0 \
                                              -p back_mm:=500.0 -p pwm:=14.0
"""
from __future__ import annotations

import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty

from fpms_rover1_msgs.action import DriveDistance

# Scanner-to-body-front is UNMEASURED on Rover 1 - one of the outstanding tape
# measurements. 250 mm is a deliberately conservative stand-in, not a fact.
DEFAULT_MARGIN_MM = 250.0
CONE_DEG = 20.0          # half-width of the forward cone
REAR_DEG = 160.0         # |angle| beyond this counts as behind


class MotionSeq(Node):
    def __init__(self):
        super().__init__('motion_seq')
        self.declare_parameter('forward_mm', 800.0)
        self.declare_parameter('back_mm', 800.0)
        self.declare_parameter('pwm', 14.0)
        self.declare_parameter('margin_mm', DEFAULT_MARGIN_MM)
        self.declare_parameter('settle_s', 2.0)

        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.fwd = None
        self.rear = None
        self.enc = None
        self.odom = None
        self.yaw = None
        self.scan_at = 0.0
        self.enc_at = 0.0
        self.tripped = False

        self.create_subscription(LaserScan, '/scan', self._on_scan, q)
        self.create_subscription(Int32MultiArray, '/rover/encoders',
                                 self._on_enc, q)
        self.create_subscription(Odometry, '/odom', self._on_odom, q)
        self.create_subscription(Vector3Stamped, '/rover/attitude',
                                 self._on_att, q)
        self.ac = ActionClient(self, DriveDistance, '/drive_distance')
        self.stop_cli = self.create_client(Empty, '/drive/stop')
        self.reset_cli = self.create_client(Empty, '/reset_odom')

    # -- inputs ------------------------------------------------------------
    def _on_enc(self, m):
        self.enc = list(m.data)[:4]
        self.enc_at = time.time()

    def _on_odom(self, m):
        self.odom = (m.pose.pose.position.x * 1000.0,
                     m.pose.pose.position.y * 1000.0)

    def _on_att(self, m):
        self.yaw = m.vector.z

    def _on_scan(self, m):
        f, r = [], []
        for i, rng in enumerate(m.ranges):
            if not (math.isfinite(rng) and m.range_min < rng < m.range_max):
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            a = (a + 180.0) % 360.0 - 180.0
            if abs(a) <= CONE_DEG:
                f.append(rng * 1000.0)
            elif abs(a) >= REAR_DEG:
                r.append(rng * 1000.0)
        # 10th percentile, not the single minimum: one spurious short ray must
        # not abort a good run, but a real surface fills many rays at once.
        if len(f) >= 5:
            f.sort()
            self.fwd = f[len(f) // 10]
        if len(r) >= 5:
            r.sort()
            self.rear = r[len(r) // 10]
        self.scan_at = time.time()

    # -- helpers -----------------------------------------------------------
    def clearance(self, forward: bool):
        return self.fwd if forward else self.rear

    def estop(self, why: str):
        self.tripped = True
        self.get_logger().warn(f'ABORT: {why}')
        if self.stop_cli.wait_for_service(timeout_sec=2.0):
            self.stop_cli.call_async(Empty.Request())

    def leg(self, dist_mm: float, pwm: float, margin: float) -> dict:
        """One leg. Returns a dict of what happened; never raises."""
        forward = dist_mm >= 0.0
        name = 'FORWARD' if forward else 'BACKWARD'
        out = {'leg': name, 'requested_mm': abs(dist_mm), 'driven': False}

        gap = self.clearance(forward)
        need = abs(dist_mm) + margin
        print(f'\n--- {name} {abs(dist_mm):.0f} mm ---')
        if gap is None:
            out['why'] = 'no scan in that direction'
            print(f'  SKIPPED: {out["why"]}')
            return out
        print(f'  clearance {gap:.0f} mm   need {need:.0f} mm '
              f'({abs(dist_mm):.0f} + {margin:.0f} margin)')
        if gap < need:
            out['why'] = f'only {gap:.0f} mm, need {need:.0f}'
            out['clearance_mm'] = gap
            print(f'  SKIPPED: {out["why"]} - not driving')
            return out
        if self.enc is None or (time.time() - self.enc_at) > 1.0:
            out['why'] = 'no fresh encoder data'
            print(f'  SKIPPED: {out["why"]}')
            return out

        e0 = list(self.enc)
        y0 = self.yaw
        fb = []

        def on_fb(msg):
            fb.append(msg.feedback.travelled_mm)
            g = self.clearance(forward)
            if g is not None and g < margin and not self.tripped:
                self.estop(f'{"forward" if forward else "rear"} clearance '
                           f'{g:.0f} mm < {margin:.0f} mm after '
                           f'{msg.feedback.travelled_mm:.0f} mm')

        gf = self.ac.send_goal_async(
            DriveDistance.Goal(distance_mm=float(dist_mm), speed_pwm=float(pwm)),
            feedback_callback=on_fb)
        dl = time.time() + 15.0
        while time.time() < dl and not gf.done():
            time.sleep(0.05)
        gh = gf.result()
        if gh is None or not gh.accepted:
            out['why'] = 'goal rejected by drive_node'
            print(f'  {out["why"]}')
            return out

        out['driven'] = True
        rf = gh.get_result_async()
        dl, last = time.time() + 180.0, 0.0
        while time.time() < dl and not rf.done():
            if fb and fb[-1] - last >= 200:
                last = fb[-1]
                g = self.clearance(forward)
                print(f'    {last:6.0f} mm   clearance '
                      f'{g:.0f} mm' if g else f'    {last:6.0f} mm')
            time.sleep(0.05)

        if rf.done():
            r = rf.result().result
            out.update(success=bool(r.success),
                       travelled_mm=float(r.travelled_mm),
                       drift_deg=float(r.heading_drift_deg),
                       message=r.message)
        else:
            self.estop('result timeout')
            out['why'] = 'result timeout'

        time.sleep(float(self.get_parameter('settle_s').value))
        if self.enc and e0:
            d = [b - a for a, b in zip(e0, self.enc)]
            mean = sum(d) / 4.0
            out['counts'] = mean
            out['spread'] = max(d) - min(d)
            out['spread_pct'] = abs(max(d) - min(d)) / max(1.0, abs(mean)) * 100.0
        out['odom'] = self.odom
        out['tripped'] = self.tripped

        print(f'  travelled {out.get("travelled_mm", float("nan")):.1f} mm   '
              f'drift {out.get("drift_deg", float("nan")):+.2f} deg')
        if 'counts' in out:
            print(f'  counts {out["counts"]:+.0f}   spread {out["spread"]} '
                  f'({out["spread_pct"]:.1f}%)')
        if self.odom:
            print(f'  odom x={self.odom[0]:+.1f} y={self.odom[1]:+.1f} mm')

        # Prove it actually stopped, rather than assuming the action's word.
        a = list(self.enc) if self.enc else None
        time.sleep(2.0)
        b = list(self.enc) if self.enc else None
        if a and b:
            dd = [y - x for x, y in zip(a, b)]
            out['stationary'] = all(v == 0 for v in dd)
            print(f'  stationary {dd} -> '
                  f'{"yes" if out["stationary"] else "NO - STILL MOVING"}')
        else:
            out['stationary'] = None
            print('  stationary: UNREADABLE')
        return out


def main(args=None):
    rclpy.init(args=args)
    n = MotionSeq()
    ev = threading.Event()

    def spin():
        while not ev.is_set():
            rclpy.spin_once(n, timeout_sec=0.05)
    th = threading.Thread(target=spin, daemon=True)
    th.start()

    f_mm = float(n.get_parameter('forward_mm').value)
    b_mm = float(n.get_parameter('back_mm').value)
    pwm = float(n.get_parameter('pwm').value)
    margin = float(n.get_parameter('margin_mm').value)

    print('=== waiting for telemetry ===')
    dl = time.time() + 15.0
    while time.time() < dl and (n.enc is None or n.fwd is None):
        time.sleep(0.1)
    if n.enc is None:
        print('  NO ENCODER DATA - refusing to drive. '
              'Is FASTRTPS_DEFAULT_PROFILES_FILE set?')
        ev.set(); th.join(timeout=2.0); n.destroy_node(); rclpy.shutdown(); return 1
    print(f'  encoders {n.enc}')
    print(f'  forward {n.fwd:.0f} mm   rear '
          f'{n.rear:.0f} mm' if n.rear else f'  forward {n.fwd:.0f} mm')

    if not n.ac.wait_for_server(timeout_sec=10.0):
        print('  /drive_distance not available - is drive_node running?')
        ev.set(); th.join(timeout=2.0); n.destroy_node(); rclpy.shutdown(); return 1

    if n.reset_cli.wait_for_service(timeout_sec=3.0):
        n.reset_cli.call_async(Empty.Request())
        time.sleep(1.0)

    results = [n.leg(+f_mm, pwm, margin), n.leg(-b_mm, pwm, margin)]

    print('\n=== SUMMARY ===')
    print(f'  {"leg":9s} {"asked":>7s} {"travelled":>10s} {"drift":>8s} '
          f'{"spread":>8s}  stopped')
    for r in results:
        if not r.get('driven'):
            print(f'  {r["leg"]:9s} {r["requested_mm"]:7.0f}  '
                  f'SKIPPED: {r.get("why", "?")}')
            continue
        print(f'  {r["leg"]:9s} {r["requested_mm"]:7.0f} '
              f'{r.get("travelled_mm", float("nan")):10.1f} '
              f'{r.get("drift_deg", float("nan")):+8.2f} '
              f'{r.get("spread_pct", float("nan")):7.1f}%  '
              f'{r.get("stationary")}')
    net = [r for r in results if r.get('driven')]
    if len(net) == 2:
        d = net[0].get('travelled_mm', 0.0) - net[1].get('travelled_mm', 0.0)
        print(f'\n  forward minus back: {d:+.1f} mm '
              f'(how far from where it started, by the encoders)')
        dr = net[0].get('drift_deg', 0.0) + net[1].get('drift_deg', 0.0)
        print(f'  net heading change: {dr:+.2f} deg')
    # Stop the spin thread and let it actually exit BEFORE destroying the
    # node. Without the join, motion_seq exited non-zero with
    # "[ros2run]: Aborted" even on a perfect run - which any automation would
    # read as a failure.
    ev.set()
    th.join(timeout=2.0)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
