#!/usr/bin/env python3
"""odom_node - where Rover 1 is, so the arena map can stay still while it moves.

WHY THIS INTEGRATES ENCODER COUNTS (changed 2026-10-09)
It used to integrate the board's own vx, because ticks-per-revolution, wheel
diameter and track width were all unmeasured. That reason is gone: a single
measured constant, COUNTS PER MILLIMETRE, replaces ticks-per-rev AND wheel
diameter for straight-line distance, and heading still comes from the fused
yaw, so track width is still not needed.

The board's vx had to go because it was measured WRONG. Commanded 0.15 m/s,
the board reported 0.15 m/s, and the rover covered ~1200 mm in 2 s - about
0.6 m/s, a 4-6x error. Integrating that number produced a pose that looked
like plausible drift and was actually a bad scale factor. Encoder counts are
the thing the wheels actually did.

counts_per_mm is a PARAMETER, not a literal, because the 6.06 default was
derived from an eyeballed "around 1200 mm". One tape-measured drive replaces
it with no code change:
    ros2 param set /odom_node counts_per_mm <value>

WHY YAW COMES FROM THE BOARD, NOT FROM INTEGRATING wz
The board publishes a FUSED attitude from its 9-axis IMU. Integrating an
angular rate accumulates error without bound; a fused yaw does not. So heading
is read, not accumulated - only x and y are integrated.

WHAT THIS IS NOT
This is dead reckoning. It drifts, it has no loop closure, and nothing corrects
it against the LiDAR. It is good for "the rover has moved about this far since
it started", not for "the rover is at this arena coordinate an hour later".
The covariances published say so rather than claiming precision.

FRAMES
    map   -> odom        static, from start_x/start_y/start_yaw. This is where
                         the rover was PLACED in arena coordinates; it is an
                         input, not something this node can discover.
    odom  -> base_link   the integrated motion.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, Vector3Stamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# Stop integrating below this. The board reads a clean zero at rest, but a
# threshold means a single noisy sample cannot walk the pose.
V_DEADBAND = 0.005          # m/s
STALE_AFTER = 1.0           # s without a velocity message = stop integrating

# Counts per millimetre. 4868.5 counts over a tape-measured 980 mm
# (2026-10-09). The previous 6.06 came from an eyeballed "around 1200 mm" and
# was 22% long. A 1244-count run called "around 200 mm" still implies 6.22, a
# 25% disagreement that is not yet settled - see drive_node for the reasoning.
# Still a parameter so the next measurement needs no edit.
DEFAULT_COUNTS_PER_MM = 4.968

# A positive pwm drives this chassis FORWARD and makes the counts FALL, so
# forward distance is the NEGATED count delta. Established by observation
# 2026-10-09 after getting the sign wrong twice.
ENC_SIGN = -1

# Per-sample sanity bound. At 24 Hz, 20000 counts is >3 m in 40 ms: that is an
# int32 wrap or a dropped frame, not motion. Such a sample is DISCARDED rather
# than integrated - a glitch must not teleport the pose.
MAX_COUNT_JUMP = 20000


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        # Where the rover was PLACED, in arena millimetres. Nothing can derive
        # this - it is where a human put the robot.
        self.declare_parameter('start_x_mm', 842.5)
        self.declare_parameter('start_y_mm', 157.5)
        self.declare_parameter('start_yaw_deg', 90.0)
        self.declare_parameter('counts_per_mm', DEFAULT_COUNTS_PER_MM)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TwistStamped, 'rover/velocity', self.on_vel, qos)
        self.create_subscription(Vector3Stamped, 'rover/attitude', self.on_att, qos)
        self.create_subscription(Int32MultiArray, 'rover/encoders', self.on_enc, qos)

        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        # Arena pose in millimetres, for the dashboard. The Cortex panel wants
        # arena mm, not metres in a TF tree it would have to walk itself.
        self.pub_arena = self.create_publisher(PoseStamped, 'rover/arena_pose', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        self.lock = threading.Lock()
        self.x = 0.0            # metres, in the odom frame
        self.y = 0.0
        self.yaw = 0.0          # radians, from the board
        self.yaw0 = None        # the board's yaw when we started
        self.have_yaw = False
        self.vx = self.vy = self.wz = 0.0
        self.last_v = 0.0
        self.last_int = None
        self.moved_m = 0.0
        # Encoder integration state.
        self.enc_prev = None        # per-wheel counts at the last sample
        self.enc_last = 0.0         # when that sample arrived
        self.enc_rejects = 0        # samples discarded by MAX_COUNT_JUMP
        self.enc_spread = 0.0       # max-min of the per-wheel deltas, counts
        self.v_enc = 0.0            # m/s from the encoders
        # The board's own vx, integrated ONLY so the 5 s log can show how far
        # it disagrees with the wheels. It drives nothing.
        self.board_m = 0.0

        # map -> odom is static: it is the placement, and it only changes on a
        # reset.
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_map_to_odom()

        self.create_service(Empty, 'reset_odom', self.on_reset)
        self.create_timer(0.05, self.tick)          # 20 Hz
        self.create_timer(5.0, self._report)
        self.get_logger().info('odom node up (dead reckoning from the board)')

    # -- inputs ------------------------------------------------------------
    def on_vel(self, m: TwistStamped):
        with self.lock:
            self.vx = m.twist.linear.x
            self.vy = m.twist.linear.y
            self.wz = m.twist.angular.z
            self.last_v = time.time()

    def on_att(self, m: Vector3Stamped):
        with self.lock:
            y = m.vector.z
            if self.yaw0 is None:
                self.yaw0 = y
            # Relative to the yaw at start, so odom begins at 0 as REP-105 wants.
            self.yaw = math.atan2(math.sin(y - self.yaw0), math.cos(y - self.yaw0))
            self.have_yaw = True

    def on_enc(self, m: Int32MultiArray):
        """Integrate wheel travel. This is what moves the pose."""
        vals = list(m.data)[:4]
        if len(vals) < 4:
            return
        now = time.time()
        with self.lock:
            prev, self.enc_prev = self.enc_prev, vals
            dt = now - self.enc_last
            self.enc_last = now
            if prev is None:
                return

            deltas = [a - b for a, b in zip(vals, prev)]
            if max(abs(d) for d in deltas) > MAX_COUNT_JUMP:
                self.enc_rejects += 1
                return
            self.enc_spread = float(max(deltas) - min(deltas))

            cpm = float(self.get_parameter('counts_per_mm').value)
            if cpm <= 0.0:
                return

            # Mean of four wheels. On a differential drive this is the right
            # scalar: a spin in place counter-rotates the sides and cancels to
            # zero translation. NOT yet verified against a measured spin.
            d_m = ENC_SIGN * (sum(deltas) / 4.0) / cpm / 1000.0
            self.v_enc = (d_m / dt) if dt > 1e-6 else 0.0
            if d_m == 0.0:
                return

            # Along the FUSED heading, not an integrated one.
            self.x += d_m * math.cos(self.yaw)
            self.y += d_m * math.sin(self.yaw)
            self.moved_m += abs(d_m)

    # -- integration -------------------------------------------------------
    def tick(self):
        """Publishing only. The encoders move the pose, in on_enc."""
        now = time.time()
        with self.lock:
            if self.last_int is None:
                self.last_int = now
                return
            dt = now - self.last_int
            self.last_int = now
            if (now - self.last_v) < STALE_AFTER and abs(self.vx) > V_DEADBAND:
                self.board_m += abs(self.vx) * dt
            x, y, yaw = self.x, self.y, self.yaw
        self._publish(x, y, yaw)

    def _publish(self, x, y, yaw):
        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quat(yaw)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.x, o.pose.pose.orientation.y, \
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
        with self.lock:
            # Speed from the wheels. The board's vx is the number that was
            # wrong by 4-6x, so it does not go on /odom.
            o.twist.twist.linear.x = self.v_enc
            o.twist.twist.linear.y = 0.0
            o.twist.twist.angular.z = self.wz
        # Honest covariances. This is dead reckoning with no correction, so the
        # position terms are LARGE and grow in spirit even though a constant is
        # published. Yaw is better because it is fused, not integrated.
        big, ok = 0.25, 0.02
        o.pose.covariance[0] = big      # x
        o.pose.covariance[7] = big      # y
        o.pose.covariance[35] = ok      # yaw
        self.pub_odom.publish(o)

        if self.tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.x, t.transform.rotation.y, \
                t.transform.rotation.z, t.transform.rotation.w = q
            self.tf.sendTransform(t)

        # Arena pose in MILLIMETRES for the dashboard.
        sx = float(self.get_parameter('start_x_mm').value)
        sy = float(self.get_parameter('start_y_mm').value)
        syaw = math.radians(float(self.get_parameter('start_yaw_deg').value))
        c, s = math.cos(syaw), math.sin(syaw)
        ax = sx + (x * c - y * s) * 1000.0
        ay = sy + (x * s + y * c) * 1000.0
        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.map_frame
        p.pose.position.x = ax
        p.pose.position.y = ay
        aq = yaw_to_quat(syaw + yaw)
        p.pose.orientation.x, p.pose.orientation.y, \
            p.pose.orientation.z, p.pose.orientation.w = aq
        self.pub_arena.publish(p)

    def _publish_map_to_odom(self):
        sx = float(self.get_parameter('start_x_mm').value) / 1000.0
        sy = float(self.get_parameter('start_y_mm').value) / 1000.0
        syaw = math.radians(float(self.get_parameter('start_yaw_deg').value))
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = sx
        t.transform.translation.y = sy
        q = yaw_to_quat(syaw)
        t.transform.rotation.x, t.transform.rotation.y, \
            t.transform.rotation.z, t.transform.rotation.w = q
        self.static_tf.sendTransform(t)

    def on_reset(self, req, resp):
        with self.lock:
            self.x = self.y = 0.0
            self.yaw0 = None       # re-zero the heading on the next attitude
            self.moved_m = 0.0
            self.board_m = 0.0
            self.enc_prev = None   # next sample re-seeds; no phantom delta
            self.enc_rejects = 0
        self.get_logger().info('odometry reset to the start pose')
        return resp

    def _report(self):
        with self.lock:
            self.get_logger().info(
                f'odom x={self.x:+.3f} y={self.y:+.3f} yaw={math.degrees(self.yaw):+.1f}deg '
                f'travelled={self.moved_m:.3f}m (wheels) vs {self.board_m:.3f}m (board vx) '
                f'spread={self.enc_spread:.0f}cts rejects={self.enc_rejects}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
