#!/usr/bin/env python3
"""odom_node - where Rover 1 is, so the arena map can stay still while it moves.

WHY THIS INTEGRATES THE BOARD'S VELOCITY AND NOT WHEEL TICKS
Differential-drive odometry from encoders needs ticks-per-revolution, wheel
diameter and track width. NONE of those are measured on Rover 1, and guessing
any of them produces a pose that is confidently wrong - the worst kind, because
it looks like drift rather than like a bad constant.

The Yahboom board already solves that internally: it reports its own vx, vy and
wz on the speed packet, computed from its own kinematics. Integrating those
needs no chassis constants at all. Measured at rest: exactly 0.0, 0.0, 0.0 -
so a stationary rover does not creep, which is the failure that makes velocity
integration untrustworthy.

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
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# Stop integrating below this. The board reads a clean zero at rest, but a
# threshold means a single noisy sample cannot walk the pose.
V_DEADBAND = 0.005          # m/s
STALE_AFTER = 1.0           # s without a velocity message = stop integrating


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

    # -- integration -------------------------------------------------------
    def tick(self):
        now = time.time()
        with self.lock:
            if self.last_int is None:
                self.last_int = now
                return
            dt = now - self.last_int
            self.last_int = now
            fresh = (now - self.last_v) < STALE_AFTER
            vx, vy, yaw = self.vx, self.vy, self.yaw
            if fresh and (abs(vx) > V_DEADBAND or abs(vy) > V_DEADBAND):
                # Body velocities into the odom frame using the FUSED yaw.
                c, s = math.cos(yaw), math.sin(yaw)
                dx = (vx * c - vy * s) * dt
                dy = (vx * s + vy * c) * dt
                self.x += dx
                self.y += dy
                self.moved_m += math.hypot(dx, dy)
            x, y = self.x, self.y
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
            o.twist.twist.linear.x = self.vx
            o.twist.twist.linear.y = self.vy
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
        self.get_logger().info('odometry reset to the start pose')
        return resp

    def _report(self):
        with self.lock:
            self.get_logger().info(
                f'odom x={self.x:+.3f} y={self.y:+.3f} yaw={math.degrees(self.yaw):+.1f}deg '
                f'travelled={self.moved_m:.2f}m')


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
