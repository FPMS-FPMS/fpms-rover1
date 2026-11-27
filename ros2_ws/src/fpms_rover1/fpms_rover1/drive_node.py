#!/usr/bin/env python3
"""drive_node - closed-loop motion for Rover 1. THIS NODE MOVES THE ROBOT.

EVERY CONSTANT BELOW WAS MEASURED ON 2026-10-09, NOT ASSUMED
  counts/mm      36.3    from a 0.15 m/s run: 10882 counts over ~300 mm
  direction      a POSITIVE motor command gives NEGATIVE encoder counts,
                 consistently on all four wheels and on both command paths
  stiction       PWM 10 never moves; PWM 12 sometimes does; PWM 11 sometimes
                 does. Below ~15 whether it moves at all is a coin flip.
  motion deadband  set_car_motion did NOTHING from 10 to 80 mm/s, then leapt
                 to 5428 counts/s at 150 mm/s. That path is unusable for slow
                 work, so this node drives raw PWM and closes the loop itself.

WHY CLOSED LOOP, CONCRETELY
Open loop at the stiction edge gave 1.69, 3.77 and 7.33 mm for the SAME
command - a 132% spread. Not a speed, a lottery. Reading the encoders and
stopping on distance turns that into a number you can trust, at the cost of
running the motors a little faster than their absolute floor.

TURNS USE THE IMU, NOT THE WHEELS
Wheels slip when turning on the spot, so encoder-derived heading drifts. The
board publishes a FUSED yaw; that is the authority for angle.
"""

from __future__ import annotations

import math
import struct
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from fpms_rover1_msgs.action import DriveDistance, Turn

# The LiDAR rotation ruler, reused rather than reimplemented. It is validated:
# parked it reads mean +0.064 deg with 0.370 spread and a 7.0 mm median
# residual over 420 ray pairs - which independently matches the team's own
# 6.2 mm on a separate implementation. It shares NO hardware with the IMU,
# which is the entire point: the board's fused yaw under-reads physical
# rotation (~1.60x on the sister board) and is sign-inverted, so every loop
# closed on it has failed. See yaw_ruler.py and
# /usr/share/fpms/knowledge/fpms-phase6-spin-must-use-fused-yaw.md
from fpms_rover1.yaw_ruler import GRID_DEG, best_shift, resample
from fpms_rover1 import geometry as G
from fpms_rover1.curvature import Slew

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = 115200
HEAD, DEVICE_ID = 0xFF, 0xFC
COMPLEMENT = 257 - DEVICE_ID
FUNC_MOTOR, FUNC_CARTYPE = 0x10, 0x15
CAR_TYPE = 0x02

# MEASURED AGAINST A REAL DISTANCE, 2026-10-09.
# The previous 36.3 was derived from the board's vx command - 0.15 m/s for
# 2 s was ASSUMED to be 300 mm. It was not: vx is not a calibrated speed, and
# that run actually covered ~1795 mm. The error was 6x, and it reported a
# 1200 mm drive as "200.4 mm" with a confident 0.2% error figure. A wrong
# constant and a tight tolerance together produce precise nonsense.
COUNTS_PER_MM = 4.968
# CALIBRATION, 2026-10-09. 4868.5 counts over a tape-measured 980 mm.
# This REPLACED 6.06, which came from an eyeballed "around 1200 mm" and ran
# 22% long: a commanded 800 mm actually travelled 980 mm.
#
# UNRESOLVED: an earlier 1244-count run was reported as "around 200 mm",
# which implies 6.22 counts/mm, not 4.97 - a 25% disagreement. 4.968 is
# adopted because it comes from the longer run (4x the distance, so a given
# tape error matters 4x less) and from a stated rather than approximate
# number. If 6.22 were the true value, a commanded 800 mm would travel
# ~1000 mm; that is exactly what the next reverse run tests.
#
# It is also a PARAMETER, so correcting it needs no rebuild:
#     ros2 param set /drive_node counts_per_mm <value>

# DIRECTION, OBSERVED 2026-10-09.
#
# The operator watched it: a cmd_vel of -0.222 drove the rover FORWARD.
# That path produced a POSITIVE pwm (the old code negated it), so:
#
#       POSITIVE pwm  ->  FORWARD        NEGATIVE pwm  ->  BACKWARD
#
# Hence FWD_PWM_SIGN = +1, and +linear.x now genuinely means forward.
#
# This is the OPPOSITE of Rover 2, whose code records "set_motor NEGATIVE
# values drive forward on this chassis". Assuming the two rovers shared a
# convention would have been wrong, which is why it was worth measuring.
#
# NOTE a positive pwm makes encoder counts FALL, so encoder deltas are
# negative when driving forward. Direction and count sign are independent
# facts and both are needed.
FWD_PWM_SIGN = 1
ENC_SIGN = -1               # a positive pwm also makes encoder counts fall

# Real speeds, now that the scale is known:
#   PWM 18 ~ 154 mm/s      PWM 26 ~ 222 mm/s      PWM 12 ~ under 30 mm/s
# These are far faster than the earlier estimates implied.
MAX_PWM = 35                # hard cap, not a parameter
MIN_PWM = 12                # below this it may not move at all
DEFAULT_PWM = 14            # ~100 mm/s. Was 18; lowered now the scale is real.
APPROACH_PWM = 12           # final crawl into the goal

CMD_VEL_TIMEOUT = 0.5       # no Twist for this long -> stop
ENC_TIMEOUT = 0.8           # no encoders -> we are driving blind -> stop
CTRL_DT = 0.05


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.declare_parameter('port', PORT)
        self.declare_parameter('counts_per_mm', COUNTS_PER_MM)
        self.declare_parameter('default_pwm', DEFAULT_PWM)
        self.declare_parameter('dist_tol_mm', 3.0)
        self.declare_parameter('yaw_tol_deg', 1.5)
        # Heading hold during a straight drive. DEFAULT OFF - it was
        # measured against a controlled baseline at 400 mm each way and does
        # not work:
        #
        #   config          fwd drift   back drift   fwd spread  back spread
        #   OFF (baseline)    -4.74       +6.31         6.6%        8.9%
        #   +1, kp 70         -4.50       +8.38         2.6%       13.8%
        #   -1, kp 70         -7.80       +5.64        10.0%        6.3%
        #   -1, kp 15         -5.48       +6.49         6.8%        8.5%
        #
        # kp 15 is indistinguishable from off; kp 70 only adds noise. Neither
        # sign reduces the error, so the loop has no useful authority.
        #
        # The likely reason is that the signal it closes on is not trustworthy:
        # the board's gyro and magnetometer scales were never verified (they
        # cannot be derived at rest, and the accelerometer already turned out
        # to follow NO datasheet full-scale). A proportional loop on a
        # mis-scaled yaw cannot converge. Validate yaw against a KNOWN
        # rotation - a protractor-measured 90 deg turn - before re-enabling.
        #
        # The code is kept, off, because it is the right shape once the yaw
        # source is trusted. Turn on with:
        #     ros2 param set /drive_node yaw_hold true
        self.declare_parameter('yaw_hold', False)

        # HEADING HOLD ON THE LIDAR. This is the one that can work, because the
        # error signal is measured rather than inferred.
        #   lidar_hold      close the loop on the scan match while driving
        #   lidar_kp        pwm counts of boost per degree of error
        #   lidar_max_boost hard cap on that boost
        #   lidar_corr_sign which side to boost; TO BE MEASURED, one short run
        #                   settles it now that the error signal is trustworthy
        #   max_drift_deg   ABORT the leg beyond this. "Realise mid-movement
        #                   that it is drifting and stop" - this is that.
        #   lidar_window    search +-this many grid bins (1 bin = 0.703 deg).
        #                   40 bins = +-28 deg, wide enough for any real drift
        #                   and narrow enough to run in the control loop.
        # DEFAULT OFF, and it must stay off until a translation-aware
        # matcher replaces the 1-DOF one. MEASURED 2026-10-09, latching one
        # reference and driving 400 mm out and back:
        #
        #   at start (null)   angle  -0.11 deg   residual    7.0 mm
        #   at the far end    angle  -5.04 deg   residual  217.5 mm
        #   back at start     angle  -1.27 deg   residual    8.0 mm
        #
        # A 27x residual jump that vanishes on returning to the same spot.
        # The mid-leg angle is NOT rotation - it is the 400 mm displacement
        # aliasing into apparent rotation, because correlating for rotation
        # alone cannot separate the two. The real net rotation over the whole
        # out-and-back is -1.27 deg with an 8 mm residual.
        #
        # So this ruler is only valid where the rover is at (near) the
        # reference position: turn sequences that return to their start, and
        # parked checks. Steering a straight leg with it means steering on an
        # artifact. Mid-leg heading needs full 2D scan matching (ICP/GICP -
        # rf2o_laser_odometry, simple-2d-lidar-odometry) or the team's existing
        # scan-to-MAP localiser, which solve for translation AND rotation.
        self.declare_parameter('lidar_hold', False)
        self.declare_parameter('lidar_kp', 0.8)
        self.declare_parameter('lidar_max_boost', 6.0)
        self.declare_parameter('lidar_corr_sign', 1)
        self.declare_parameter('max_drift_deg', 12.0)
        self.declare_parameter('lidar_window', 40)
        # Residual above this means the scans no longer describe the same
        # viewpoint, so any angle from them is an artifact. Parked measures
        # 7-8 mm; displaced 400 mm measures 218 mm. 40 mm is comfortably above
        # the noise and far below the contaminated regime.
        self.declare_parameter('lidar_max_residual_mm', 40.0)

        # HEADING HOLD ON GICP LASER ODOMETRY. This is the one with a signal
        # that survives translation, which is what every earlier attempt
        # lacked: the IMU is mis-scaled, and a 1-DOF scan correlation reads
        # displacement as rotation.
        # MEASURED A/B at 800 mm each way, all three measured by GICP:
        #   hold off          fwd +1.23  back -1.70   mean |drift| 1.47 deg
        #   hold on, sign +1  fwd -0.66  back -0.67   mean |drift| 0.67 deg
        #   hold on, sign -1  fwd +0.15  back -2.87   mean |drift| 1.51 deg
        # +1 halves the drift AND makes it consistent in both directions;
        # -1 is no better than off. Closed-loop, so it will not decay as the
        # battery sags the way a fixed duty bias would.
        self.declare_parameter('gicp_hold', True)
        self.declare_parameter('gicp_kp', 1.5)        # pwm counts per degree
        self.declare_parameter('gicp_max_boost', 6.0)
        self.declare_parameter('gicp_corr_sign', 1)
        self.declare_parameter('gicp_stale_s', 1.0)

        # TURNS. 'gicp' closes the turn on laser odometry; 'imu' closes it on
        # the board's fused yaw, which is what the code did before and is
        # NOT SAFE: that yaw is mis-scaled (~1.60x under-read on the sister
        # board) and sign-inverted, and the team's own note records the cost -
        # "the turn loop closed on raw yaw, so the rover turned the wrong way
        # and ~3x too far, and the operator reported it got lost".
        self.declare_parameter('turn_source', 'gicp')
        # The turn closes on the MAGNITUDE rotated, and reports the SIGNED
        # rotation, so the mapping from a commanded + angle to a physical
        # direction is measured rather than assumed. Once known, set this.
        self.declare_parameter('turn_cmd_sign', 0)   # 0 = unknown/unenforced
        # TURNS NEED THEIR OWN DUTY BUDGET. MAX_PWM 35 is ample for straight
        # driving (~222 mm/s) but a 4-wheel skid-steer has to scrub all four
        # tyres sideways to rotate, which is far harder. MEASURED 2026-10-09:
        #   pwm 16 -> +0.10 deg in 30 s   (dead)
        #   pwm 32 -> +3.41 deg in 30 s   (effectively stalled)
        # The board itself accepts +-100, so 35 was our own cap, not a limit.
        # The team saw the same on the sister chassis: pwm 18 was "below the
        # motor deadzone" and they used 30-40, noting higher power also turns
        # MORE SYMMETRICALLY (encoder ratio 2.09 at pw30, 1.28 at pw40).
        self.declare_parameter('max_turn_pwm', 60)

        # ARC TURNS. In-place rotation does NOT work on this chassis: at FULL
        # duty the wheels creep at ~1 mm/s against four-tyre scrub (vs 75 mm/s
        # straight at pwm 14), flat from pwm 50 to 100, encoders counting in
        # correct opposition the whole time. A torque stall, not slip.
        #
        # Rolling resistance is a fraction of static scrub, so an arc turns
        # where a spin cannot. Both sides drive FORWARD at different duty.
        #
        # The RADIUS IS DELIBERATELY NOT MODELLED. Duty-to-radius on a
        # skid-steer is nonlinear and surface-dependent, and Rover 2 settled
        # this already: its turn arc is measured (148 mm) and explicitly not
        # modelled. So this closes the loop on MEASURED rotation and reports
        # the radius it actually achieved.
        #
        # inner stays at or above MIN_PWM: an inner wheel below the stiction
        # floor stops driving and starts dragging, which is scrub again.
        # 'scurve' runs BOTH halves inside ONE goal: curve one way to the
        # peak angle, then back to the original heading, with the duty split
        # SLEWED through the reversal. Two separate Turn goals would work
        # geometrically and be wrong in the way that matters - the stop between
        # them is what makes motion rigid. One goal, one continuous motion.
        self.declare_parameter('turn_mode', 'arc')     # 'arc'|'spin'|'scurve'
        # How fast the duty split may reverse, in pwm counts per second. This
        # is the fluidity knob: instant reversal is two rigid turns back to
        # back; a ramp through zero is a single S.
        self.declare_parameter('scurve_slew_pwm_s', 40.0)
        self.declare_parameter('arc_outer_pwm', 26)
        # SIGNED. Negative drives the inner side BACKWARD, which is the only
        # way to get a usable radius on this chassis.
        #
        # MEASURED 2026-10-09: outer 26 / inner +13 spent ~530 mm of forward
        # clearance to rotate 7.2 deg - an arc radius of roughly 4.2 m, almost
        # a straight line. A 90 deg turn at that radius needs ~6 m of room.
        # The operator watched it run forward and lean left and called it, and
        # they were right: a 2:1 forward split barely curves this chassis,
        # because skid-steer scrub resists yaw far harder than it resists
        # rolling.
        #
        # Negative inner gives a continuum: wide arc -> pivot (inner ~0) ->
        # spin (inner = -outer, which is the stall that does not work). The
        # useful setting is near the pivot end, where the wheels still ROLL
        # (so no stall) but the radius is small.
        #
        # Negative values deliberately bypass the MIN_PWM floor check for the
        # inner side: the floor exists so a driven wheel is not left creeping,
        # and a reversed inner wheel is driven, just the other way.
        self.declare_parameter('arc_inner_pwm', -10)
        # An arc turn TRANSLATES while it rotates - it sweeps forward and
        # sideways - so it needs room ahead, unlike a spin.
        # Derived from MEASURED geometry, not guessed. An arc turn sweeps
        # the furthest corner of the body about the scanner
        # (TURN_SWEEP_RADIUS ~208 mm for 170x120), plus a stopping pad. The
        # old flat 450 was over-strict: it aborted an arc at 216 mm ahead when
        # the body front is only 80 mm past the scanner.
        self.declare_parameter('arc_min_clear_mm',
                               round(G.TURN_SWEEP_RADIUS + 120.0, 1))
        # If the radius it is actually achieving exceeds this, the turn is a
        # near-straight line and will run out of room. Stop and say so rather
        # than drive on. 26/+13 measured ~4200 mm; a usable turn is a few
        # hundred.
        self.declare_parameter('arc_max_radius_mm', 900.0)
        self.declare_parameter('yaw_kp', 15.0)        # pwm counts per radian
        self.declare_parameter('yaw_max_boost', 3.0)  # pwm counts, hard cap
        # -1, MEASURED. I first reasoned my way to +1 from motors(l,r) packing
        # [l,l,r,r] and so assumed encoders 0,1 were the left side. Tested at
        # 400 mm each way, +1 made drift WORSE in BOTH directions - forward
        # -0.0032 -> -0.0113 deg/mm, reverse +0.0107 -> +0.0210 deg/mm, and
        # reverse wheel spread 8.2% -> 13.8%. The correction was feeding the
        # drift, so the index-to-side mapping (or the board's yaw sign) is the
        # opposite of what I assumed. Which of the two does not matter here:
        # the loop only needs the sign that shrinks the error, and that is -1.
        self.declare_parameter('yaw_corr_sign', -1)
        self.declare_parameter('max_seconds', 30.0)
        # Max |linear.x| in m/s that /cmd_vel maps to full PWM. Only used for
        # manual teleop; the actions do not go through it.
        # Calibrated: PWM 26 measured at 222 mm/s, so full PWM 35 ~ 0.30 m/s.
        # linear.x in a Twist is therefore roughly real metres per second.
        self.declare_parameter('cmd_vel_full_scale', 0.30)

        self.cpmm = float(self.get_parameter('counts_per_mm').value)
        self.default_pwm = int(clamp(int(self.get_parameter('default_pwm').value), MIN_PWM, MAX_PWM))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int32MultiArray, 'rover/encoders', self.on_enc, qos)
        self.create_subscription(Vector3Stamped, 'rover/attitude', self.on_att, qos)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos)
        # GICP laser odometry (simple-2d-lidar-odometry, package lidar_odometry).
        # Its TRANSLATION is scaled ~0.42 and is deliberately NOT used - distance
        # comes from calibrated encoders. Only its YAW is used, and that is
        # validated two independent ways: 0.01 deg/s phantom rotation while
        # parked (at max_correspondence_distance 0.1; the shipped 1.0 gave
        # 0.11 deg/s and 105 mm of phantom travel in 15 s), and a net -1.36 deg
        # over an out-and-back that matches the LiDAR ruler's independent
        # -1.27 deg to within 0.09 deg.
        self.create_subscription(Odometry, 'scan_odom', self.on_scan_odom, qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.lock = threading.Lock()
        self.enc = None
        self.enc_at = 0.0
        self.yaw = None
        self.scan_grid = None       # resampled fixed grid, newest scan
        self.scan_at = 0.0
        self.scan_n = 0             # bumped per scan so the loop sees new data
        self.gicp_yaw = None        # degrees, from laser odometry
        self.gicp_at = 0.0
        self.fwd_clear = None       # mm, 10th pct over a +-60 deg cone
        self.cmd_vel = None
        self.cmd_vel_at = 0.0
        self.busy = False           # an action owns the motors
        self.ser = None
        self._open()

        cb = ReentrantCallbackGroup()
        self.srv_stop = self.create_service(Trigger, 'drive/stop', self.on_stop, callback_group=cb)
        self.act_drive = ActionServer(
            self, DriveDistance, 'drive_distance',
            execute_callback=self.exec_drive, callback_group=cb,
            goal_callback=self.accept, cancel_callback=self.accept_cancel)
        self.act_turn = ActionServer(
            self, Turn, 'turn',
            execute_callback=self.exec_turn, callback_group=cb,
            goal_callback=self.accept, cancel_callback=self.accept_cancel)

        self.create_timer(CTRL_DT, self.cmd_vel_tick, callback_group=cb)
        self.get_logger().warn(
            f'drive_node up - CAN MOVE THE ROBOT. pwm cap {MAX_PWM}, '
            f'default {self.default_pwm}, {self.cpmm:.1f} counts/mm')

    # -- board -------------------------------------------------------------
    def _open(self):
        try:
            import serial
            self.ser = serial.Serial(self.get_parameter('port').value, BAUD, timeout=0.2)
            self._frame(FUNC_CARTYPE, [CAR_TYPE, 0x5F])
            self.get_logger().info('board open')
        except Exception as exc:                          # noqa: BLE001
            self.ser = None
            self.get_logger().error(f'cannot open board: {exc}')

    def _frame(self, func, payload):
        if self.ser is None:
            return
        c = [HEAD, DEVICE_ID, 0x00, func] + list(payload)
        c[2] = len(c) - 1
        c.append(sum(c, COMPLEMENT) & 0xFF)
        try:
            self.ser.write(bytes(c))
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().error(f'write failed: {exc}')

    def motors(self, left, right):
        """Motors 1,2 = left pair; 3,4 = right pair."""
        l = int(clamp(left, -MAX_PWM, MAX_PWM))
        r = int(clamp(right, -MAX_PWM, MAX_PWM))
        self._frame(FUNC_MOTOR, [struct.pack('b', v)[0] for v in (l, l, r, r)])

    def halt(self):
        for _ in range(3):
            self.motors(0, 0)

    # -- inputs -------------------------------------------------------------
    def on_enc(self, m):
        with self.lock:
            self.enc = list(m.data)
            self.enc_at = time.time()

    def on_att(self, m):
        with self.lock:
            self.yaw = m.vector.z

    def on_cmd_vel(self, m):
        with self.lock:
            self.cmd_vel = (m.linear.x, m.angular.z)
            self.cmd_vel_at = time.time()

    def on_scan_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        with self.lock:
            self.gicp_yaw = math.degrees(
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            self.gicp_at = time.time()

    def on_scan(self, m: LaserScan):
        """Resample here, not in the control loop.

        Resampling 500 rays is ~500 cheap operations; doing it once per scan at
        10 Hz keeps it off the 20 Hz control path entirely.
        """
        g = resample(m)
        # Wide forward cone, because an arc turn sweeps sideways as well as
        # ahead. 10th percentile, not the minimum: one spurious short ray must
        # not abort a good turn, but a real surface fills many rays at once.
        fwd = []
        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min < r < m.range_max):
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            a = (a + 180.0) % 360.0 - 180.0
            if abs(a) <= 60.0:
                fwd.append(r * 1000.0)
        fwd.sort()
        with self.lock:
            self.scan_grid = g
            self.scan_at = time.time()
            self.scan_n += 1
            if len(fwd) >= 5:
                self.fwd_clear = fwd[len(fwd) // 10]

    def lidar_drift(self, ref, window):
        """Degrees rotated since `ref`, or None if it cannot be measured.

        Positive/negative follows the ruler's convention, which matches a
        commanded angle. Returns None rather than 0.0 when the match is weak -
        a drift of "zero" and a drift of "unknown" must never look the same to
        a control loop.
        """
        with self.lock:
            g = self.scan_grid
        if ref is None or g is None:
            return None, None
        shift, resid, pairs = best_shift(ref, g, center=0, radius=window)
        if pairs < 60:
            return None, None
        # FAIL CLOSED on residual. Without this the loop steers on a number
        # that looks like 5 degrees of drift and is actually displacement.
        lim = float(self.get_parameter('lidar_max_residual_mm').value) / 1000.0
        if resid > lim:
            return None, resid
        deg = (shift * GRID_DEG + 180.0) % 360.0 - 180.0
        return deg, resid

    def mean_counts(self):
        with self.lock:
            return (sum(self.enc) / len(self.enc)) if self.enc else None, self.enc_at

    # -- /cmd_vel (manual teleop) -------------------------------------------
    def cmd_vel_tick(self):
        with self.lock:
            if self.busy:
                return
            cv, at = self.cmd_vel, self.cmd_vel_at
        if cv is None:
            return
        if time.time() - at > CMD_VEL_TIMEOUT:
            # A teleop that stops publishing must stop the robot, not leave it
            # running on the last Twist it happened to send.
            self.halt()
            with self.lock:
                self.cmd_vel = None
            return
        v, w = cv
        full = float(self.get_parameter('cmd_vel_full_scale').value) or 0.25
        base = (v / full) * MAX_PWM
        diff = (w / full) * MAX_PWM * 0.5
        # REP-103: +linear.x is FORWARD. FWD_PWM_SIGN converts that into the
        # pwm polarity this chassis actually needs.
        self.motors(FWD_PWM_SIGN * (base - diff), FWD_PWM_SIGN * (base + diff))

    # -- actions -------------------------------------------------------------
    def accept(self, goal):
        with self.lock:
            if self.busy:
                self.get_logger().warn('rejecting goal: another motion is running')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def accept_cancel(self, goal):
        return CancelResponse.ACCEPT

    def _guard(self):
        """Returns a reason to stop, or None."""
        _, at = self.mean_counts()
        if at == 0.0 or (time.time() - at) > ENC_TIMEOUT:
            return 'no encoder data - refusing to drive blind'
        return None

    def exec_drive(self, goal_handle):
        req = goal_handle.request
        res = DriveDistance.Result()
        with self.lock:
            self.busy = True
        try:
            reason = self._guard()
            if reason:
                goal_handle.abort(); res.success = False; res.message = reason
                return res
            pwm = int(clamp(int(req.speed_pwm) or self.default_pwm, MIN_PWM, MAX_PWM))
            target = abs(float(req.distance_mm)) * self.cpmm
            direction = 1.0 if req.distance_mm >= 0 else -1.0
            start, _ = self.mean_counts()
            with self.lock:
                yaw0 = self.yaw
            tol = float(self.get_parameter('dist_tol_mm').value) * self.cpmm
            deadline = time.time() + float(self.get_parameter('max_seconds').value)
            yaw_hold = bool(self.get_parameter('yaw_hold').value)
            yaw_kp = float(self.get_parameter('yaw_kp').value)
            yaw_max_boost = float(self.get_parameter('yaw_max_boost').value)
            yaw_sign = float(self.get_parameter('yaw_corr_sign').value)
            if yaw_hold and yaw0 is None:
                self.get_logger().warn('no attitude yet - driving WITHOUT '
                                       'heading hold')

            lidar_hold = bool(self.get_parameter('lidar_hold').value)
            lidar_kp = float(self.get_parameter('lidar_kp').value)
            lidar_max = float(self.get_parameter('lidar_max_boost').value)
            lidar_sign = float(self.get_parameter('lidar_corr_sign').value)
            max_drift = float(self.get_parameter('max_drift_deg').value)
            window = int(self.get_parameter('lidar_window').value)

            # Latch the reference scan the leg will be measured against. If
            # lidar_hold is on we REFUSE to start without one: the whole point
            # is that this leg is steered, and silently driving it open-loop
            # would be the dangerous kind of fallback.
            with self.lock:
                ref_grid = self.scan_grid
                scan_seen = self.scan_n
            if lidar_hold and ref_grid is None:
                self.halt(); goal_handle.abort()
                res.success = False
                res.message = ('no /scan - refusing to drive with heading hold '
                               'on and nothing to measure against')
                return res
            drift_deg, drift_resid = 0.0, None

            gicp_hold = bool(self.get_parameter('gicp_hold').value)
            gicp_kp = float(self.get_parameter('gicp_kp').value)
            gicp_max = float(self.get_parameter('gicp_max_boost').value)
            gicp_sign = float(self.get_parameter('gicp_corr_sign').value)
            gicp_stale = float(self.get_parameter('gicp_stale_s').value)
            with self.lock:
                gyaw0, gat = self.gicp_yaw, self.gicp_at
            gicp_live = (gyaw0 is not None
                         and (time.time() - gat) <= gicp_stale)
            if gicp_hold and not gicp_live:
                self.halt(); goal_handle.abort()
                res.success = False
                res.message = ('no /scan_odom - refusing to drive steered with '
                               'no heading source. Is lidar_odometry running?')
                return res
            gicp_err = 0.0
            fb = DriveDistance.Feedback()
            last_t, last_c = time.time(), start

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.halt(); goal_handle.canceled()
                    res.success = False; res.message = 'cancelled'
                    res.travelled_mm = abs(self.mean_counts()[0] - start) / self.cpmm
                    return res
                reason = self._guard()
                if reason or time.time() > deadline:
                    self.halt(); goal_handle.abort()
                    res.success = False
                    res.message = reason or 'deadline - goal not reached'
                    res.travelled_mm = abs(self.mean_counts()[0] - start) / self.cpmm
                    return res

                cur, _ = self.mean_counts()
                moved = abs(cur - start)
                remain = target - moved
                if remain <= tol:
                    break
                # Crawl the last 10 mm so it does not overshoot the tolerance.
                p = pwm if remain > 10 * self.cpmm else APPROACH_PWM

                # HEADING HOLD, added 2026-10-09.
                # Measured: an uncorrected straight run drifts -2.56 deg going
                # forward and +8.75 deg in reverse. Same cause - one side is
                # persistently slower (encoders 0,1 trailed 2,3 by 3% forward
                # and 6% reverse) - and the yaw sign flips with direction
                # because reversing the wheels reverses the rotation.
                #
                # An open-loop duty bias would need a different constant per
                # direction and would still drift as the battery sags. The
                # board publishes a FUSED yaw, so this closes the loop on it
                # and needs no measured constant at all.
                #
                # The correction only ever ADDS duty to the lagging side, never
                # subtracts from the leading one: MIN_PWM is a stiction floor
                # (10 does not move the wheels, 12 does), so trimming the fast
                # side down risks stalling it and turning a drift into a spin.
                boost_l = boost_r = 0.0
                with self.lock:
                    ynow = self.yaw
                if yaw_hold and ynow is not None and yaw0 is not None:
                    err = math.atan2(math.sin(ynow - yaw0),
                                     math.cos(ynow - yaw0))
                    mag = clamp(abs(err) * yaw_kp, 0.0, yaw_max_boost)
                    if err * yaw_sign < 0.0:
                        boost_l = mag
                    elif err * yaw_sign > 0.0:
                        boost_r = mag

                # HEADING HOLD ON THE LIDAR, added 2026-10-09.
                # The IMU version above is off because its signal is
                # mis-scaled. This one measures the error directly against a
                # latched scan, so it is the loop that can actually converge.
                if lidar_hold:
                    with self.lock:
                        n_now, at = self.scan_n, self.scan_at
                    if (time.time() - at) > 1.0:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = ('scan went stale mid-leg - stopped '
                                       'rather than drive unsteered')
                        res.travelled_mm = moved / self.cpmm
                        return res
                    if n_now != scan_seen:
                        scan_seen = n_now
                        d, r = self.lidar_drift(ref_grid, window)
                        if d is not None:
                            drift_deg, drift_resid = d, r

                    # STOP ITSELF DRIFTING. Not a warning, not a log line - the
                    # goal aborts and the wheels stop.
                    if abs(drift_deg) > max_drift:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = (f'DRIFT ABORT: {drift_deg:+.2f} deg off '
                                       f'course (limit {max_drift:.1f}) after '
                                       f'{moved / self.cpmm:.0f} mm')
                        res.travelled_mm = moved / self.cpmm
                        res.heading_drift_deg = drift_deg
                        self.get_logger().warn(res.message)
                        return res

                    mag = clamp(abs(drift_deg) * lidar_kp, 0.0, lidar_max)
                    if drift_deg * lidar_sign < 0.0:
                        boost_l = max(boost_l, mag)
                    elif drift_deg * lidar_sign > 0.0:
                        boost_r = max(boost_r, mag)

                # HEADING HOLD ON GICP. Unlike the two attempts above, this
                # error signal is measured and survives translation.
                if gicp_live:
                    with self.lock:
                        gy, gat = self.gicp_yaw, self.gicp_at
                    if gicp_hold and (time.time() - gat) > gicp_stale:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = ('scan_odom went stale mid-leg - stopped '
                                       'rather than drive unsteered')
                        res.travelled_mm = moved / self.cpmm
                        return res
                    gicp_err = (gy - gyaw0 + 180.0) % 360.0 - 180.0

                    # STOP ITSELF DRIFTING: abort, halt the wheels, say why.
                    if gicp_hold and abs(gicp_err) > max_drift:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = (f'DRIFT ABORT: {gicp_err:+.2f} deg off '
                                       f'course by laser odometry (limit '
                                       f'{max_drift:.1f}) after '
                                       f'{moved / self.cpmm:.0f} mm')
                        res.travelled_mm = moved / self.cpmm
                        res.heading_drift_deg = gicp_err
                        self.get_logger().warn(res.message)
                        return res

                    if gicp_hold:
                        mag = clamp(abs(gicp_err) * gicp_kp, 0.0, gicp_max)
                        if gicp_err * gicp_sign < 0.0:
                            boost_l = max(boost_l, mag)
                        elif gicp_err * gicp_sign > 0.0:
                            boost_r = max(boost_r, mag)

                dl = FWD_PWM_SIGN * direction * clamp(p + boost_l, MIN_PWM, MAX_PWM)
                dr = FWD_PWM_SIGN * direction * clamp(p + boost_r, MIN_PWM, MAX_PWM)
                self.motors(dl, dr)

                now = time.time()
                if now - last_t >= 0.2:
                    fb.travelled_mm = moved / self.cpmm
                    fb.remaining_mm = max(0.0, remain / self.cpmm)
                    fb.speed_mms = abs(cur - last_c) / self.cpmm / (now - last_t)
                    goal_handle.publish_feedback(fb)
                    last_t, last_c = now, cur
                time.sleep(CTRL_DT)

            self.halt()
            time.sleep(0.35)                     # let it settle before measuring
            final, _ = self.mean_counts()
            with self.lock:
                yaw1 = self.yaw
            res.travelled_mm = abs(final - start) / self.cpmm

            # heading_drift_deg carries the LIDAR measurement when there is
            # one, because the IMU number is mis-scaled and reporting it as
            # "the drift" has misled this project repeatedly. The IMU value is
            # kept in the message so the two can be compared.
            imu_deg = None
            if yaw0 is not None and yaw1 is not None:
                imu_deg = math.degrees(
                    math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
            final_d, final_r = self.lidar_drift(ref_grid, window)
            if final_d is not None:
                res.heading_drift_deg = final_d
            elif imu_deg is not None:
                # The LiDAR could not measure it from here. Report the IMU
                # number but the message says which source it is, because
                # silently swapping sources is how a mis-scaled yaw gets
                # mistaken for ground truth.
                res.heading_drift_deg = imu_deg
            res.success = True
            res.message = f'travelled {res.travelled_mm:.1f} mm of {abs(req.distance_mm):.1f} mm'
            if final_d is not None:
                res.message += (f'; lidar drift {final_d:+.2f} deg '
                                f'(residual {final_r * 1000:.0f} mm)')
            elif final_r is not None:
                res.message += (f'; lidar UNMEASURABLE from here '
                                f'(residual {final_r * 1000:.0f} mm - the '
                                f'rover moved, so the angle would be an '
                                f'artifact)')
            if imu_deg is not None:
                res.message += f'; imu says {imu_deg:+.2f} deg'
            if gicp_live:
                with self.lock:
                    gy = self.gicp_yaw
                if gy is not None:
                    g = (gy - gyaw0 + 180.0) % 360.0 - 180.0
                    res.message += (f'; GICP drift {g:+.2f} deg'
                                    f'{"" if gicp_hold else " (measured, NOT corrected)"}')
                    # GICP yaw is the trustworthy mid-leg number, so it carries
                    # heading_drift_deg whenever it is available - corrected or
                    # not. That is what makes a baseline comparable.
                    res.heading_drift_deg = g
            goal_handle.succeed()
            self.get_logger().info(res.message)
            return res
        finally:
            self.halt()
            with self.lock:
                self.busy = False

    def exec_turn(self, goal_handle):
        req = goal_handle.request
        res = Turn.Result()
        with self.lock:
            self.busy = True
        try:
            src = str(self.get_parameter('turn_source').value).lower()
            stale = float(self.get_parameter('gicp_stale_s').value)
            with self.lock:
                yaw0 = self.yaw                       # board, for comparison
                g0, gat = self.gicp_yaw, self.gicp_at

            if src == 'gicp':
                if g0 is None or (time.time() - gat) > stale:
                    goal_handle.abort(); res.success = False
                    res.message = ('no /scan_odom - refusing to turn on the '
                                   'board yaw, which is mis-scaled and '
                                   'sign-inverted. Start lidar_odometry.')
                    return res
                # GICP publishes degrees of yaw; work in radians like the rest.
                ref0 = math.radians(g0)
            else:
                if yaw0 is None:
                    goal_handle.abort(); res.success = False
                    res.message = 'no attitude data - refusing to turn blind'
                    return res
                self.get_logger().warn('turning on the BOARD YAW - mis-scaled '
                                       'and sign-inverted; angle will be wrong')
                ref0 = yaw0

            turn_cap = int(self.get_parameter('max_turn_pwm').value)
            pwm = int(clamp(int(req.speed_pwm) or self.default_pwm,
                            MIN_PWM, turn_cap))
            mode = str(self.get_parameter('turn_mode').value).lower()
            arc_out = int(clamp(int(self.get_parameter('arc_outer_pwm').value),
                                MIN_PWM, turn_cap))
            # Signed: clamp magnitude, keep direction. A positive inner below
            # MIN_PWM would creep rather than drive, so it is floored; a
            # negative inner is a driven wheel and only needs the cap.
            _ai = int(self.get_parameter('arc_inner_pwm').value)
            if _ai >= 0:
                arc_in = int(clamp(_ai, MIN_PWM, turn_cap))
            else:
                arc_in = -int(clamp(-_ai, MIN_PWM, turn_cap))
            need_clear = float(self.get_parameter('arc_min_clear_mm').value)
            if mode == 'arc':
                with self.lock:
                    fc = self.fwd_clear
                if fc is None:
                    goal_handle.abort(); res.success = False
                    res.message = ('no scan - refusing an arc turn without '
                                   'knowing what is ahead')
                    return res
                if fc < need_clear:
                    goal_handle.abort(); res.success = False
                    res.message = (f'only {fc:.0f} mm ahead, an arc turn needs '
                                   f'{need_clear:.0f} - NOT turning')
                    return res
            enc0, _ = self.mean_counts()
            # S-curve state. The slew starts at zero so the FIRST half also
            # eases in rather than snapping to full split.
            s_returning = False
            s_slew = Slew(float(self.get_parameter('scurve_slew_pwm_s').value))
            s_last = time.time()
            target = math.radians(abs(float(req.angle_deg)))
            direction = 1.0 if req.angle_deg >= 0 else -1.0     # + = left
            tol = math.radians(float(self.get_parameter('yaw_tol_deg').value))
            deadline = time.time() + float(self.get_parameter('max_seconds').value)
            acc, prev = 0.0, ref0
            imu_acc, imu_prev = 0.0, yaw0     # tracked ONLY to measure the
                                              # board's scale against GICP
            fb = Turn.Feedback()

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.halt(); goal_handle.canceled()
                    res.success = False; res.message = 'cancelled'
                    res.turned_deg = math.degrees(acc)
                    return res
                if time.time() > deadline:
                    self.halt(); goal_handle.abort()
                    res.success = False; res.message = 'deadline - angle not reached'
                    res.turned_deg = math.degrees(acc)
                    return res
                with self.lock:
                    byaw, gy, gat = self.yaw, self.gicp_yaw, self.gicp_at
                if src == 'gicp':
                    if (time.time() - gat) > stale:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = ('scan_odom went stale mid-turn - '
                                       'stopped rather than turn blind')
                        res.turned_deg = math.degrees(abs(acc))
                        return res
                    cur = math.radians(gy)
                else:
                    cur = byaw
                # Accumulate so a wrap past +-pi is not read as a huge jump.
                d = math.atan2(math.sin(cur - prev), math.cos(cur - prev))
                acc += d
                prev = cur
                if byaw is not None and imu_prev is not None:
                    imu_acc += math.atan2(math.sin(byaw - imu_prev),
                                          math.cos(byaw - imu_prev))
                    imu_prev = byaw
                remain = target - abs(acc)
                # In scurve mode reaching the peak is a PHASE CHANGE, not the
                # end of the goal - the goal ends when the heading is restored.
                if remain <= tol and mode != 'scurve':
                    break
                if mode == 'scurve':
                    # PHASE 1 curve toward the peak, PHASE 2 curve back.
                    # `acc` is measured rotation, so the switch happens on real
                    # heading, not on a timer or a dead-reckoned guess.
                    if not s_returning and abs(acc) >= target:
                        s_returning = True
                        self.get_logger().info(
                            f'S-curve: peak {math.degrees(abs(acc)):.1f} deg '
                            f'reached, reversing WITHOUT stopping')
                    want = -1.0 if s_returning else 1.0
                    # Slew the SPLIT, not the curvature: the kappa->duty map is
                    # not calibrated, and slewing a number we cannot convert
                    # would be pretending we can.
                    tgt_split = want * direction * (arc_out - arc_in) / 2.0
                    now_t = time.time()
                    s_split = s_slew.step(tgt_split, max(1e-3, now_t - s_last))
                    s_last = now_t
                    mid = (arc_out + arc_in) / 2.0
                    li = clamp(mid - s_split, -turn_cap, turn_cap)
                    ri = clamp(mid + s_split, -turn_cap, turn_cap)
                    self.motors(FWD_PWM_SIGN * li, FWD_PWM_SIGN * ri)

                    with self.lock:
                        fc = self.fwd_clear
                    if fc is not None and fc < need_clear * 0.5:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = (f'S ABORT: only {fc:.0f} mm ahead at '
                                       f'{math.degrees(abs(acc)):.1f} deg'
                                       f'{" (returning)" if s_returning else ""}')
                        res.turned_deg = math.degrees(abs(acc))
                        self.get_logger().warn(res.message)
                        return res

                    # Done when the heading is back, not when a distance is up.
                    if s_returning and abs(acc) <= math.radians(
                            float(self.get_parameter('yaw_tol_deg').value)):
                        break
                elif mode == 'arc':
                    # Both sides FORWARD, inner slower. A positive angle is a
                    # left turn, so the LEFT pair is the inner one.
                    near = remain <= math.radians(12)
                    o = arc_out if not near else max(MIN_PWM, arc_out - 6)
                    i_ = arc_in
                    li, ri = (i_, o) if direction > 0 else (o, i_)
                    self.motors(FWD_PWM_SIGN * li, FWD_PWM_SIGN * ri)
                    # How much forward room this turn still needs, from the
                    # radius it is ACHIEVING right now - so a too-wide arc is
                    # refused early instead of discovered against a wall.
                    if abs(acc) > math.radians(3):
                        enc_now, _ = self.mean_counts()
                        path = abs(enc_now - enc0) / self.cpmm
                        r_now = path / abs(acc)
                        if r_now > float(self.get_parameter(
                                'arc_max_radius_mm').value):
                            self.halt(); goal_handle.abort()
                            res.success = False
                            res.message = (
                                f'ARC TOO WIDE: radius {r_now:.0f} mm after '
                                f'{math.degrees(abs(acc)):.1f} deg - a '
                                f'{abs(req.angle_deg):.0f} deg turn would need '
                                f'~{r_now * math.radians(abs(req.angle_deg)):.0f}'
                                f' mm of travel. Lower arc_inner_pwm.')
                            res.turned_deg = math.degrees(abs(acc))
                            self.get_logger().warn(res.message)
                            return res

                    with self.lock:
                        fc = self.fwd_clear
                    if fc is not None and fc < need_clear * 0.5:
                        self.halt(); goal_handle.abort()
                        res.success = False
                        res.message = (f'ARC ABORT: only {fc:.0f} mm ahead '
                                       f'after {math.degrees(abs(acc)):.1f} deg')
                        res.turned_deg = math.degrees(abs(acc))
                        self.get_logger().warn(res.message)
                        return res
                else:
                    p = pwm if remain > math.radians(12) else APPROACH_PWM
                    v = FWD_PWM_SIGN * direction * p
                    self.motors(-v, v)           # left pair against right pair
                fb.turned_deg = math.degrees(abs(acc))
                fb.remaining_deg = math.degrees(max(0.0, remain))
                goal_handle.publish_feedback(fb)
                time.sleep(CTRL_DT)

            self.halt()
            time.sleep(0.5)
            with self.lock:
                yaw1, gy1 = self.yaw, self.gicp_yaw
            cur = math.radians(gy1) if (src == 'gicp' and gy1 is not None) else yaw1
            if cur is not None:
                acc += math.atan2(math.sin(cur - prev), math.cos(cur - prev))
            if yaw1 is not None and imu_prev is not None:
                imu_acc += math.atan2(math.sin(yaw1 - imu_prev),
                                      math.cos(yaw1 - imu_prev))

            res.turned_deg = math.degrees(abs(acc))
            res.success = True
            signed = math.degrees(acc)
            res.message = (f'turned {res.turned_deg:.1f} deg of '
                           f'{abs(req.angle_deg):.1f} deg '
                           f'[{src}] signed {signed:+.1f} deg')
            # The board's own number alongside, and the ratio. This is how the
            # gyro scale gets measured instead of guessed: the team found 1.60x
            # under-read on the sister board and no package can derive it.
            if abs(imu_acc) > 1e-6:
                res.message += (f'; board yaw {math.degrees(imu_acc):+.1f} deg'
                                f' -> scale {abs(acc / imu_acc):.3f}')
            # MEASURED arc geometry. Wheel path length over the angle actually
            # rotated gives the radius this duty pair produced on this surface -
            # the number to quote, rather than one derived from a kinematic
            # model that skid-steer scrub invalidates.
            encN, _ = self.mean_counts()
            path_mm = abs(encN - enc0) / self.cpmm
            res.message += f'; wheel path {path_mm:.0f} mm'
            if mode == 'arc' and abs(acc) > math.radians(2):
                res.message += (f'; MEASURED arc radius '
                                f'{path_mm / abs(acc):.0f} mm '
                                f'(outer {arc_out}/inner {arc_in})')
            goal_handle.succeed()
            self.get_logger().info(res.message)
            return res
        finally:
            self.halt()
            with self.lock:
                self.busy = False

    def on_stop(self, req, resp):
        self.halt()
        resp.success = True
        resp.message = 'motors halted'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.halt()
        except Exception:                                  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
