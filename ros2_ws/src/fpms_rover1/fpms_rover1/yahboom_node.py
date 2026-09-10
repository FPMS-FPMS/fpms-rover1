#!/usr/bin/env python3
"""yahboom_node - the STM32 Yahboom ROS Board V3.0 as ROS 2 topics. READ ONLY.

THIS NODE CANNOT MOVE THE ROVER, BY CONSTRUCTION
-------------------------------------------------
There is no motor command anywhere in this file - no /cmd_vel subscriber, no
set_motor, no set_car_motion. That is deliberate and it is not an oversight:

  * Rover 2's own code records that `set_motor` NEGATIVE values drive FORWARD
    on that chassis. Whether Rover 1 shares that sign is UNVERIFIED. Getting it
    backwards means the rover drives into whatever is behind it.
  * The correct way to settle it is with the wheels off the ground, watching
    them, not by inferring it over SSH.

The ONLY byte this node can ever transmit is the auto-report enable
(FUNC_AUTO_REPORT, 0x01), and only if the board is silent. That command
configures telemetry streaming and cannot actuate anything.

WHY THE PROTOCOL IS REIMPLEMENTED HERE INSTEAD OF USING Rosmaster_Lib
---------------------------------------------------------------------
Rosmaster_Lib is not on PyPI; it ships as a zip from the vendor. Two
independent GitHub mirrors of it were compared on 2026-09-10 and they differ
by ~4000 lines (107 KB vs 59 KB) - so neither can be trusted as authentic
vendor code, and that library contains the motor write path. Rather than run
unverified code that can drive motors, only the RECEIVE half of the protocol
is implemented here, from the wire format both mirrors agree on.

WIRE FORMAT (receive)
    0xFF 0xFB <len> <type> <data...> <checksum>
    len      = payload length + 2
    checksum = (len + type + sum(data[:-1])) & 0xFF
    115200 baud, 8N1. Both mirrors agree on the baud.

PAYLOADS
    0x0A speed     vx,vy,vz int16 /1000 m/s ; battery uint8 /10 V
    0x0B mpu raw   MPU9250. Defined by the protocol; THIS board never sends it.
    0x0E icm raw   ICM20948 - what this board actually sends. gyro 16.4 LSB/dps,
                   accel 2048 LSB/g, mag 6.67 LSB/uT.
    0x0C attitude  roll,pitch,yaw int16 /10000 rad
    0x0D encoder   4 x int32, cumulative counts
"""

from __future__ import annotations

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, BatteryState, MagneticField
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from std_msgs.msg import Int32MultiArray

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = 115200

HEAD = 0xFF
DEVICE_ID = 0xFC
RX_ID = DEVICE_ID - 1              # 0xFB on frames coming FROM the board
COMPLEMENT = 257 - DEVICE_ID

FUNC_AUTO_REPORT = 0x01
FUNC_REPORT_SPEED = 0x0A
FUNC_REPORT_MPU_RAW = 0x0B
FUNC_REPORT_IMU_ATT = 0x0C
FUNC_REPORT_ENCODER = 0x0D
FUNC_REPORT_ICM_RAW = 0x0E

# MPU9250 path (+-500 dps, +-2 g). Kept because the protocol defines it, but
# THIS board does not use it - see below.
GYRO_SCALE = 1.0 / 3754.9
ACCEL_SCALE = 1.0 / 1671.84

# ICM20948 path. Verified empirically on Rover 1: the board emits 0x0E with an
# 18-byte payload and never 0x0B, so its IMU is an ICM20948 despite the
# vendor's "MPU9250" marketing. Full-scale settings give the LSB ratios below.
#   gyro  +-2000 dps -> 16.4 LSB/dps
#   accel +-16 g     -> 2048 LSB/g
#   mag   AK09916    -> 6.67 LSB/uT  (0.15 uT/LSB)
# Note the vendor library defines these same ratios and then does NOT apply
# them on the ICM branch, returning raw counts labelled as physical units.
#
# ACCEL SCALE WAS MEASURED, NOT ASSUMED - and none of the datasheet
# full-scales fit. With the rover stationary, |accel| must be exactly 1 g.
# Median over 150 samples was 10245 counts (spread 10081-10413), which gives:
#     +-2g  (16384 LSB/g) -> 0.625 g   wrong
#     +-4g  ( 8192 LSB/g) -> 1.251 g   wrong
#     +-8g  ( 4096 LSB/g) -> 2.501 g   wrong
#     +-16g ( 2048 LSB/g) -> 5.002 g   wrong
# 10240 counts/g (= 1024 * 10) lands within 0.05% of the measurement, so that
# is taken as the board's fixed-point constant. Re-measure if the STM32
# firmware is ever reflashed.
ICM_ACCEL_COUNTS_PER_G = 10240.0
ICM_ACCEL_SCALE = 9.80665 / ICM_ACCEL_COUNTS_PER_G  # -> m/s^2

# GYRO SCALE IS NOT VERIFIED. 16.4 LSB/dps is the ICM20948 +-2000 dps figure,
# but the accel constant above shows this board does not use datasheet scaling,
# so treat angular_velocity as provisional. Verifying it needs a KNOWN
# rotation (e.g. turn the rover exactly 90 degrees and integrate) - it cannot
# be derived at rest, where the only honest reading is the ~6-count bias.
ICM_GYRO_SCALE = (1.0 / 16.4) * math.pi / 180.0     # -> rad/s, PROVISIONAL

# Mag likewise unverified, and uncalibrated regardless (no hard/soft iron).
ICM_MAG_SCALE = (1.0 / 6.67) * 1e-6                 # -> tesla, PROVISIONAL


class YahboomNode(Node):
    def __init__(self):
        super().__init__('yahboom_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('frame_id', 'imu_link')
        # The single permitted transmission. Set false to be strictly passive;
        # if the board is not already auto-reporting you will then get nothing.
        self.declare_parameter('enable_auto_report', True)

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.may_enable = bool(self.get_parameter('enable_auto_report').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.pub_att = self.create_publisher(Vector3Stamped, 'rover/attitude', qos)
        self.pub_bat = self.create_publisher(BatteryState, 'rover/battery', qos)
        self.pub_enc = self.create_publisher(Int32MultiArray, 'rover/encoders', qos)
        self.pub_vel = self.create_publisher(TwistStamped, 'rover/velocity', qos)
        self.pub_mag = self.create_publisher(Vector3Stamped, 'rover/mag_raw', qos)
        self.pub_magf = self.create_publisher(MagneticField, 'imu/mag', qos)

        self.frames = 0
        self.checksum_errors = 0
        self.battery_v = None
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'yahboom node up (READ ONLY), {self.port} @ {BAUD}')

    # -- serial ------------------------------------------------------------
    def _run(self):
        import serial
        while self.running and rclpy.ok():
            try:
                with serial.Serial(self.port, BAUD, timeout=1) as ser:
                    self.get_logger().info('board port opened')
                    if self.may_enable and not self._listen(ser, 3.0):
                        self.get_logger().info('board silent - enabling auto-report '
                                               '(telemetry only, cannot actuate)')
                        self._enable_auto_report(ser)
                    self._pump(ser)
            except Exception as exc:                        # noqa: BLE001
                self.get_logger().warn(f'board: {type(exc).__name__}: {exc}')
                time.sleep(2.0)

    def _listen(self, ser, secs: float) -> bool:
        """Is the board already talking? Listen before transmitting anything."""
        end = time.time() + secs
        while time.time() < end:
            if ser.read(1) == b'\xff':
                return True
        return False

    def _enable_auto_report(self, ser):
        cmd = [HEAD, DEVICE_ID, 0x05, FUNC_AUTO_REPORT, 1, 0]
        cmd.append(sum(cmd, COMPLEMENT) & 0xFF)
        ser.write(bytes(cmd))

    def _pump(self, ser):
        while self.running and rclpy.ok():
            if ser.read(1) != b'\xff':
                continue
            b = ser.read(1)
            if not b or b[0] != RX_ID:
                continue
            hdr = ser.read(2)
            if len(hdr) < 2:
                continue
            ext_len, ext_type = hdr[0], hdr[1]
            n = ext_len - 2
            if n < 1 or n > 64:
                continue
            body = ser.read(n)
            if len(body) < n:
                continue
            data, rx_ck = body[:-1], body[-1]
            if ((ext_len + ext_type + sum(data)) & 0xFF) != rx_ck:
                self.checksum_errors += 1
                continue
            self.frames += 1
            self._dispatch(ext_type, data)

    # -- payloads ----------------------------------------------------------
    def _dispatch(self, t: int, d: bytes):
        now = self.get_clock().now().to_msg()
        if t == FUNC_REPORT_SPEED and len(d) >= 7:
            vx, vy, vz = struct.unpack('<hhh', d[0:6])
            self.battery_v = d[6] / 10.0
            m = TwistStamped()
            m.header.stamp = now
            m.header.frame_id = 'base_link'
            m.twist.linear.x = vx / 1000.0
            m.twist.linear.y = vy / 1000.0
            m.twist.angular.z = vz / 1000.0
            self.pub_vel.publish(m)

            b = BatteryState()
            b.header.stamp = now
            b.voltage = float(self.battery_v)
            b.present = self.battery_v > 1.0
            # Everything not measured stays NaN rather than 0.0, which would
            # read as "measured, and it is zero".
            b.current = float('nan'); b.charge = float('nan')
            b.capacity = float('nan'); b.design_capacity = float('nan')
            b.percentage = float('nan')
            self.pub_bat.publish(b)

        elif t == FUNC_REPORT_MPU_RAW and len(d) >= 18:
            gx, gy, gz, ax, ay, az, mx, my, mz = struct.unpack('<9h', d[0:18])
            m = Imu()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.angular_velocity.x = gx * GYRO_SCALE
            m.angular_velocity.y = gy * -GYRO_SCALE      # board axes -> REP-103
            m.angular_velocity.z = gz * -GYRO_SCALE
            m.linear_acceleration.x = ax * ACCEL_SCALE
            m.linear_acceleration.y = ay * ACCEL_SCALE
            m.linear_acceleration.z = az * ACCEL_SCALE
            # -1 in element 0 is the REP-145 way of saying "no orientation in
            # this message". The attitude the board computes is published
            # separately on rover/attitude; claiming it here would imply a
            # fused quaternion that this message does not carry.
            m.orientation_covariance[0] = -1.0
            self.pub_imu.publish(m)

            g = Vector3Stamped()
            g.header.stamp = now
            g.header.frame_id = self.frame_id
            # RAW COUNTS, not Tesla. The magnetometer is uncalibrated and the
            # vendor documents no scale factor, so publishing it as a
            # MagneticField in Tesla would be inventing units.
            g.vector.x, g.vector.y, g.vector.z = float(mx), float(my), float(mz)
            self.pub_mag.publish(g)

        elif t == FUNC_REPORT_ICM_RAW and len(d) >= 18:
            gx, gy, gz, ax, ay, az, mx, my, mz = struct.unpack('<9h', d[0:18])
            m = Imu()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.angular_velocity.x = gx * ICM_GYRO_SCALE
            m.angular_velocity.y = -gy * ICM_GYRO_SCALE   # board axes -> REP-103
            m.angular_velocity.z = -gz * ICM_GYRO_SCALE
            m.linear_acceleration.x = ax * ICM_ACCEL_SCALE
            m.linear_acceleration.y = ay * ICM_ACCEL_SCALE
            m.linear_acceleration.z = az * ICM_ACCEL_SCALE
            m.orientation_covariance[0] = -1.0     # REP-145: no orientation here
            self.pub_imu.publish(m)

            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = self.frame_id
            # Real tesla - the AK09916 scale factor is documented. It is
            # UNCALIBRATED though: no hard- or soft-iron correction, so use it
            # for change detection, not for absolute heading.
            mag.magnetic_field.x = mx * ICM_MAG_SCALE
            mag.magnetic_field.y = my * ICM_MAG_SCALE
            mag.magnetic_field.z = mz * ICM_MAG_SCALE
            mag.magnetic_field_covariance[0] = -1.0
            self.pub_magf.publish(mag)

        elif t == FUNC_REPORT_IMU_ATT and len(d) >= 6:
            roll, pitch, yaw = struct.unpack('<hhh', d[0:6])
            v = Vector3Stamped()
            v.header.stamp = now
            v.header.frame_id = self.frame_id
            v.vector.x = roll / 10000.0
            v.vector.y = pitch / 10000.0
            v.vector.z = yaw / 10000.0
            self.pub_att.publish(v)

        elif t == FUNC_REPORT_ENCODER and len(d) >= 16:
            e = struct.unpack('<4i', d[0:16])
            m = Int32MultiArray()
            m.data = list(e)
            self.pub_enc.publish(m)

    def _report(self):
        bat = f'{self.battery_v:.1f} V' if self.battery_v is not None else 'no battery reading'
        self.get_logger().info(
            f'frames={self.frames} checksum_errors={self.checksum_errors} {bat}')


def main(args=None):
    rclpy.init(args=args)
    node = YahboomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
