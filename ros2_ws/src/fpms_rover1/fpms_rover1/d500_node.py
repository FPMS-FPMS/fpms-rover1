#!/usr/bin/env python3
"""d500_node - publishes the LDROBOT D500 scanner as sensor_msgs/LaserScan.

The D500 speaks the LD19 wire protocol, which is why an LD19 frame decoder
works against it unchanged. The HARDWARE IS A D500.

THE FRAME FORMAT HERE IS CONFIRMED, NOT ASSUMED
------------------------------------------------
47 bytes: 0x54, 0x2C, speed u16, start angle u16, 12 x (dist u16, intensity u8),
end angle u16, timestamp u16, CRC8. CRC8 is polynomial 0x4D, init 0, no
reflection. Verified against the live scanner on Rover 1, 2026-09-10:
419 frames, 0 CRC rejects, ~9.9 Hz, returns 64-7191 mm.

A frame that fails CRC is DROPPED, never guessed at, and counted. Rendering a
corrupt frame as geometry is worse than dropping it - it puts obstacles in the
map that are not there.

WHY A REVOLUTION IS DETECTED BY ANGLE WRAP
-------------------------------------------
The scanner does not announce a revolution boundary. Each frame carries only a
start and end angle, so a full sweep is assembled by watching for the angle to
wrap past its previous value. Points land in fixed 1-degree bins, which makes
angle_increment exact and keeps LaserScan's contract (evenly spaced rays)
honest - interpolating irregular samples into a regular array is what makes a
scan look subtly wrong in RViz.

Bins with no return are published as +inf, which is the ROS convention for
"nothing there", NOT 0.0 - a 0.0 range reads as an obstacle at the origin.
"""

from __future__ import annotations

import math
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan

HDR, VERLEN, FRAME_LEN, POINTS = 0x54, 0x2C, 47, 12
BINS = 360

DEFAULT_PORT = ("/dev/serial/by-id/"
                "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")

_CRC_T = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = ((_c << 1) ^ 0x4D) & 0xFF if (_c & 0x80) else (_c << 1) & 0xFF
    _CRC_T.append(_c)


def crc8(buf: bytes) -> int:
    c = 0
    for b in buf:
        c = _CRC_T[(c ^ b) & 0xFF]
    return c


class D500Node(Node):
    def __init__(self):
        super().__init__('d500_node')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', 230400)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 12.0)
        # The LD19 spins clockwise when viewed from above, which is the
        # opposite of the REP-103 right-handed convention ROS expects. Flipping
        # here means RViz and Nav2 see a correctly handed scan instead of a
        # mirrored world - a bug that looks like bad odometry, not a bad driver.
        self.declare_parameter('invert', True)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.invert = bool(self.get_parameter('invert').value)

        # Sensor data is best-effort by convention: a late scan is useless, so
        # never make the scanner wait on a slow subscriber.
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, 'scan', qos)

        self.dist = [0.0] * BINS
        self.inten = [0.0] * BINS
        self.have = [False] * BINS
        self.speed_dps = 0.0
        self.last_start = None
        self.frames = 0
        self.crc_reject = 0
        self.scans = 0
        self.last_stamp = self.get_clock().now()

        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'D500 node up, reading {self.port} at {self.baud}')

    # -- serial ------------------------------------------------------------
    def _reader(self):
        import serial
        while self.running and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.get_logger().info('scanner opened')
                    self._pump(ser)
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().warn(f'scanner: {type(exc).__name__}: {exc}')
                for _ in range(20):
                    if not (self.running and rclpy.ok()):
                        return
                    rclpy.spin_once(self, timeout_sec=0.1)

    def _pump(self, ser):
        buf = bytearray()
        while self.running and rclpy.ok():
            chunk = ser.read(940)
            if not chunk:
                self.get_logger().warn('no data - is the scanner spinning?')
                return
            buf += chunk
            if len(buf) > 8192:            # a desync must never grow unbounded
                del buf[:-2048]
            while len(buf) >= FRAME_LEN:
                if buf[0] != HDR or buf[1] != VERLEN:
                    buf.pop(0)
                    continue
                frame = bytes(buf[:FRAME_LEN])
                if crc8(frame[:-1]) != frame[-1]:
                    self.crc_reject += 1
                    buf.pop(0)
                    continue
                self._ingest(frame)
                del buf[:FRAME_LEN]

    def _ingest(self, f: bytes):
        speed, start = struct.unpack_from('<HH', f, 2)
        end, _ts = struct.unpack_from('<HH', f, 6 + POINTS * 3)
        self.speed_dps = float(speed)
        self.frames += 1

        # Wrapped past the previous start angle -> a revolution completed.
        if self.last_start is not None and start < self.last_start:
            self._publish()
        self.last_start = start

        span = (end - start) % 36000
        step = span / (POINTS - 1) if POINTS > 1 else 0.0
        for i in range(POINTS):
            d, q = struct.unpack_from('<HB', f, 6 + i * 3)
            ang = ((start + step * i) % 36000) / 100.0
            b = int(ang) % BINS
            if d <= 0:
                continue
            self.dist[b] = d / 1000.0
            self.inten[b] = float(q)
            self.have[b] = True

    # -- publish -----------------------------------------------------------
    def _publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds / 1e9
        self.last_stamp = now
        if not any(self.have):
            return

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        inc = 2.0 * math.pi / BINS

        # Always publish a POSITIVE increment with angle_min < angle_max. A
        # negative increment with angle_min > angle_max is legal but ambiguous,
        # and several consumers (RViz, some Nav2 layers) mishandle it. The
        # scanner's clockwise sweep is corrected by remapping the array
        # instead, which is unambiguous.
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - inc
        msg.angle_increment = inc
        msg.scan_time = float(dt) if dt > 0 else 0.1
        msg.time_increment = msg.scan_time / BINS
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges, intens = [], []
        for b in range(BINS):
            # ROS bin b is at angle +b degrees (counter-clockwise, REP-103).
            # The LD19 measures clockwise, so that ray is its bin -b.
            src = (BINS - b) % BINS if self.invert else b
            if self.have[src] and self.range_min <= self.dist[src] <= self.range_max:
                ranges.append(self.dist[src])
                intens.append(self.inten[src])
            else:
                ranges.append(float('inf'))   # ROS convention: no return
                intens.append(0.0)
        msg.ranges = ranges
        msg.intensities = intens
        self.pub.publish(msg)
        self.scans += 1

        self.have = [False] * BINS

    def _report(self):
        self.get_logger().info(
            f'scans={self.scans} frames={self.frames} crc_reject={self.crc_reject} '
            f'rpm={self.speed_dps / 6.0:.0f}')


def main(args=None):
    rclpy.init(args=args)
    node = D500Node()
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
