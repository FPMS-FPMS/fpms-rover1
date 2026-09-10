#!/usr/bin/env python3
"""thermal_node - Thermal Master P1 as ROS 2 topics.

THE CAMERA HAS NO KERNEL DRIVER AND NEVER WILL
-----------------------------------------------
The P1 (USB 3474:45c2) presents vendor-specific USB class 255 / subclass 240
with bulk endpoints - not UVC. Nothing in the kernel binds it, so there is no
/dev/video* node and no V4L2 path. It is reached over libusb by a userspace
driver: jvdillon/p3-ir-camera (Apache 2.0), which was reverse engineered from
USB traffic. That driver is NOT vendored here; point `driver_path` at a clone.

Needs a udev rule or the device is root-only:
    SUBSYSTEM=="usb", ATTR{idVendor}=="3474", ATTR{idProduct}=="45c2", MODE="0666"

WHAT IS PUBLISHED, AND WHY THREE TOPICS
----------------------------------------
  /thermal/image             mono8   the IR brightness image. What a human
                                     reads. NOT temperature.
  /thermal/temperature       32FC1   degrees Celsius per pixel. This is the
                                     measurement; the mono8 image above is an
                                     AGC'd rendering of it and its grey levels
                                     do NOT map linearly to temperature.
  /thermal/image/compressed  png     colour-mapped, for the browser. rosbridge
                                     ships binary as base64 JSON, so a raw
                                     32FC1 frame would be ~102 KB per frame
                                     before encoding. PNG keeps it small.
  /thermal/stats             min, max, mean, centre in Celsius - cheap enough
                                     for a UI to read every frame without
                                     decoding an image at all.

Measured on Rover 1, 2026-09-10: 160x120, 9.0 fps, 23.5-35.7 C.
"""

from __future__ import annotations

import os
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

DEFAULT_DRIVER = os.path.expanduser('~/drivers/p3-ir-camera')


def ironbow(n: int = 256) -> np.ndarray:
    """A black-red-yellow-white ramp, built here so the node needs no cv2.

    Perceptually monotonic in lightness, which matters: a rainbow map invents
    edges that are not in the data and makes a thermal image lie.
    """
    x = np.linspace(0.0, 1.0, n)
    r = np.clip(x * 3.0, 0, 1)
    g = np.clip(x * 3.0 - 1.0, 0, 1)
    b = np.clip(x * 3.0 - 2.0, 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


LUT = ironbow()


class ThermalNode(Node):
    def __init__(self):
        super().__init__('thermal_node')
        self.declare_parameter('driver_path', DEFAULT_DRIVER)
        self.declare_parameter('frame_id', 'thermal')
        self.declare_parameter('model', 'p1')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('max_fps', 9.0)

        self.driver_path = self.get_parameter('driver_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.model = str(self.get_parameter('model').value).lower()
        self.want_compressed = bool(self.get_parameter('publish_compressed').value)
        self.min_dt = 1.0 / max(0.5, float(self.get_parameter('max_fps').value))

        qos = QoSProfile(depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_ir = self.create_publisher(Image, 'thermal/image', qos)
        self.pub_temp = self.create_publisher(Image, 'thermal/temperature', qos)
        self.pub_png = self.create_publisher(CompressedImage, 'thermal/image/compressed', qos)
        self.pub_stats = self.create_publisher(Float32MultiArray, 'thermal/stats', qos)

        self.frames = 0
        self.errors = 0
        self.last_temp = None
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'thermal node up, driver at {self.driver_path}')

    # -- capture -----------------------------------------------------------
    def _run(self):
        # Deliberately step DOWN off real-time. The unit runs SCHED_FIFO for
        # the scanner and STM32 readers, whose serial buffers overflow if they
        # are late. This node PNG-encodes at 8 fps - that is throughput work
        # with no latency requirement, and letting it preempt the readers
        # would trade a real deadline for an imaginary one.
        try:
            os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
            self.get_logger().info('capture thread on SCHED_OTHER (not RT)')
        except Exception as exc:                           # noqa: BLE001
            self.get_logger().warn(f'could not leave RT: {exc}')
        if self.driver_path not in sys.path:
            sys.path.insert(0, self.driver_path)
        while self.running and rclpy.ok():
            cam = None
            try:
                from p3_camera import Model, P3Camera, get_model_config, raw_to_celsius
                model = Model.P1 if self.model == 'p1' else Model.P3
                cam = P3Camera(config=get_model_config(model))
                cam.connect()
                cam.init()
                cam.start_streaming()
                self.get_logger().info(f'camera streaming ({self.model})')
                self._pump(cam, raw_to_celsius)
            except Exception as exc:                       # noqa: BLE001
                self.errors += 1
                self.get_logger().warn(f'camera: {type(exc).__name__}: {exc}')
            finally:
                try:
                    if cam:
                        cam.stop_streaming()
                except Exception:                          # noqa: BLE001
                    pass
            for _ in range(30):
                if not (self.running and rclpy.ok()):
                    return
                time.sleep(0.1)

    def _pump(self, cam, raw_to_celsius):
        next_at = 0.0
        while self.running and rclpy.ok():
            ir, raw = cam.read_frame_both()
            now = time.time()
            if now < next_at:
                continue
            next_at = now + self.min_dt
            temps = np.asarray(raw_to_celsius(raw), dtype=np.float32)
            self._publish(np.asarray(ir, dtype=np.uint8), temps)

    # -- publish -----------------------------------------------------------
    def _publish(self, ir: np.ndarray, temps: np.ndarray):
        stamp = self.get_clock().now().to_msg()
        h, w = ir.shape[:2]

        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.height, m.width = h, w
        m.encoding = 'mono8'
        m.is_bigendian = 0
        m.step = w
        m.data = ir.tobytes()
        self.pub_ir.publish(m)

        t = Image()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.height, t.width = temps.shape
        t.encoding = '32FC1'
        t.is_bigendian = 0
        t.step = temps.shape[1] * 4
        t.data = temps.astype(np.float32).tobytes()
        self.pub_temp.publish(t)

        tmin = float(np.min(temps)); tmax = float(np.max(temps))
        s = Float32MultiArray()
        s.data = [tmin, tmax, float(np.mean(temps)),
                  float(temps[temps.shape[0] // 2, temps.shape[1] // 2])]
        self.pub_stats.publish(s)
        self.last_temp = (tmin, tmax)

        if self.want_compressed:
            try:
                from PIL import Image as PILImage
                import io
                # Scale across THIS frame's own span. A fixed span would make
                # a 24-25 C room look uniformly black; the real range is
                # published on /thermal/temperature for anything that needs
                # absolute values.
                span = max(0.5, tmax - tmin)
                norm = np.clip((temps - tmin) / span, 0, 1)
                idx = (norm * 255).astype(np.uint8)
                rgb = LUT[idx]
                buf = io.BytesIO()
                PILImage.fromarray(rgb, mode='RGB').save(buf, format='PNG', optimize=False)
                c = CompressedImage()
                c.header.stamp = stamp
                c.header.frame_id = self.frame_id
                c.format = 'png'
                c.data = buf.getvalue()
                self.pub_png.publish(c)
            except Exception as exc:                       # noqa: BLE001
                self.get_logger().warn(f'png encode: {type(exc).__name__}: {exc}')
                self.want_compressed = False

        self.frames += 1

    def _report(self):
        rng = (f'{self.last_temp[0]:.1f}-{self.last_temp[1]:.1f} C'
               if self.last_temp else 'no frame')
        self.get_logger().info(f'frames={self.frames} errors={self.errors} {rng}')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalNode()
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
