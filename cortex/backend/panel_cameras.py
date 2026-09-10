#!/usr/bin/env python3
"""panel_cameras - CAMERAS panel backend: thermal and regular.

TWO CAMERAS, AND ONLY ONE OF THEM EXISTS AS A USABLE FEED. READ THIS BEFORE
WIRING A VIEWER TO EITHER.

  regular   /dev/video0 on THIS machine - the laptop's built-in HD WebCam.
            It is NOT on the rover. Rover 1 has no camera fitted at all: its
            only video nodes are /dev/video-dec0 and /dev/video-enc0, which
            are the RK3588 hardware codecs, not capture devices.

  thermal   a Thermal Master P1 (USB 3474:45c2) IS plugged into the rover and
            IS on the bus, but no kernel driver binds it. It presents
            vendor-specific class 255 / subclass 240 with bulk endpoints, not
            UVC, so it will never appear as a V4L2 device and there is no
            frame to fetch. Reading it needs a userspace libusb driver against
            an undocumented protocol. Until that exists this panel reports it
            as present-but-unusable and shows nothing. It must never show a
            placeholder that could be mistaken for a live feed.

WHY ffmpeg WITH -c:v copy
--------------------------
The webcam emits Motion-JPEG natively (confirmed: mjpeg 1280x720/640x480).
Copying that stream costs no transcode at all, which matters on a 2-core
i7-7500U that also runs ROS. Decoding to raw and re-encoding would burn a core
for no gain, and the browser wants JPEG anyway.

THE CAMERA IS OPENED LAZILY AND RELEASED AUTOMATICALLY
------------------------------------------------------
/dev/video0 is exclusive: while this holds it, nothing else on the machine can
use the webcam. So capture starts on the first frame request and a watchdog
stops it once nobody has asked for IDLE_STOP seconds - i.e. as soon as you
leave the tab. Leaving a camera held open because a panel was mounted once is
exactly the kind of thing that gets discovered a week later.
"""

import base64
import json
import os
import subprocess
import threading
import time
import urllib.request

DEVICE = "/dev/video0"
WIDTH, HEIGHT, FPS = 640, 480, 5
IDLE_STOP = 6.0                    # release the device this long after last use

ROVER_HOST = "fpms1.local"
BRIDGE = "http://%s:8781" % ROVER_HOST
ROVER_POLL = 5.0

# The thermal camera is now LIVE over ROS. It has no kernel driver and no
# V4L2 node - a userspace libusb driver reads it and thermal_node publishes
# it - so the frames reach the browser through rosbridge, not through this
# backend. Sending them via panel state would put a 12 KB PNG into the SSE
# snapshot broadcast to every client on every tick.
ROSBRIDGE_WS = "ws://%s:9090" % ROVER_HOST
THERMAL_IMAGE_TOPIC = "/thermal/image/compressed"
THERMAL_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
THERMAL_STATS_TOPIC = "/thermal/stats"
THERMAL_STATS_TYPE = "std_msgs/msg/Float32MultiArray"

SOI = b"\xff\xd8\xff"
EOI = b"\xff\xd9"


class _Capture:
    """Holds one ffmpeg, hands out the most recent JPEG."""

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.frame = None
        self.frame_at = 0.0
        self.frames = 0
        self.started_at = 0.0
        self.last_want = 0.0
        self.error = None
        threading.Thread(target=self._watchdog, daemon=True).start()

    # -- lifecycle ---------------------------------------------------------
    def _spawn(self):
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2",
            "-input_format", "mjpeg",
            "-video_size", "%dx%d" % (WIDTH, HEIGHT),
            "-framerate", str(FPS),
            "-i", DEVICE,
            "-c:v", "copy",          # no transcode: the camera already sends JPEG
            "-f", "mjpeg", "pipe:1",
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, bufsize=0)
        self.started_at = time.time()
        self.frames = 0
        self.error = None
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def _read(self, proc):
        buf = bytearray()
        try:
            while proc.poll() is None:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                # Keep only the newest complete frame; a slow consumer must
                # never make this grow without bound.
                while True:
                    s = buf.find(SOI)
                    if s < 0:
                        if len(buf) > 1 << 20:
                            del buf[:-4]
                        break
                    e = buf.find(EOI, s + 3)
                    if e < 0:
                        if s > 0:
                            del buf[:s]
                        break
                    jpg = bytes(buf[s:e + 2])
                    del buf[:e + 2]
                    with self.lock:
                        self.frame, self.frame_at = jpg, time.time()
                        self.frames += 1
        except Exception as exc:                      # noqa: BLE001
            with self.lock:
                self.error = "%s: %s" % (type(exc).__name__, exc)

    def _detach(self):
        """Give up the process. CALLER MUST HOLD THE LOCK, and must reap the
        returned process OUTSIDE it - see _reap."""
        p, self.proc = self.proc, None
        self.frame = None
        return p

    @staticmethod
    def _reap(p):
        """Terminate and wait, WITHOUT holding the lock.

        Holding the lock across wait() deadlocks in practice: the reader thread
        needs the lock to store each frame, so it stops draining ffmpeg's
        stdout; the pipe fills; ffmpeg blocks in write() and cannot act on
        SIGTERM; wait() then burns its full timeout before the kill. Measured:
        release took 14 s instead of 7 s and the frame counter stuck at 0.
        """
        if not p or p.poll() is not None:
            return
        try:
            p.terminate()
            p.wait(timeout=2)
        except Exception:                              # noqa: BLE001
            try:
                p.kill()
            except Exception:                          # noqa: BLE001
                pass
        finally:
            try:
                if p.stdout:
                    p.stdout.close()
            except Exception:                          # noqa: BLE001
                pass

    def _watchdog(self):
        while True:
            time.sleep(1.0)
            doomed = None
            with self.lock:
                if self.proc is not None and \
                        (time.time() - self.last_want) > IDLE_STOP:
                    doomed = self._detach()
            self._reap(doomed)                         # lock released first

    # -- use ---------------------------------------------------------------
    def want(self):
        """Mark the camera as wanted now, starting it if necessary."""
        with self.lock:
            self.last_want = time.time()
            if self.proc is None:
                if not os.path.exists(DEVICE):
                    self.error = "%s does not exist" % DEVICE
                    return
                try:
                    self._spawn()
                except Exception as exc:               # noqa: BLE001
                    self.error = "%s: %s" % (type(exc).__name__, exc)

    def latest(self):
        with self.lock:
            return self.frame, self.frame_at, self.error

    def info(self):
        with self.lock:
            live = self.proc is not None and self.proc.poll() is None
            up = time.time() - self.started_at if live else 0.0
            return {
                "running": live,
                "frames": self.frames,
                "fps": round(self.frames / up, 1) if live and up > 1 else None,
                "age_s": round(time.time() - self.frame_at, 2) if self.frame_at else None,
                "error": self.error,
            }


CAM = _Capture()


class _RoverThermal:
    """Cheap periodic check of what the rover says about the thermal camera."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = None
        self.error = "not polled yet"
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with urllib.request.urlopen(BRIDGE + "/health", timeout=3) as r:
                    h = json.loads(r.read().decode("utf-8", "replace"))
                with self.lock:
                    self.data = (h.get("devices") or {}).get("thermal_cam")
                    self.error = None
            except Exception as exc:                   # noqa: BLE001
                with self.lock:
                    self.data, self.error = None, "%s: %s" % (type(exc).__name__, exc)
            time.sleep(ROVER_POLL)

    def get(self):
        with self.lock:
            return self.data, self.error


THERMAL = _RoverThermal()


def _local_nodes():
    return [f"/dev/video{n}" for n in range(8) if os.path.exists(f"/dev/video{n}")]


def state() -> dict:
    t_dev, t_err = THERMAL.get()
    return {
        "regular": {
            "device": DEVICE,
            "present": os.path.exists(DEVICE),
            "nodes": _local_nodes(),
            "size": [WIDTH, HEIGHT],
            "fps": FPS,
            "capture": CAM.info(),
            "where": "ground station",
            # Said explicitly because the obvious assumption is wrong.
            "note": "this is the LAPTOP's built-in webcam. Rover 1 has no "
                    "camera fitted - its only video nodes are the RK3588 "
                    "hardware codecs.",
        },
        "thermal": {
            "where": "rover",
            "host": ROVER_HOST,
            "device": "Thermal Master P1",
            "size": [160, 120],
            # Live as of 2026-09-10 via thermal_node over rosbridge.
            "usable": True,
            "rosbridge": ROSBRIDGE_WS,
            "image_topic": THERMAL_IMAGE_TOPIC,
            "image_type": THERMAL_IMAGE_TYPE,
            "stats_topic": THERMAL_STATS_TOPIC,
            "stats_type": THERMAL_STATS_TYPE,
            "note": "No kernel driver binds this camera (USB class 255/240, "
                    "not UVC), so there is no V4L2 node. A userspace libusb "
                    "driver reads it and thermal_node publishes it to ROS.",
            "colormap": "ironbow, scaled to each frame's own min/max - the "
                        "absolute values are on /thermal/temperature",
        },
    }


def act_frame(params):
    """Return the most recent JPEG, base64'd. Starts the camera if needed."""
    CAM.want()
    deadline = time.time() + 2.5
    while time.time() < deadline:
        jpg, at, err = CAM.latest()
        if jpg:
            return {"ok": True,
                    "b64": base64.b64encode(jpg).decode("ascii"),
                    "bytes": len(jpg),
                    "ts": at}
        if err:
            return {"ok": False, "error": "camera unavailable", "detail": err}
        time.sleep(0.05)
    _, _, err = CAM.latest()
    return {"ok": False, "error": "no frame within 2.5 s",
            "detail": err or "camera opened but produced nothing"}


def act_stop(params):
    """Release /dev/video0 immediately instead of waiting for the watchdog."""
    with CAM.lock:
        doomed = CAM._detach()
    CAM._reap(doomed)                                  # never under the lock
    return {"ok": True, "released": DEVICE}


def actions() -> dict:
    return {"frame": act_frame, "stop": act_stop}
