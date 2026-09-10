# FPMS Rover 1 — ground station integration

Rover 1 is an **Orange Pi 5 Max** (`fpms1`) running Ubuntu 22.04.5 jammy.
This repo holds everything that makes it visible from the FPMS Cortex on the
ground station laptop.

Built and verified against real hardware on 2026-09-10.

## Hardware, as actually wired

| Device | Bus | Port | Status |
|---|---|---|---|
| **LDROBOT D500 / LD19** LiDAR | CP2102 USB-UART | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._0001-if00-port0` | working, 230400 baud |
| **STM32 Yahboom ROS Board V3.0** | CH340 USB-UART | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | identified, not yet driven |
| **Thermal Master P1** | USB `3474:45c2` | none | needs a userspace driver |

Use the `by-id` paths. `ttyUSB0`/`ttyUSB1` reorder between boots.

### The LiDAR frame format is confirmed, not assumed

LD19 frames are 47 bytes: `0x54`, `0x2C`, speed u16, start angle u16,
12 x (distance u16, intensity u8), end angle u16, timestamp u16, CRC8.
CRC8 is polynomial `0x4D`, init 0, no reflection.

Verified against the live scanner: **419 frames, 0 CRC rejects**, ~9.9 Hz,
returns from 64 mm to 7191 mm. The decoder in `rover/fpms-rover-bridge`
rejects any frame that fails CRC rather than rendering garbage, and exports
`crc_reject` so the UI can show link quality.

### The thermal camera has no kernel driver, and never will

It presents vendor-specific USB class 255 / subclass 240 with bulk endpoints,
not UVC. There is no `/dev/video*` for it. It is **not** broken — userspace
drivers exist, e.g. [jvdillon/p3-ir-camera](https://github.com/jvdillon/p3-ir-camera)
(Apache 2.0, confirms `P1 = 3474:45c2, 160x120`, gives per-pixel °C).
Until that is wired in, the UI reports it as present-but-unusable and shows
**nothing** — never a placeholder that could be mistaken for a live feed.

### Rover 1 has no camera of its own

Its only video nodes are `/dev/video-dec0` and `/dev/video-enc0`, which are the
RK3588 hardware codecs, not capture devices. The "regular camera" in the
Cameras tab is the **ground station laptop's** webcam. The panel says so.

## Layout

    rover/          runs ON the Orange Pi
      fpms-rover-bridge          LiDAR + health over HTTP (port 8781)
      fpms-rover-bridge.service  systemd unit, runs unprivileged
      fpms-wpa-priority          makes wifi prefer FPMS_Net (see below)
    cortex/         panels for the FPMS Cortex on the ground station
      backend/panel_{control,lidar,cameras}.py
      web/panel_{control,lidar,cameras}.js
    groundstation/
      FPMS_Net                   one command to bring the rover AP up

## Install

### On the rover

    sudo install -o root -g root -m 0755 rover/fpms-rover-bridge /usr/local/bin/
    sudo install -o root -g root -m 0644 rover/fpms-rover-bridge.service /etc/systemd/system/
    sudo systemctl enable --now fpms-rover-bridge

    sudo install -o root -g root -m 0755 rover/fpms-wpa-priority /usr/local/sbin/
    sudo mkdir -p /etc/systemd/system/netplan-wpa-wlan0.service.d
    printf '[Service]\nExecStartPre=/usr/local/sbin/fpms-wpa-priority\n' \
      | sudo tee /etc/systemd/system/netplan-wpa-wlan0.service.d/10-fpms-priority.conf
    sudo systemctl daemon-reload

### On the ground station

Copy the panels into the installed Cortex and register three tabs in
`web/app.js`'s `PANELS` array:

    { id: 'control', label: 'Control', glyph: '◈', mod: './panel_control.js', needs: 'control' },
    { id: 'lidar',   label: 'LiDAR',   glyph: '◎', mod: './panel_lidar.js',   needs: 'lidar' },
    { id: 'cameras', label: 'Cameras', glyph: '▣', mod: './panel_cameras.js', needs: 'cameras' },

`/usr/local` is covered by `fpms-integrity`, so afterwards:

    sudo fpms-integrity baseline

## Two decisions worth knowing

**netplan has no `priority` key for wifi.** With two access points in one
block, wpa_supplicant keeps whichever it associates with first and never
reconsiders — observed here sitting on the home network while the ground
station AP was **48 dB stronger**. `fpms-wpa-priority` post-processes
netplan's generated config to add `priority=`. It runs as `ExecStartPre` for
the supplicant, so **it must never exit non-zero** — a failure there leaves
the rover with no wifi and no way in. Every path in it bails with `exit 0`.

**The LiDAR scan does not go through panel state.** A 360-point scan is ~6 KB
at 10 Hz, and Cortex panel state is merged into the SSE snapshot sent to every
client every tick — that path already caused a 211 KB frame of which the
panels dict was 96%. So the browser fetches `/scan` straight from the rover;
`panel_lidar.py` only hands over the URL.

## Status

Working today over plain HTTP. ROS 2 **Humble** (jammy, the correct base) is
being added alongside this, with the HTTP path kept as the fallback.
Humble only — a Jazzy/Iron node on this network can crash the robot's Humble
nodes with `std::bad_alloc`.

## ROS 2 Humble (added 2026-09-10)

ROS runs **alongside** the HTTP bridge, not instead of it.

    ros2_ws/src/fpms_rover1/          ament_python package
      fpms_rover1/ld19_node.py        LD19 -> sensor_msgs/LaserScan on /scan
      launch/rover1.launch.py         the node + rosbridge on :9090
    rover/fpms-ros-rover1.service     systemd unit, ROS_DOMAIN_ID=20

Build and enable on the rover:

    cd ~/ros2_ws && colcon build --packages-select fpms_rover1 --symlink-install
    sudo install -o root -g root -m 0644 rover/fpms-ros-rover1.service /etc/systemd/system/
    sudo systemctl enable --now fpms-ros-rover1

### The two paths are mutually exclusive, on purpose

The scanner's serial port has one owner. Linux will let **both** the ROS node
and the HTTP bridge open it, and then each receives a fraction of the bytes and
both silently emit corrupt scans - no error, just wrong geometry. So the two
units declare `Conflicts=` on each other: starting either stops the other.

ROS is the primary path and is `enabled`. The HTTP bridge is installed but
`disabled`; start it and the ROS stack stops automatically:

    sudo systemctl start fpms-rover-bridge     # falls back to HTTP
    sudo systemctl start fpms-ros-rover1       # back to ROS

`panel_lidar.js` prefers rosbridge and drops to HTTP only after ROS has
actually gone quiet for 2 s, and shows which source is live.

### Measured, not assumed

- `/scan` at **9.93 Hz**, 0 CRC rejects over 4139 frames, 596 rpm
- LaserScan: 360 rays, 1.00 deg increment, angle_min 0 -> angle_max 2pi-inc
- Subscribed from the ground station over rosbridge: 360 rays, 350 returns,
  0.111-7.153 m, 9.7 Hz

A ray with no return is published as **+inf**, the ROS convention - not 0.0,
which would read as an obstacle at the origin. Clockwise sweep is corrected by
remapping the array, keeping `angle_increment` positive, because a negative
increment with `angle_min > angle_max` is legal but several consumers
mishandle it.
