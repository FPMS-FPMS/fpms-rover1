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

## Thermal camera on ROS (2026-09-10)

The Thermal Master P1 is **live**. It has no kernel driver and no V4L2 node -
a userspace libusb driver reads it and `thermal_node` publishes it.

    /thermal/image             mono8  IR brightness. What a human reads.
    /thermal/temperature       32FC1  degrees Celsius PER PIXEL. The measurement.
    /thermal/image/compressed  png    ironbow, for the browser
    /thermal/stats                    min, max, mean, centre in Celsius

The mono8 image is an AGC'd rendering; **its grey levels do not map linearly
to temperature.** Anything that needs absolute values must read
`/thermal/temperature`, not the picture.

Setup on the rover (the driver is not vendored - point `driver_path` at a clone):

    git clone https://github.com/jvdillon/p3-ir-camera ~/drivers/p3-ir-camera
    sudo apt-get install -y python3-usb libusb-1.0-0
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="3474", ATTR{idProduct}=="45c2", MODE="0666"' \
      | sudo tee /etc/udev/rules.d/99-thermal-master-p1.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger

Without that udev rule the device is root-only and the node cannot open it.

### Measured

Camera reports `model P1, serial P1016021DG051600119, fw 00.00.00.04`.
160x120 at 9.0 fps from the driver, published at ~8.3 Hz on all four topics.
Verified over rosbridge from the ground station: a valid 160x120 PNG
(12,250 B) plus `min 19.2 / max 31.9 / mean 21.3 / centre 20.8 C`.

The colormap is scaled to **each frame's own** min/max. A fixed span would
render a 24-25 C room as uniform black. That is a display choice only; the
absolute values are on `/thermal/temperature`.

## Naming

The scanner is a **D500**. It speaks the LDROBOT LD19 protocol, which is why
the LD19 frame decoder works against it unchanged.

## STM32 Yahboom ROS Board V3.0 (2026-09-10) — READ ONLY

    /imu/data_raw      sensor_msgs/Imu          gyro + accel
    /imu/mag           sensor_msgs/MagneticField uncalibrated
    /rover/attitude    roll, pitch, yaw (rad) computed on the board
    /rover/battery     sensor_msgs/BatteryState
    /rover/encoders    4 x int32 cumulative counts
    /rover/velocity    the board's own vx, vy, wz

**This node cannot move the rover.** There is no `/cmd_vel` subscriber and no
motor write path — verified by AST, not by eye: exactly one `ser.write()`
(the telemetry-enable) and zero `create_subscription()`. Rover 2's code records
that `set_motor` **negative** drives **forward**; whether Rover 1 shares that
sign is unverified, and it gets tested with the wheels off the ground.

### Rosmaster_Lib is deliberately NOT used

It is not on PyPI — it ships as a vendor zip. Two independent GitHub mirrors
were compared and **differ by ~4000 lines** (107 KB vs 59 KB), so neither can
be trusted as authentic vendor code, and that library contains the motor write
path. Only the *receive* half of the protocol is reimplemented here, from the
wire format both mirrors agree on:

    0xFF 0xFB <len> <type> <data...> <checksum>
    checksum = (len + type + sum(data[:-1])) & 0xFF,  115200 8N1

### Two things the datasheet got wrong

**The IMU is an ICM20948, not the MPU9250 the vendor advertises.** The board
emits function `0x0E` with an 18-byte payload and never `0x0B`. Verified by
sniffing the live board.

**The accelerometer uses no datasheet full-scale.** Measured against gravity
with the rover stationary (median of 150 samples, |accel| must be 1 g):

    raw |accel| = 10245 counts
      +-2g  (16384 LSB/g) -> 0.625 g    +-8g  (4096) -> 2.501 g
      +-4g  ( 8192 LSB/g) -> 1.251 g    +-16g (2048) -> 5.002 g

None fit. **10240 counts/g** (1024 x 10) lands within 0.05% and is taken as the
board's fixed-point constant. After the fix, |accel| reads 9.839 m/s^2 against
a true 9.807 — **0.33% error**.

**Gyro and mag scales remain PROVISIONAL.** They cannot be derived at rest,
where the only honest reading is the ~6-count bias. Verifying the gyro needs a
known rotation. Since the accel proved not to follow the datasheet, do not
assume the gyro does.

## Thermal view (enhanced 2026-09-10)

The dashboard reads temperatures, not just colours.

**Node side — the display range is chosen properly.** Mapping each frame across
its own min..max has two failure modes, and both were happening:

- *One pixel decides the image.* A single hot reflection or a dead cold pixel
  crushes everything else into a few levels. Now clipped at the 1st/99th
  percentile; the true extremes still go out on `/thermal/stats`.
- *The range jumps every frame*, so the picture pulses as objects enter and
  leave — which reads as an unstable sensor. Now blended between frames, with a
  **snap** when the scene genuinely changes (>6 °C), so walking into a warm room
  does not fade in over several seconds.

Observed: scene 22.8–35.2 °C, display range 22.9–33.5 °C — the percentile clip
rejecting outliers exactly as intended.

**The exact range travels INSIDE the PNG** as `tEXt` (`fpms_lo_c`, `fpms_hi_c`,
`fpms_min_c`, `fpms_max_c`, `fpms_min_xy`, `fpms_max_xy`, `fpms_palette`). That
is what makes a spot readout possible without shipping the float field: the
browser inverts the colour map exactly. Pairing the image with a separate stats
topic would be off by up to a frame, and `Float32MultiArray` carries no header
to align on.

Raw temperatures are ~102 KB of base64 per frame at 160x120 float32; at 8 fps
that is 800 KB/s through rosbridge. The PNG is ~12 KB.

**Panel side** — a canvas, not an `<img>`:

- hover anywhere for a **spot temperature**
- hottest and coldest pixels ringed and labelled, from coordinates the node
  computed on the real float field rather than guessed from the picture
- a **colour bar** labelled with the display range, so the colours mean something
- smoothing is **off by default** and opt-in: interpolation invents detail the
  160x120 sensor never measured
- ironbow rather than a rainbow map — a rainbow invents edges that are not in
  the data

## Installing the panels permanently

    sudo bash ~/fpms-dev/cortex-rover/install-rover1-panels.sh
    sudo fpms-integrity baseline

After that plain `cortex` opens the Cortex with the three tabs; the shim at
`~/.local/bin/cortex` is removed by the installer.

The installer **verifies everything before writing anything** — a half
installed Cortex that will not start is worse than one without the new tabs:

- every backend file must `ast.parse`
- every web file must parse **as an ES module**, not as a script. `node --check`
  uses the CommonJS goal and misses duplicate-declaration errors in modules;
  that exact gap shipped a broken panel here once already.
- `app.js` is the only pre-existing file replaced, and the install is **refused**
  if the installed copy contains any line the new one drops — i.e. if someone
  else edited it. Its real diff is 12 added lines, 0 removed.

It backs up the whole tree to `/usr/local/lib/fpms/cortex-backup-<stamp>` first,
verifies the result afterwards, and supports `--uninstall`.

## Reconnection and stale feeds

**roslibjs does not resubscribe by itself.** When the socket drops — and it
does; rosbridge logs `WebSocketClosedError: Tried to write to a closed
websocket` — roslibjs neither reconnects nor replays subscriptions. A panel
that subscribed once then goes silent forever *while its last message stays on
screen looking live*. `cortex/web/ros.js` now registers every subscription and
replays them on each reconnect, retrying the socket with backoff to 15 s.

**A stopped feed must look stopped.** The LiDAR plot used to render the last
scan indefinitely at full strength, which is indistinguishable from a live
scanner viewing a static scene — that is how a frozen plot gets trusted. Both
the LiDAR and thermal views now watchdog their own freshness: geometry fades,
and the overlay reads `FEED STOPPED — last scan N.Ns ago`. The last frame is
kept rather than blanked, because it is still the last thing the rover saw.

Verified by killing the ROS stack with a browser attached: live → `FEED
STOPPED — last scan 6.2s ago` → automatic recovery to `ROS 323 pts 9.91 Hz`
with no user action.

## Thermal USB handover

`rover/patches/p3-release-interfaces.patch` fixes the vendored driver's
`disconnect()`, which only did `self.dev = None`. libusb keeps the interface
claim until the process exits, so a service restart had the outgoing process
still holding interface 1 while the incoming one started:

    usbfs: process N (thermal_node) did not claim interface 1 before use

Measured 1 warning per 2 restarts before; **0 across 3 restarts** after.
Interface 1 carries the streaming endpoints (`0x81`/`0x02`) — interface 0 only
has `0x84`/`0x05`, which is why control commands never tripped it and only
streaming did.

## Scanner driver: LDROBOT's own (default) or ours

    ros2 launch fpms_rover1 rover1.launch.py                        # vendor (default)
    ros2 launch fpms_rover1 rover1.launch.py lidar_driver:=builtin  # our d500_node

**The D500 Developer Kit's scanner is an STL-19P.** Neither LDROBOT repo lists
"D500" or "STL-19P" by name, but the STL-19P speaks the **LD19** wire protocol —
which is not a guess: our own LD19 decoder read this exact unit for 436,759
frames at **zero CRC rejects** before the vendor driver was introduced. So the
vendor's `LDLiDAR_LD19` profile at 230400 baud is the right one.

Driver: [ldrobotSensorTeam/ldlidar_ros2](https://github.com/ldrobotSensorTeam/ldlidar_ros2),
built in `~/ldlidar_ws`.

**The vendor driver is better and is now the default.** It publishes the
scanner's *native* resolution — `angle_increment` 0.0125 rad, **~501 rays per
revolution**, which is 5000 measurements/s ÷ 10 Hz. Our `d500_node` bins to 360
and throws that resolution away. It also publishes `/pointcloud2d` and a proper
`base_link → base_laser` transform.

**Our node is kept, because it reports what the vendor's does not:** CRC
rejects and frame counters. That is what proved the serial link was healthy
when the scanner was suspected of freezing. Use `lidar_driver:=builtin` when
you need to see whether the *link* is healthy rather than the map.

We do not use the vendor's `ld19.launch.py` directly: it hardcodes
`/dev/ttyUSB0`, which on this rover is the **Yahboom STM32 board**. `ttyUSBn`
reorders between boots, so ours pins the `by-id` path.

⚠️ The `base_link → base_laser` height of 0.18 m is the **vendor's example
value**, not a measurement of this rover. Fix it before anything navigates off
that transform.

## LiDAR tab: the arena, not a bare polar plot

Rectangular field view showing the driveable arena inside the **wider** object
bounds (the zones deliberately sit outside the arena — that is the geometry,
not a drawing choice), a 500 mm grid, the edge-keep inset, the live scan, and
the markers.

**M1 and M2 are the mission stack's own constants**, carried through verbatim
from `panel_field.py` → `FPMSM2PERFECT/fpms_rover2/geometry.py`, which warns:
*"DO NOT re-measure these into something else. Every failure of the 2026-09-02
session traced to a zone constant that had drifted."*

    M3     112.5, 1357.5   operator: "300 mm to the left of M1"
    M1     412.5, 1357.5   mission stack constant
    LAKE   627.5, 1357.5   operator: "in the middle of both"
    M2     842.5, 1357.5   mission stack constant
    HOME   842.5,  157.5   mission stack constant

Left to right along the arena: **M3, M1, LAKE, M2** — M1 bottom-left and M2
bottom-right as the operator describes them, with the lake between the two.

⚠️ **The lake is geometrically tight.** It sits 215 mm from both M1 and M2, and
a zone is 131 mm in radius, so anything wider than **84 mm radius overlaps
them**. It is drawn at 80 mm. If the real lake is bigger, the three features
genuinely overlap and it is the layout that needs checking, not the drawing.

**Placeholders are drawn dashed with a `?`** and listed as such. A guessed
point must not look surveyed on a map somebody will drive from.

M3 is placed by continuing the exact M1→M2 spacing (430 mm), which puts it at
x = −17.5 — outside the arena, which is *correct* for a zone. A first attempt
at x = 157.5 was wrong and the map showed it: `zone_radius` is 131 mm, so two
zones need 262 mm between centres and 412.5 − 157.5 = 255 made M3 overlap M1.
**Any replacement must clear 262 mm.**

⚠️ **Rover 1 has no localisation.** The scan is drawn from an *assumed* pose at
HOME facing 90°, and the panel says so on the canvas. It is not a tracked
position, and the map must not be treated as one.

Also on the tab: the Control tab's **Refresh rover** action, scan-range presets,
and Scan/Markers visibility toggles.

Note: in the Rover 2 code `M1,M2 = left; M3,M4 = right` refers to **motor
indices** in `set_motor(m1,m2,m3,m4)` — nothing to do with these markers.

## Obstacles are detected, not configured

The rover, the zones, the lake and HOME are **fixed**; obstacles move between
runs. So the panel detects them from the scan rather than taking a coordinate.

Method and constants are **Rover 2's**, from `INTERNATIONALS_ROVER2.py` — a
scan return counts as an obstacle when it is:

- inside the arena,
- **not** within 150 mm of a waypoint (a zone is 131 mm in radius, so 150
  clears the vase with 19 mm to spare — tight on purpose, so an obstacle
  standing close to a zone is still seen), and
- **not** within 30 mm of a wall (those returns *are* the wall).

What survives is clustered at 220 mm (`CLUSTER_RADIUS_MM`), with clusters under
3 points dropped as noise and a cap of 30. Each is drawn as a dashed box and
listed with its centre, extent and point count.

⚠️ Positions inherit the **assumed pose**. Without localisation an obstacle's
arena coordinate is only as good as the guess that the rover is at HOME. Real
obstacle coordinates need odometry — the Yahboom board already publishes
encoders and IMU, so that is buildable.

## Odometry — the map stays still, the rover moves

    /odom               nav_msgs/Odometry
    /rover/arena_pose   PoseStamped, arena MILLIMETRES, for the dashboard
    map -> odom         static, from where the rover was PLACED
    odom -> base_link   the integrated motion
    /reset_odom         std_srvs/Empty

**It integrates the board's velocity, not wheel ticks.** Encoder odometry needs
ticks-per-rev, wheel diameter and track width — *none* measured on Rover 1, and
a guessed constant produces a pose that is confidently wrong, which looks like
drift rather than like a bad number. The Yahboom board already reports its own
vx/vy/wz from its internal kinematics, so no chassis constants are needed.

**Yaw is read, not accumulated.** The board publishes a fused 9-axis attitude.
Integrating an angular rate accumulates error without bound; a fused yaw does
not. Only x and y are integrated.

Measured: **zero drift while stationary** — `x=+0.000 y=+0.000 travelled=0.00m`
— because the board reads a clean 0.0 at rest. `/odom` and `/rover/arena_pose`
both at 20 Hz.

⚠️ This is dead reckoning. It drifts, there is no loop closure, and nothing
corrects it against the LiDAR. The published covariances say so (0.25 on x/y,
0.02 on yaw) rather than claiming precision. It answers "how far has it moved
since it started", not "where is it an hour later".

`start_x_mm` / `start_y_mm` / `start_yaw_deg` are where the rover was **placed**.
Nothing can derive that — it is a human putting a robot down.

The LiDAR panel now draws from the live pose and states which pose is in force:
`pose LIVE (odometry)` or `pose ASSUMED = HOME`. Obstacle positions follow the
live pose too, so they stay put on the map as the rover moves past them.

## Path planning to the M1/M2 midpoint

Grid A* (`cortex/web/planner.js`) replanned twice a second from the *current*
detected obstacles, since obstacles move between runs.

Constants are **Rover 2's**, not invented — `GRID_RES = 50` (its own note: *"was
100: only ~6 cells across a 650mm arena"*) and `ROUTE_CLEARANCE = 90`.

### Three things it refuses to lie about

**The goal is outside the driveable arena.** M1/M2 sit at y=1357.5 against an
arena 1200 deep — by design. A path to that point does not exist and never
will. So the planner drives to a **standoff** and reports the remaining
distance, drawn as an orange dashed leg that is obviously not driveable.

**Goal inside inflation.** Your notes record Nav2 `STATUS_ABORTED` failures from
exactly this. Instead of failing, the goal relocates to the nearest free cell
and the relocation distance is reported.

**Start inside inflation.** The rover is a real object and can legitimately be
standing in inflated space. The start relocates — but the gap is drawn in red
dashes and reported (`start moved: 673 mm`), because an unbroken line from the
rover would claim a route it does not have.

Diagonals require both orthogonals clear, so a path cannot clip the corner of
an inflated obstacle.

⚠️ **The robot radius is unmeasured.** It defaults to 130 mm — Rover 2's own
`ROBOT_RADIUS = ROBOT_W/2`. Deliberately *not* the circumscribed 239 mm: that
is the correct figure for rotating in place, but 239+90 = 329 mm of inflation
on a 1000 mm arena leaves so little free space the planner relocates the start
halfway across the field. Rover 2's file already flags its own 90 mm clearance
as *"too tight for 230mm chassis"*. **Both numbers are provisional until Rover 1
is measured.**

## Path planning, and a lake that places itself

Grid A* (`cortex/web/planner.js`), replanned twice a second from the *current*
detected obstacles. `GRID_RES = 50` and `ROUTE_CLEARANCE = 90` are Rover 2's own
constants (its note: *"was 100: only ~6 cells across a 650mm arena"*).

**Three things the planner refuses to lie about:**

- **The goal is outside the driveable arena.** M1/M2 sit at y=1357.5 against an
  arena 1200 deep, by design — no path there exists. It drives to a standoff
  and reports the remainder as an orange dashed leg.
- **Goal inside inflation** — the exact case your notes record as Nav2
  `STATUS_ABORTED`. The goal relocates to the nearest free cell, distance
  reported.
- **Start inside inflation.** The rover can legitimately be standing in
  inflated space. The start relocates, but the gap is drawn in red dashes and
  reported, because an unbroken line would claim a route that does not exist.

⚠️ Robot radius defaults to **130 mm** — Rover 2's `ROBOT_W/2`, *not* the
circumscribed 239. 239+90 = 329 mm of inflation on a 1000 mm arena leaves so
little free space the start relocates halfway across the field. Both figures
are provisional until Rover 1 is measured; the panel labels them `(unmeasured)`.

### The lake is derived, not stored

It sits **between the two rovers**, so it is not a coordinate — it is the
midpoint of Rover 1's live pose and Rover 2's. It was briefly hardcoded at the
M1/M2 midpoint and that was simply wrong.

Nothing stores it. The panel computes it and shows **nothing** while either
position is unknown, rather than a guessed point sitting on the map looking
authoritative. Rover 2 is read from **its own** rosbridge
(`ws://192.168.137.18:9090`) rather than proxied, so Rover 1 keeps working
while Rover 2 is off — which it has been all session.

The planner targets the lake once derivable and falls back to the M1/M2
midpoint until then, so there is always something to plan to.

## Rover placement, and the lake resolving itself

    Rover 1   157.5, 157.5   bottom-LEFT
    Rover 2   842.5, 157.5   bottom-RIGHT (the mission stack's HOME)
    LAKE      500.0, 157.5   DERIVED — the midpoint of the two

Rover 2 sits at the mission stack's HOME, so Rover 1 bottom-left is its mirror:
1000 − 842.5 = **157.5**, which also clears the 150 mm `edge_keep` by 7.5 mm.
That is now the odometry's `start_x_mm`.

**The lake is still derived, not stored.** A *live* pose from Rover 2's own
rosbridge always wins; its station is only a fallback, and while it is standing
in for a live pose Rover 2 is drawn **dashed with a `?`** — a station is where a
robot was put, not where it is once it moves.

Because the rovers are at the bottom and the zones are the far row they drive
towards, the view now renders low-y at the bottom — start near, goal ahead.
`Flip view` switches to the far-side vantage.

Unlike the M1/M2 midpoint, the lake at (500, 157.5) is **inside** the driveable
arena, so it is a goal the rover can actually reach rather than one it can only
stand off from.

## Verifying a panel actually works

`node --check` uses the **CommonJS** parse goal and accepts things an ES module
rejects. The `SourceTextModule` check fixes that — but **neither catches an
undefined variable**, because that is a runtime `ReferenceError`, not a syntax
error. A panel referencing a variable that no longer exists parses perfectly
and then renders `LiDAR could not load`.

So the real check is loading each tab in a browser and looking at the DOM:

    python3 cdp_tabcheck.py     # per-tab, fresh page each, greps for the error card

Each tab must be checked in its **own fresh page**. Navigating by hash does not
reload the page, so a panel read straight after a hash change can be caught
mid-mount and read as broken when it is fine — that artifact cost a debug cycle
earlier.

## Zone geometry — and a divergence from the mission stack

Operator geometry: *"M3 is 300 mm left of M1"* **and** *"M1 centred between M2
and M3"*. Both solve together with M2 fixed:

    M1 = (M2 + M3)/2  and  M3 = M1 - 300   =>   M1 = M2 - 300

    M3  242.5      M1  542.5      M2  842.5      even 300 mm pitch

300 mm clears the 262 mm two-zone minimum by 38 mm.

**The mission stack was moved to match** on 2026-09-10, so Rover 2 drives to
the same M1 this panel draws:

    ~/rover-backup/FPMSM2PERFECT/fpms_rover2/geometry.py            412.5 -> 542.5
    ~/rover-backup/fpms_rover2-M2-HOME-26MM-20260904/.../geometry.py 412.5 -> 542.5
    cortex/backend/panel_field.py                                   412.5 -> 542.5

Both trees carry a `.bak-<stamp>` beside them, and each edit is annotated in
place. `geometry.py`'s warning — *"DO NOT re-measure these into something
else"* — was honoured to the letter: nothing was **re-measured**. The value was
**changed by instruction**, and it says so, so the next person does not read
542.5 as drift and quietly restore 412.5.

⚠️ **One divergence remains, and it predates this.**
`INTERNATIONALS_ROVER2.py` works in a different frame entirely — its
`TARGET_COORDS` put `m1` at (90, 1110) and `m2` at (880, 1075), **820 mm apart
against this frame's 300**. It *derives* those from zone fractions rather than
storing coordinates, so it cannot simply be set to 542.5 — the derivation would
have to change. The two stacks already disagreed before today.

## Also on the LiDAR tab

- **Scan is clipped to the arena.** Returns beyond it are the room the arena
  stands in — real, but not part of this map, and drawing them made the arena
  look like it had walls where it does not.
- **Both rovers are labelled**, with a `?` while a position is a station rather
  than a live pose.
- **Real dimensions drawn on the plan** — 1000 mm across the bottom, 1200 mm up
  the side — and in the readout as `arena 1000 × 1200 mm`.
- **Mission buttons**: `start`, `stop`, `restart`, `Shutdown R2`.

⚠️ Those command **Rover 2** at 192.168.137.18 over ssh, *not* Rover 1 — and the
panel says so in the UI, not just in a comment. Rover 1 has no mission service
deployed. `Shutdown` is confirm-guarded; the others are not.
