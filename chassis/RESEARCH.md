# Part research (sources for every number the CAD uses beyond the team's own measurements)

Team measurements win. These fill the gaps and settle the questions the brief raised.

## JGB37-520 gearbox face - SETTLED by two real drawings (no test coupon needed)
- Yahboom 520 encoder motor drawing ("6-M3 depth 5", "7" shaft offset, Ø37/Ø32.8/Ø12 boss/Ø6 shaft):
  https://cdn.shopify.com/s/files/1/0066/9686/1780/files/520Motor_06_2048x2048.jpg
- Yahboom bracket drawing ("6 x Ø3.20 on Ø31"): https://cdn.shopify.com/s/files/1/0066/9686/1780/files/520_motor_parameters_10.jpg
- Pololu 37D drawing (same pattern, "7.0" offset, "6xM3", "Ø31.0"): https://www.pololu.com/file/0J1735/37d-metal-gearmotors-dimension-diagram.pdf
- **The Ø31 circle is centred on the GEARBOX centre, not the shaft.**
- **Holes sit at ±30°, ±90°, ±150° from the shaft direction: the shaft sits between two holes, none on the shaft line.**
  The shaft is 7 below the gearbox centre, so on the side wall the holes are at Z 33.6, 47.0, 60.4 (pairs at X ±7.75, ±15.5, ±7.75).
- Three gearbox assembly screws on the same circle at 180° and ±60° may stand proud -> Ø6 relief pockets in the pad.
- Thread depth differs by source (Pololu 3, team 4, Yahboom 5). Design: 6 mm pad, counterbore 3.4 deep, **M3x6 socket head** -> 3.4 mm engagement.
- Shaft: 15 beyond the boss (21 from the face). Boss Ø12 x 6 = pad thickness, so the boss ends flush with the outer face.

## Wheel - Yahboom 85 mm "sea urchin" off-road wheel
- Drawing: https://cdn.shopify.com/s/files/1/0066/9686/1780/files/Tire-Yahoom-07_2048x2048.jpg
- 86.6 over spikes, 81 tyre body, width 31, Ø52 rim bore, 18 mm hex coupling recessed 4 mm into the wheel.
- **Conflict with the brief (Ø80 x ~38). The CAD keeps the axle at Z 40 (Ø80) as briefed and checks clearance against the worst case Ø86.6 x 38.** Caliper it.

## LDROBOT D500 (LD19 datasheet V2.5)
- https://botland.com.pl/img/art/inne/21991_Karta%20katalogowa.pdf p.7
- 54.00 x 46.29 footprint, 34.8 tall, scan plane ~27.0 above the mounting face.
- 3 x Ø2.5 through (Ø4.46 x 2 counterbore): two holes 46.80 apart, third on the centreline 31.92 from that line.

## Yahboom ROS robot control board V3.0 (YB-ERF01-V3.0)
- https://cdn.shopify.com/s/files/1/0066/9686/1780/files/ROS_robot_board_V3.0_parameters.jpg
- 85 x 56, 4 x Ø2.5 holes on 58 x 49 (3.5 from right/top/bottom edges, 23.5 from the left edge).

## Geekworm N515 (Orange Pi 5 Max case)
- https://geekworm.com/products/n515 - 92.8 x 65 x 35.8, 3010 fan under a round top vent, 2 keyhole slots 65 apart on the long centreline underneath.
- Ports: USB/Ethernet on one short end (the 40 mm plug zone), HDMI/audio/USB-C power on one long side.

## ESP32-S3-DevKitC-1 - https://dl.espressif.com/dl/schematics/esp_idf/DXF_ESP32-S3-DevKitC-1_V1.1_20220429.pdf
- 62.74 x 25.40 PCB, no mounting holes, USB on the short edge opposite the antenna; antenna overhangs ~6 (envelope 69 long).

## Pololu D36V50F5 - https://www.pololu.com/file/0J1732/d36v50fx-step-down-voltage-regulator-dimensions.pdf
- 25.4 x 25.4, 3 x Ø2.18 holes 2.2 from both edges.

## 22 mm E-stop (Schneider Harmony ZB5) - https://static.chipdip.ru/lib/667/DOC029667447.pdf
- Panel hole Ø22.3 (+0.4), anti-rotation notch 3.2 wide to 24.1 across; panel 1-6 thick; body behind ~30 x 40 x 40-48.
