# v3 fit checks (computed by checks_v3.py from the exported meshes)

**Hard checks: ALL PASS**

- WARN: intake hose stowed@0: tightest bend R20.6 (target 25) - kink guard at the loop
- WARN: intake hose stowed@20: tightest bend R20.6 (target 25) - kink guard at the loop
- WARN: intake hose swing-50@0: tightest bend R23.4 (target 25) - kink guard at the loop
- WARN: intake hose swing-50@20: tightest bend R23.4 (target 25) - kink guard at the loop
- WARN: intake hose deployed@0: tightest bend R20.4 (target 25) - kink guard at the loop
- WARN: intake hose deployed@20: tightest bend R20.4 (target 25) - kink guard at the loop
- WARN: tube S-out: tightest smoothed bend R22.3 (target 25) - fit a spring kink guard there
- WARN: tube R-out: tightest smoothed bend R22.4 (target 25) - fit a spring kink guard there
- WARN: tube S-in: tightest smoothed bend R23.2 (target 25) - fit a spring kink guard there
- WARN: tube R-in: tightest smoothed bend R20.9 (target 25) - fit a spring kink guard there

## v0 / v1 / v2 / v3

| | v0 (original) | v1 | v2 (approved) | v3 |
|---|---|---|---|---|
| Length | 206.0 mm | 238.7 mm | 206.5 mm | 206.5 mm |
| Nose past front wheel (Ø86.6 worst case) | 29.2 mm | 52.9 mm | 19.7 mm | 19.7 mm |
| Top of body (LiDAR not included) | 137.3 mm | 138.4 mm | 151.5 mm | 163.0 mm |
| Fill: component volume / bay volume | 59.0% | 53.3% | 50.4% | 46.1% |
| Fill incl. plug, fan-air and tube-bend zones | - | 61.7% | 58.2% | 49.5% |
| Space claimed (convex hull) | 3537.0 cm3 | 3762.5 cm3 | 3660.1 cm3 | 4005.9 cm3 |
| Weight on front axle | - | 73.7% | 61.2% | 61.1% |

v0 numbers come from the original script; its pumps had no room to bend their tubes and its tank and E-stop zones were not reserved, so its fill is flattered.

## Overlaps (must be zero)

0 overlaps

## Pocket clearance (minimum 0.4 mm per side)

Smallest gap from each component to any printed part, ignoring only the surface it is designed to rest or bolt on.
Motor bosses run in Ø12.9 holes: 0.45 mm radial by construction.

| component | min gap mm | seat (excluded) |
|---|---|---|
| battery_3s2p | 0.4 at [153.6, 41.9, 66.2] | rests on its underside |
| motor_rear_L | 0.44 at [6.8, 64.6, 29.8] | gearbox face bolted to the pad |
| motor_rear_R | 0.44 at [6.8, -49.9, 29.8] | gearbox face bolted to the pad |
| motor_front_L | 0.44 at [100.3, 64.6, 29.8] | gearbox face bolted to the pad |
| motor_front_R | 0.44 at [100.3, -49.9, 29.8] | gearbox face bolted to the pad |
| orange_pi_n515 | 0.45 at [-30.7, -65.6, 70.2] | rests on its underside |
| regulator_5v | 0.45 at [30.2, 67.4, 35.6] | rests on its underside |
| pump_lower | 0.45 at [101.6, -7.1, 105.3] | rests on its underside |
| pump_upper | 0.45 at [101.6, -7.1, 145.3] | rests on its underside |
| tank_water | 0.45 at [123.8, 4.0, 95.0] | rests on its underside |
| nine_volt_1 | 0.45 at [132.2, 8.1, 157.0] | held by rails / posts / cradle (contact boxes excluded) |
| esp32_s3 | 0.45 at [153.6, -57.1, 129.6] | held by rails / posts / cradle (contact boxes excluded) |
| xy_mos_1 | 0.45 at [76.5, -36.8, 34.0] | rests on its underside |
| xy_mos_2 | 0.45 at [76.5, 18.2, 34.0] | rests on its underside |
| nozzle_luer | 0.45 at [120.0, 4.4, 150.5] | held by rails / posts / cradle (contact boxes excluded) |
| tube_r_in | 0.45 at [106.4, -71.2, 114.9] |  |
| tube_s_out | 0.46 at [76.8, -71.1, 115.0] |  |
| yahboom_board | 0.58 at [9.1, -46.3, 139.1] | rests on its underside |
| estop_body | 0.6 at [-47.0, 24.0, 113.2] | panel face on the rear wall |
| vl53l0x | 0.6 at [138.4, -10.2, 51.6] | lies on the 45 deg slope |
| nine_volt_2 | 0.61 at [135.1, 63.5, 142.4] | held by rails / posts / cradle (contact boxes excluded) |
| servo_mg90s | 0.61 at [153.1, -53.8, 64.8] | held by rails / posts / cradle (contact boxes excluded) |
| charge_jack_dc099 | 0.63 at [-47.0, -35.9, 146.3] | panel face on the rear wall |
| tube_s_in | 0.68 at [102.3, -47.5, 94.5] |  |
| tube_r_out | 0.72 at [101.9, -62.2, 142.5] |  |

## Hinge sweep (shell + its cap, hatch, deck, components and front wheels rotated about the pin)

- moves freely from -1.75 to 20.0 deg (0.25 deg steps)
- 0 deg catch touches at -2.0 deg; 20 deg stop at 20.25 deg
- smallest gap between tub and shell as printed: 0.396 mm (target 0.4)
- hinge-zone downward overhang steeper than 44 deg: {'main_tub': 0.0, 'front_shell': 0.0} mm2

## Outline and curbs

- length 206.5 mm (limit 210)
- nose reaches X 156.5, 19.7 mm past the Ø86.6 front wheel (limit 20)
- widest point |Y| 91.5 mm (wheels reach 114): nothing sticks out sideways
- nose underside rises at 45 deg from X 119.0, 17.8 mm behind the wheel front
- curb, wheel_d80.0: wheel touches first for every curb up to the axle
- curb, wheel_d86.6: wheel touches first for every curb up to the axle

## Wheels

| wheel | gap to its body mm |
|---|---|
| wheel_rear_L | 4.0 |
| wheel_rear_R | 4.0 |
| wheel_front_L | 4.0 |
| wheel_front_R | 4.0 |

## Bed (Bambu P1S 256 x 256)

- Print 1 (tub + shell assembled): [206.5, 148.0, 134.6] mm
- Print 3 (rear hatch, tank hatch, LiDAR riser, refill arm, ESP32 carrier, 4 coupling spacers): [216.0, 148.0, 98.1] mm
- Print 2 (caps, hatches, decks, shelf): [233.5, 252.1, 94.6] mm, fits: True (75.2% of the bed)

## Fill per bay (v3)

| bay | bay cm3 | components cm3 | fill | incl. plug/air/bend zones |
|---|---|---|---|---|
| tub_lower | 434.2 | 123.5 | 28.4% | 28.4% |
| tub_upper | 900.2 | 370.3 | 41.1% | 53.3% |
| shell_lower | 461.1 | 131.3 | 28.5% | 28.5% |
| shell_upper | 689.2 | 400.0 | 58.0% | 58.0% |
| nose | 688.3 | 437.9 | 63.6% | 63.6% |
| TOTAL | 3173.0 | 1463.0 | 46.1% | 49.5% |

Dead volume in the hinge gap: v0 515.8 cm3 -> v3 356.2 cm3

## Weight and balance (estimates marked in meta.json)

- total 2761 g, printed PETG ~739 g (solid-volume estimate; the slicer number replaces it)
- CG at X 57.1 (front axle carries **61.1%**, target 70%), CG height 83.7 mm
- front shell 1567 g with CG at X 105.4: 874.4 N.mm holds it closed about the hinge

## LiDAR clear of the lifted shell (B5)

- scan plane Z 214.0 (riser 20.0 mm); shell top 163.0 at 0 deg, 190.26 at 20 deg
- margin at 20 deg **23.74 mm** (need >= 15.0: 10 + 5.0 mm allowed for beam height, not published)

## Wet zone (A3)

| electronic part | bottom Z | wet floor + 5 | nearest water mm | exposed pts | under sealed tray | ok |
|---|---|---|---|---|---|---|
| battery_3s2p | 65.5 | 33.4 | 10.6 | 0 | False | True |
| nine_volt_1 | 141.8 | 33.4 | 1.2 | 0 | False | True |
| nine_volt_2 | 141.8 | 33.4 | 30.1 | 0 | False | True |
| esp32_s3 | 78.9 | 33.4 | 11.2 | 0 | False | True |
| xy_mos_1 | 28.4 | 33.4 | 30.1 | 0 | True | True |
| xy_mos_2 | 28.4 | 33.4 | 56.0 | 0 | True | True |
| servo_mg90s | 64.2 | 33.4 | 33.4 | 0 | False | True |
| vl53l0x | 42.3 | 33.4 | 59.6 | 0 | False | True |

Drains Ø3.5: shell [(100.0, 0.0), (116.0, -60.0), (116.0, -16.0), (116.0, 40.0), (116.0, 62.0)], tub [(-17, 0), (38, 55), (38, -55)]. Exposed = a sample point with an unobstructed straight line to water within 15 mm that is level with or above it.

## Cables across the hinge (A4)

Pass 333.0 mm2, bundle 60.2 mm2 (**18.1%** fill, limit 40); bundle 6.1 mm from the hinge axis -> length change 2.1 mm at 20 deg (they twist, not stretch).

| cable | conductors | wire OD | plug passes | route |
|---|---|---|---|---|
| battery main lead | 2 | 3.6 | True | battery (shell) -> E-stop / main switch (tub) |
| switched 12 V to pumps + XY-MOS | 2 | 2.3 | True | E-stop (tub) -> XY-MOS (shell), XT30 |
| charge lead | 2 | 2.3 | True | DC-099 jack (tub) -> battery BMS (shell), XT30 |
| front motor L | 6 | 1.2 | True | Yahboom board (tub) -> JGB37 front L, PH2.0-6P |
| front motor R | 6 | 1.2 | True | Yahboom board (tub) -> JGB37 front R, PH2.0-6P |
| ESP32 USB | 1 | 3.5 | True | ESP32 (shell) -> Orange Pi (tub), micro-USB/USB-C |

- VL53L0X, servo, XY-MOS gates, 9 V cells: all on the shell side, wired to the ESP32 there
- AS5600: board on the tub, magnet on the shell - no wire crosses
- LiDAR lead: lid hole under the riser -> Pi, all in the tub
- RGB bar: riser -> Yahboom board (tub)

## Refill arm and intake hose (A2)

| arm @ hinge deg | front wheel gap | rear wheel gap | lowest Z |
|---|---|---|---|
| refill_arm_stowed@0 | 10.39 | 30.0 | 62.8 |
| refill_arm_stowed@20 | 10.39 | 30.0 | 94.3 |
| refill_arm_deployed@0 | 10.76 | 30.0 | 0.97 |
| refill_arm_deployed@20 | 10.76 | 30.0 | 52.49 |

| hose outside @ hinge deg | wheel gap | lowest Z | min bend R | length |
|---|---|---|---|---|
| stowed@0 | 17.7 | 72.6 | 20.6 | 149.0 |
| stowed@20 | 17.7 | 104.3 | 20.6 | 149.0 |
| swing-50@0 | 18.9 | 70.6 | 23.4 | 149.3 |
| swing-50@20 | 18.9 | 103.4 | 23.4 | 149.3 |
| swing-10@0 | 21.1 | 67.2 | 27.7 | 149.0 |
| swing-10@20 | 21.1 | 101.7 | 27.7 | 149.0 |
| swing+25@0 | 18.6 | 31.8 | 26.1 | 151.6 |
| swing+25@20 | 18.6 | 90.7 | 26.1 | 151.6 |
| deployed@0 | 15.1 | 1.7 | 20.4 | 151.3 |
| deployed@20 | 15.1 | 54.1 | 20.4 | 151.3 |

- one hose for every arm angle: its length varies by 2.6 mm over the swing (limit 4); the loop changes shape, not length

## Spray nozzle (A1)

- tip [155.9, 0.0, 142.7], 15.0 deg down, inside the outline: True
- 4 deg half-angle cone, 600 mm: blocked by nothing; straight line reaches the ground at X ~688.0

## Tubes (6x9 silicone, R25 minimum)

| tube | length mm | min bend radius mm |
|---|---|---|
| S-out | 160.0 | 22.3 |
| R-out | 87.0 | 22.4 |
| S-in | 89.0 | 23.2 |
| R-in | 53.0 | 20.9 |

## AS5600 hinge sensor (B9)

- magnet face to chip 1.0 mm (datasheet 0.5-3 mm (aim ~1)), magnet on the hinge axis

## ESP32 (B6)

- C-rails on both long edges, from the ledge to 50.0 of 68.8 mm height; USB end up, under an open slot in the shell cap: reachable with nothing removed

## Assembly: a straight path out for every part (reverse = its way in), in ASSEMBLY.md order

| part | straight path out (unit vector) | travel mm | note |
|---|---|---|---|
| regulator_5v | (-0.33, -0.09, 0.94) | 127.3 | first 9.0 mm along (0.0, 0.0, 1.0) |
| as5600_board | (0.0, 0.0, -1.0) | 27.5 |  |
| xy_mos_1 | (0.0, 0.0, 1.0) | 128.6 |  |
| xy_mos_2 | (0.0, 0.0, 1.0) | 128.6 |  |
| tub_deck | (-1.0, 0.0, 0.0) | 78.8 | with orange_pi_n515 screwed on |
| tub_shelf | (-1.0, 0.0, 0.0) | 69.4 | with yahboom_board screwed on |
| pump_deck | (0.0, 0.0, 1.0) | 91.0 |  |
| pump_lower | (0.0, 0.0, 1.0) | 88.6 |  |
| pump_upper | (0.0, 0.0, 1.0) | 48.6 |  |
| servo_mg90s | (0.0, 0.0, 1.0) | 92.8 | first 4.5 mm along (0.0, 1.0, 0.0) |
| esp_carrier | (0.0, 0.0, 1.0) | 80.6 |  |
| esp32_s3 | (0.0, 0.0, 1.0) | 78.1 |  |
| tank_water | (0.0, 0.0, 1.0) | 96.6 |  |
| battery_3s2p | (0.0, 0.0, 1.0) | 91.5 |  |
| nozzle_luer | (-0.33, -0.09, 0.94) | 16.1 | first 8.0 mm along (-0.97, 0.0, 0.26) |
| rear_hatch | (0.0, 0.0, 1.0) | 91.0 | first 3.5 mm along (0.0, 0.0, 1.0) |

