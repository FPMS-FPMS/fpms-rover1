# Fit checks (computed by checks.py from the exported meshes)

**Hard checks: ALL PASS**

## v0 / v1 / v2

| | v0 (original) | v1 | v2 |
|---|---|---|---|
| Length | 206.0 mm | 238.7 mm | 206.5 mm |
| Nose past front wheel (Ø86.6 worst case) | 29.2 mm | 52.9 mm | 19.7 mm |
| Top of body (LiDAR not included) | 137.3 mm | 138.4 mm | 151.5 mm |
| Fill: component volume / bay volume | 59.5% | 53.3% | 50.4% |
| Fill incl. plug, fan-air and tube-bend zones | - | 61.7% | 58.2% |
| Space claimed (convex hull) | 3537.0 cm3 | 3762.5 cm3 | 3660.1 cm3 |
| Weight on front axle | - | 73.7% | 61.2% |

v0 numbers come from the original script; its pumps had no room to bend their tubes and its tank and E-stop zones were not reserved, so its fill is flattered.

## Overlaps (must be zero)

0 overlaps

## Pocket clearance (minimum 0.4 mm per side)

Smallest gap from each component to any printed part, ignoring only the surface it is designed to rest or bolt on.
Motor bosses run in Ø12.9 holes: 0.45 mm radial by construction.

| component | min gap mm | seat (excluded) |
|---|---|---|
| motor_rear_L | 0.44 | gearbox face bolted to the pad |
| motor_rear_R | 0.44 | gearbox face bolted to the pad |
| motor_front_L | 0.44 | gearbox face bolted to the pad |
| motor_front_R | 0.44 | gearbox face bolted to the pad |
| battery_3s2p | 0.44 | rests on its underside |
| orange_pi_n515 | 0.45 | rests on its underside |
| regulator_5v | 0.45 | rests on its underside |
| pump_lower | 0.45 | rests on its underside |
| pump_upper | 0.45 | rests on its underside |
| tank_water | 0.45 | rests on its underside |
| nine_volt_1 | 0.45 | rests on its underside |
| nine_volt_2 | 0.45 | rests on its underside |
| esp32_s3 | 0.45 | rests on its underside |
| xy_mos_1 | 0.45 | rests on its underside |
| xy_mos_2 | 0.45 | rests on its underside |
| servo_mg90s | 0.45 | hangs by its ears |
| yahboom_board | 0.58 | rests on its underside |
| estop_body | 0.6 | panel face on the rear wall |
| lidar_d500 | 0.6 | rests on its underside |
| vl53l0x | 0.6 | lies on the 45 deg slope |

## Hinge sweep (shell + its cap, hatch, deck, components and front wheels rotated about the pin)

- moves freely from -1.75 to 20.0 deg (0.25 deg steps)
- 0 deg catch touches at -2.0 deg; 20 deg stop at 20.25 deg
- smallest gap between tub and shell as printed: 0.396 mm (target 0.4)
- hinge-zone downward overhang steeper than 44 deg: {'main_tub': 0.0, 'front_shell': 0.0} mm2

## Outline and curbs

- length 206.5 mm (limit 210)
- nose reaches X 156.5, 19.7 mm past the Ø86.6 front wheel (limit 20)
- widest point |Y| 74.0 mm (wheels reach 114): nothing sticks out sideways
- nose underside rises at 45 deg from X 119.0, 17.8 mm behind the wheel front
- curb, wheel_d80.0: wheel touches first for every curb up to the axle
- curb, wheel_d86.6: wheel touches first for every curb up to the axle

## Wheels

| wheel | gap to its body mm |
|---|---|
| wheel_rear_L | 2.0 |
| wheel_rear_R | 2.0 |
| wheel_front_L | 2.0 |
| wheel_front_R | 2.0 |

## Bed (Bambu P1S 256 x 256)

- Print 1 (tub + shell assembled): [206.5, 148.0, 123.1] mm
- Print 2 (caps, hatch, decks, shelf, LiDAR plate): [252.5, 253.5, 83.1] mm, fits: True (76.9% of the bed)

## Fill per bay (v2)

| bay | bay cm3 | components cm3 | fill | incl. plug/air/bend zones |
|---|---|---|---|---|
| tub_lower | 434.2 | 122.6 | 28.2% | 28.2% |
| tub_upper | 815.5 | 368.5 | 45.2% | 58.6% |
| shell_lower | 461.1 | 131.3 | 28.5% | 28.5% |
| shell_upper | 605.4 | 400.0 | 66.1% | 85.9% |
| nose | 603.4 | 447.7 | 74.2% | 74.2% |
| TOTAL | 2919.6 | 1470.1 | 50.4% | 58.2% |

Dead volume in the hinge gap: v0 515.8 cm3 -> v2 288.5 cm3

## Weight and balance (estimates marked in meta.json)

- total 2662 g, printed PETG ~680 g (solid-volume estimate; the slicer number replaces it)
- CG at X 57.2 (front axle carries **61.2%**, target 70%), CG height 79.4 mm
- front shell 1514 g with CG at X 104.8: 835.8 N.mm holds it closed about the hinge
- optional ballast (steel shot in epoxy, low in the nose under the battery, room for 129 g): front axle **64.7%**, CG X 60.5 / height 78.1 mm, total 2791 g; 141 g of it reaches 65%

