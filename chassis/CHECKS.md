# Fit checks (computed by checks.py from the exported meshes)

**Hard checks: ALL PASS**

## Overlaps (must be zero)

0 overlaps

## Component clearance to the nearest printed part (support face excluded)

| component | min gap mm |
|---|---|
| yahboom_board | 0.0 |
| estop_body | 0.0 |
| regulator_5v | 0.0 |
| motor_rear_L | 0.01 |
| motor_rear_R | 0.01 |
| motor_front_L | 0.01 |
| motor_front_R | 0.01 |
| xy_mos_1 | 0.01 |
| xy_mos_2 | 0.01 |
| servo_mg90s | 0.01 |
| tank_water | 0.02 |
| vl53l0x | 0.02 |
| orange_pi_n515 | 0.03 |
| battery_3s2p | 0.03 |
| esp32_s3 | 0.03 |
| nine_volt_1 | 0.04 |
| lidar_d500 | 0.07 |
| pump_1 | 0.09 |
| pump_2 | 0.09 |
| nine_volt_2 | 0.5 |

## Hinge sweep (shell + its lid, deck, components and front wheels rotated about the pin)

- moves freely from -1.5 to 20.25 deg (0.25 deg steps)
- nose-down stop (0 deg catch) touches at -1.75 deg; nose-up stop at 20.5 deg
- smallest gap between tub and shell as printed: 0.396 mm (target 0.4)
- hinge-zone downward overhang steeper than 44 deg: {'main_tub': 0.0, 'front_shell': 0.0} mm2

## Wheels

| wheel | gap to its body mm |
|---|---|
| wheel_rear_L | 2.0 |
| wheel_rear_R | 2.0 |
| wheel_front_L | 2.0 |
| wheel_front_R | 2.0 |

## Bed (Bambu P1S 256 x 256 x 256)

- Print 1 (tub + shell assembled): [238.7, 162.6, 110.0] mm
- Print 2 (lids, decks, LiDAR mount, hatch): packed footprint [234.0, 253.8, 20.1] mm, fits: True (parts cover 76.8% of the bed)

## Fill per bay

| bay | v0 bay cm3 | v0 fill | v1 bay cm3 | v1 fill | v1 incl. plug/air/bend zones |
|---|---|---|---|---|---|
| tub_lower | 364.5 | 33.6% | 432.5 | 28.3% | 28.3% |
| tub_upper | 644.7 | 56.8% | 672.8 | 54.5% | 70.7% |
| shell_lower | 300.4 | 38.8% | 440.5 | 29.6% | 29.6% |
| shell_upper | 531.4 | 61.2% | 605.8 | 66.0% | 85.8% |
| nose | 616.1 | 85.7% | 585.4 | 75.2% | 75.2% |
| TOTAL | 2457.1 | 59.4% | 2737.0 | 53.3% | 61.7% |

Dead volume in the hinge gap: v0 515.8 cm3 -> v1 168.1 cm3
Space the whole chassis claims (convex hull, body + lids): {'v0': np.float64(3537.0), 'v1': np.float64(3772.7)}

## Weight and balance (estimates marked in meta.json)

- total 2650 g, printed PETG ~668 g (solid-volume estimate; the slicer number replaces it)
- CG at X 69.0 (front axle carries **73.8%**, target 70%), CG height 69.5 mm
- front shell 1534 g with CG at X 122.6: 1115.4 N.mm holds it closed about the hinge

