# Rover 1 chassis v3 - hardware to buy

Quantities are for one robot. Prices are not listed: none could be confirmed online, so check before ordering.
Sources are Canadian or ship to Canada: Amazon.ca, Digi-Key.ca, RobotShop.ca, McMaster-Carr, Canadian Tire, Princess Auto.
**Measure before printing** (these set CAD numbers): R385 barb spacing (assumed 14 mm, barbs one above the other once the
pump is turned on its axis); tank cap/fittings (assumed 2 x 90 deg elbows on the top face, 17 mm tall); which face of each
9 V cell has its USB port; the ESP32 board's connector type (the official DevKitC-1 is micro-USB); MG90S ear position.

## Screws and fixings
| item | qty | where | source |
|---|---|---|---|
| M3 x 6 socket head (SHCS) | 24 | 6 per JGB37 motor, heads flush in the counterbores | Amazon.ca M3 SHCS assortment / Digi-Key |
| M3 x 8 socket head | 13 | tub cap 6 (4 side-wall bosses + 2 on the rear hatch, which they clamp), shell cap 2, battery hatch 1 (boss behind the battery bay), tub deck 2 (self-tap into the two deck posts), spare 2 | same kit |
| M3 x 10 socket head | 4 | LiDAR riser into the tub-cap pads | same kit |
| M2 x 6 socket or pan head | 3 | D500 LiDAR to the riser (its holes are Ø2.5 with Ø4.46 counterbores: M2, not M2.5) | Amazon.ca M2 kit |
| M2 x 8 | 1 | refill servo rear ear into its post | same |
| M2.5 x 8 + 2 mm nylon spacer + nut | 4 | Yahboom board on the shelf | Amazon.ca M2.5 nylon standoff kit |
| M3 x 8 | 2 | Pi (N515) to the tub deck | M3 kit |
| zip ties 2.5 x 100 mm | 15 | 4 motor tunnels, 4 strain-relief bridges, 1 nozzle cradle, 3 hose clips on the arm, spare | Canadian Tire |
| zip ties 3.6 x 200 mm | 2 | the two straps round the pump stack | Canadian Tire |

## Sensors and electrics
| item | qty | notes | source |
|---|---|---|---|
| AS5600 magnetic angle module, 23 x 23 | 1 | slides into its tub slot, chip facing the magnet | Amazon.ca / RobotShop |
| magnet 6 x 2.5 mm, **diametrically** magnetised NdFeB | 1 (+1 spare) | glued flush in the shell lug on the hinge axis; gap to the chip 1.0 mm (datasheet 0.5-3) | often included with the AS5600; else Amazon.ca |
| DC-099 metal panel jack, 5.5 x 2.1 mm, 5 A | 1 | charge inlet, rear wall (Ø11.2 hole, nut inside) | Digi-Key.ca / Tayda / Amazon.ca |
| 12.6 V 2 A Li-ion CC/CV charger, 5.5 x **2.1** centre-positive plug | 1 | the 3S2P pack must have a 3S BMS | Amazon.ca |
| main switch | 0 | none: the E-stop is the main switch (latching; press = off, twist = on) | - |
| XT60 pair | 1 | battery main lead across the hinge | RobotShop / Amazon.ca |
| XT30 pairs | 2 | switched 12 V to the XY-MOS, charge lead | RobotShop / Amazon.ca |
| silicone wire 14 AWG red/black | 0.5 m each | battery lead | Amazon.ca |
| silicone wire 18 AWG red/black | 1 m each | 12 V to pumps, charge lead | Amazon.ca |
| Yahboom RGB bar, 14-LED flexible (97 x 12) | 1 | in the LiDAR-riser recess (assumed this variant) | Yahboom store |

## Water
| item | qty | notes | source |
|---|---|---|---|
| silicone tube 6 x 9 mm | 1.2 m | cut lengths below (+10 mm per barb end) | Amazon.ca |
| female Luer lock to 1/4" barb adapter | 1 (+1) | the nozzle body | Avantor CA (Masterflex/Value Plastics) / Amazon.ca kit |
| blunt Luer-lock dispensing tip 14 ga, 1/4" long, straight | 2 | the stream orifice (McMaster 75165A117 class); 16 ga = narrower, faster stream | McMaster / Amazon.ca kit |
| 90 deg barbed elbow, 1/4" (6 mm) | 3 | 2 on the tank top (assumed - match your tank cap), 1 on the spray pump's inlet barb (turns the inlet forward so S-in can reach the tank) | Amazon.ca / McMaster |
| intake strainer, 6 mm barb (aquarium pre-filter or mesh) | 1 | on the hose at the arm tip; parks in the strainer clip when stowed | Amazon.ca / pet shop |
| spring kink guard for 9 mm OD tube, ~30 mm | 5 | one per tube at its tightest bend (CHECKS.md WARN lines: R20.9-23.2 against a R25 target) and one on the intake hose's free loop | McMaster / Amazon.ca |

## Tube lengths (from the routed centrelines in tubes_v3.json; checks_v3.py re-measures them)
| tube | from -> to | routed length mm | cut length mm (+10 per barb end) | tightest bend |
|---|---|---|---|---|
| S-out | spray pump outlet (lower barb) -> nozzle barb | 160 | 180 | R22.3 |
| R-out | refill pump outlet -> tank fitting (rear) | 87 | 107 | R22.4 |
| S-in | tank fitting (front) -> elbow on the spray pump inlet | 89 | 109 | R23.2 |
| intake hose + R-in (ONE piece) | strainer barb at the arm tip -> clipped along the arm to 8 mm from the pivot -> 65 mm free loop -> wall gland (X 124, Z 116) -> refill pump inlet | ~152 outside + 53 inside = 205 | 225 | R20.5 (free loop, at both ends of the swing) / R20.9 (inside) |

Total silicone 6 x 9: about 620 mm; buy 1.5 m (spare for a re-cut).

## Printed (no purchase)
Print 1 (tub + shell, hinge in place), Print 2 (caps, hatches, decks, shelf), Print 3 (LiDAR riser, refill arm, ESP32 carrier,
4 coupling spacers 8 mm), Test coupon. See CHECKS.md for the bed layouts and the slicer numbers in PROGRESS.md.
