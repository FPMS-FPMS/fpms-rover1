# Rover 1 chassis v3 - assembly

Every part goes in along one straight path. `checks_v3.py` (section "Assembly") sweeps each part along that path, in this order,
against the printed parts and everything installed before it; it must not touch anything (CHECKS.md shows the numbers).
Frame: X forward, Y left, Z up. "Down" = straight in from above with the caps and hatches off.

## 0. Before the big print
1. Print `stl/FPMS_Rover1_TestCoupon.stl` (< 1 h). It is a 20 mm slice of the real hinge (two cups + fused pin, the knuckle
   between them, 0.4 mm gap) and one motor pad (6 x M3 counterbored on Ø31, Ø12.9 boss hole).
   - The hinge slice must break free by hand (see step 1) and swing smoothly.
   - A JGB37 must bolt to the pad with 6 x M3x6 and its boss must enter the Ø12.9 hole without force.
   If either fails, change `GAP` / `CLR` in `fpms_rover1_v3.py` before printing Print 1.

## 1. Free the hinge (Print 1 comes off the bed in one piece)
1. Let it cool fully on the plate. Do not lever the shell against the tub.
2. Hold the tub, push the nose DOWN gently (towards the 0 deg catch), then lift it UP to the 20 deg stop, 10 times.
   The first movement breaks the few strands across the 0.4 mm gap.
3. If it is stiff: run a 0.3 mm feeler / hobby blade around the knuckle faces at |Y| 42-62 and the cup faces at 30-42 and 62-74.
   Never pry at the pin.
4. Check the AS5600 magnet lug (Y 10-20, on the axis) turns freely in its pocket too.

## 2. Tub (dry side)
| # | part | path | notes |
|---|---|---|---|
| 1 | Pololu 5 V regulator | down into its frame ahead of the left rear motor | |
| 2 | AS5600 board | down into its slot (Y 23, chip facing -Y, toward the magnet) | magnet 6 x 2.5 diametric glued into the shell lug first (CA glue, flush) |
| 3 | rear motors | sideways through the walls, 6 x M3x6 each; zip tie through the tunnel at the encoder end | PH2.0 plugs toward the centre channel |
| 4 | Pi deck + Orange Pi (N515) | down onto the ledges / posts | plug zone faces left (+Y) |
| 5 | board shelf + Yahboom board | down; board on 4 x M2.5 nylon spacers | |
| 6 | E-stop (main switch) | body down to the hole, then back (-X) through the Ø22.4 hole; nut on the inside | the guard fins are printed on |
| 7 | DC-099 charge jack | from outside forward (+X) through the Ø11.2 hole, nut inside | above the board, Y -30 |

## 3. Shell (wet side)
| # | part | path | notes |
|---|---|---|---|
| 1 | front motors | as the rear | |
| 2 | XY-MOS x2 | down into their holders behind the front motors | they end up under the sealed pump-deck tray |
| 3 | pump deck (tray) | down onto its ledges | open front edge spills into the nose drains |
| 4 | lower pump (SPRAY) | down onto the tray, barbs to the right (-Y), barbs one above the other | strap 1 |
| 5 | upper pump (REFILL) | down onto the lower pump, barbs right | strap 2 round both |
| 6 | refill servo (MG90S) | down into the bay 5 mm off the wall, then sideways (-Y) so the spline enters the wall hole | cut the FRONT ear off first; rear ear: 1 x M2x8 into the post |
| 7 | horn + refill arm | from outside onto the spline (+Y), horn screw | set the servo to the stowed angle (arm up) before fitting |
| 8 | ESP32 | down between the two C-rails, USB end up | the open slot in the shell cap is over its USB port |
| 9 | tank | down into its slot, fittings to the right | tubes go on after |
| 10 | battery | down into the left slot, lead to the rear | |
| 11 | nozzle (Luer adapter + 14 ga tip) | down into the front-wall cradle, needle through the hole | one zip tie through the cradle |
| 12 | tubes | see "Water" | |
| 13 | caps, tank hatch, battery hatch (9 V cells in its cradle) | down | cap: 3 x M3x8; tub cap: 4 x M3x8 |
| 14 | LiDAR riser + D500 + RGB strip | riser 4 x M3x10 into the cap pads; LiDAR 3 x M2x6 from above | strip into the riser recess, lead down the centre hole |

## Water (6x9 silicone; lengths and routes from `tubes_v3.json`, R >= 25 everywhere - see CHECKS.md "Tubes")
- **Spray out** (lower pump, LOWER barb) -> up the right wall column -> over the upper pump -> turns forward at Y 0 -> nozzle.
- **Spray in** (tank fitting at the back of the tank top) -> lower pump, UPPER barb.
- **Refill in** (strainer on the arm tip -> along the arm, in its groove -> loop -> right-wall window) -> upper pump, LOWER barb.
- **Refill out** (upper pump, UPPER barb) -> tank fitting at the front of the tank top.
- Cut each tube to its listed length + 10 mm; push 10 mm onto every barb.

## Cables
- Hinge crossings (all through the cable pass on the centreline at the hinge axis, tied to the strain-relief bridges either side):
  battery lead (XT60), switched 12 V to the XY-MOS (XT30), charge lead (XT30), both front motor leads (PH2.0-6P), ESP32 USB.
  Pass each plug through on its own; the pass is checked to take the biggest (XT60).
- Shell only (no crossing): VL53L0X, servo, XY-MOS gates, 9 V cells -> ESP32.
- Tub only: LiDAR (through the riser and the lid hole to the Pi), AS5600, RGB strip -> Yahboom board, E-stop, charge jack.
- Leave about 15 mm of slack in the bundle at the pass: at 20 deg the bundle's length changes by the amount in CHECKS.md, and it twists rather than stretches.
