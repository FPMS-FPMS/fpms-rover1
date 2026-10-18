# Rover 1 chassis v3 - assembly

Every part has one straight way out of the body, and its way in is the same path reversed. `checks_v3.py` (section
"Assembly" in CHECKS.md) proves it: it sweeps each part out along that path, in the reverse of the order below, against the
printed parts present at that step and everything fitted before it, and fails if anything touches. A few parts need one
short move first (lift off a holder, spline out of the wall); those are listed as two moves.
Frame: X forward, Y left, Z up. "Down" = straight in from above with the caps and hatches off.

## 0. Before the big print
1. Print `stl/FPMS_Rover1_TestCoupon.stl` (48 min, 17.7 g, no supports). It is a 20 mm slice of the real hinge (two cups +
   fused pin, the knuckle between them, 0.4 mm gap) and one motor pad (6 x M3 counterbored on Ø31, Ø12.9 boss hole).
   - The hinge slice must break free by hand (see step 1) and swing smoothly.
   - A JGB37 must bolt to the pad with 6 x M3x6 and its boss must enter the Ø12.9 hole without force.
   If either fails, change `GAP` / `CLR` in `fpms_rover1_v3.py` before printing Print 1.

## 1. Free the hinge (Print 1 comes off the bed in one piece)
1. Let it cool fully on the plate. Do not lever the shell against the tub.
2. Hold the tub, push the nose DOWN gently (towards the 0 deg catch), then lift it UP to the 20 deg stop, 10 times.
   The first movement breaks the few strands across the 0.4 mm gap.
3. If it is stiff: run a 0.3 mm feeler / hobby blade around the knuckle faces at |Y| 42-62 and the cup faces at 30-42 and 62-74.
   Never pry at the pin.
4. Check the AS5600 magnet lug (on the axis) turns freely in its pocket too.

## 2. On the bench first
- **Pi deck + Orange Pi:** Pi onto the deck, 2 x M3x8. They go into the tub as one unit.
- **Board shelf + Yahboom board:** board onto the shelf on 4 x M2.5 nylon spacers. One unit.
- **Rear hatch:** E-stop (the main switch) through its Ø22.4 hole from outside, nut inside; DC-099 charge jack through its
  Ø11.2 hole, nut inside. Solder / plug their leads now, with enough length to reach the board shelf.
- **Magnet:** 6 x 2.5 diametric magnet glued flush into the shell's hinge lug (CA glue).

## 3. Tub (dry side). The rear hatch is OFF: the tub is open at the back above the deck
| # | part | way in | notes |
|---|---|---|---|
| 1 | Pololu 5 V regulator | in from above, tilted 20 deg back along the rake, then the last 9 mm straight down into its frame | frame is ahead of the left rear motor |
| 2 | AS5600 board | UP from underneath through the belly slot under the hinge, chip facing the magnet | the slot also drains |
| 3 | rear motors | sideways through the walls, 6 x M3x6 each; zip tie through the tunnel at the encoder end | PH2.0 plugs toward the centre channel |
| 4 | Pi deck + Pi (one unit) | slide in FORWARD (+X) through the open rear, flat on the two posts; 2 x M3x8 self-tap down into the posts | the Pi's top front corner passes 1.6 mm under the raked front wall: keep it level |
| 5 | shelf + Yahboom board (one unit) | slide in FORWARD through the open rear onto the right-wall ledge and the front gusset | |
| 6 | rear hatch (E-stop + jack on it) | down from above, the last 3.5 mm drops its lip inside the rear-wall stub | the tub cap's two rear screws go into its bosses and clamp it |

## 4. Shell (wet side)
| # | part | way in | notes |
|---|---|---|---|
| 1 | front motors | as the rear | |
| 2 | XY-MOS x2 | down into their holders behind the front motors | they end up under the sealed pump-deck tray |
| 3 | pump deck (tray) | down onto its ledges | open front edge spills into the nose drains; notched round two cap bosses |
| 4 | lower pump (SPRAY) | down onto the tray, barbs to the right (-Y), one above the other | 90 deg elbow on its UPPER (inlet) barb, pointing forward. Strap 1 |
| 5 | upper pump (REFILL) | down onto the lower pump, barbs right | strap 2 round both |
| 6 | refill servo (MG90S) | down into the bay, then 4.5 mm sideways (-Y) so the spline enters the wall hole; the spline slot above the hole lets it drop in | cut the FRONT ear off first; rear ear: 1 x M2x8 into the post |
| 7 | horn + refill arm | from outside onto the spline (+Y), horn screw | set the servo to the stowed angle (arm up) before fitting |
| 8 | ESP32 carrier | down; it rests on the servo's top and is located by the side wall and the bay/tank wall | |
| 9 | ESP32 | down between the carrier's two C-rails, USB end up | the open slot in the shell cap is over its USB port |
| 10 | tank | down into its slot, fittings to the right | tubes go on after |
| 11 | battery | down into the left slot, lead to the rear | |
| 12 | nozzle (Luer adapter + 14 ga tip) | lower it in tilted (front end first), then push it 8 mm FORWARD along its own axis so the tip goes out through the front-wall hole | one zip tie through the cradle; push S-out onto its barb before fitting |
| 13 | tubes | see "Water" | |
| 14 | shell cap, tank hatch, battery hatch (9 V cells in its cradle) | down | shell cap 2 x M3x8; battery hatch 1 x M3x8 into the boss behind the battery bay (its tab reaches back over it) |
| 15 | tub cap | down | 6 x M3x8: 4 into the side-wall bosses, 2 into the rear hatch |
| 16 | LiDAR riser + D500 + RGB strip | riser 4 x M3x10 into the cap pads; LiDAR 3 x M2x6 from above | strip into the riser recess, lead down the centre hole |

## Water (6x9 silicone; routes from `tubes_v3.json`, lengths and bends re-measured in CHECKS.md "Tubes")
- **Spray out** (lower pump, LOWER barb) -> up the right side -> over the upper pump -> forward at Y 0 -> nozzle barb. 160 mm.
- **Spray in** (tank fitting at the BACK of the tank top) -> elbow on the lower pump's UPPER barb. 89 mm.
- **Refill out** (upper pump, UPPER barb) -> tank fitting at the FRONT of the tank top. 87 mm.
- **Refill in** (ONE 225 mm piece): strainer on the arm tip -> clipped along the arm's outer face to 8 mm from the pivot
  in the groove on the arm's outer face (3 zip ties round arm + hose) -> a 65 mm free loop -> through the oblique wall gland at X 124, Z 116 ->
  refill pump, LOWER barb. With the arm stowed, check the loop hangs clear of the wheel; the checker proves 15 mm or more at
  five arm angles and that the same length works from stowed to deployed (the loop changes shape, not length).
- Cut each tube to its routed length + 10 mm per barb end; push 10 mm onto every barb.
- The routed bends are R20.9-23.2 (the target was R25; the body could not give more): slide a spring kink guard onto each
  tube at the tightest bend CHECKS.md lists, before pushing it onto the barbs.

## Cables
- Hinge crossings (all through the cable pass on the centreline at the hinge axis, tied to the strain-relief bridges either side):
  battery lead (XT60), switched 12 V to the XY-MOS (XT30), charge lead (XT30), both front motor leads (PH2.0-6P), ESP32 USB.
  Pass each plug through on its own; the pass is checked to take the biggest (XT60).
- Shell only (no crossing): VL53L0X, servo, XY-MOS gates, 9 V cells -> ESP32.
- Tub only: LiDAR (through the riser and the lid hole to the Pi), AS5600, RGB strip -> Yahboom board, E-stop, charge jack.
- Leave about 15 mm of slack in the bundle at the pass: at 20 deg the bundle's length changes by the amount in CHECKS.md, and it twists rather than stretches.
