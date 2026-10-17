# Rover 1 chassis - engineering log (v0 -> v1)

What we changed, what we tried that did not work, and why. Every number here is either a team measurement,
from a cited drawing (RESEARCH.md) or computed by `checks.py` from the actual print files (CHECKS.md).

## Starting point (v0)
The concept-C chassis: a main tub and a front shell joined by a print-in-place hinge at axle height.
Rendering it with the real motors and wheels as ghosts showed four problems:
1. **Motors were not mounted.** The wall only had a plain Ø12.4 hole and a half saddle. There were no screws, nothing
   stopping the motor from turning, and nothing holding up the encoder end.
2. **A 32 mm gap ran the full height between tub and shell** (516 cm3 of empty space), because a tall shell
   swinging about a low hinge sweeps backwards at the top.
3. **The pumps could not be plumbed.** They stood 100 mm tall with only ~6 mm above their barbs, but 6x9 silicone
   needs a 25 mm bend radius.
4. **The nose hit steps first.** It stood 32 mm ahead of the front wheel at 26 mm ground clearance, so any step
   taller than 26 mm met the body before the wheel.

## 1. Motor mounts - research before geometry
- The question: is the 6 x M3 Ø31 circle centred on the gearbox or on the off-centre shaft, and at what angle?
- Two independent drawings (Yahboom's own 520 motor drawing and Pololu's 37D drawing) settled it:
  - the circle is centred on the **gearbox**;
  - the holes sit at **±30°, ±90°, ±150°** from the shaft direction, so the shaft lies *between* two holes.
- Simple geometry agrees: a Ø31 circle around the shaft would put holes outside the Ø37 gearbox.
- Because the drawings settled it, we did not need a test coupon.
- The result is a 6 mm pad:
  - the Ø12 x 6 boss sits in a Ø12.5 hole and ends flush outside, which locates the motor and stops it rotating;
  - six M3 holes are counterbored so M3x6 heads sit flush next to the wheel coupling (3.4 mm thread engagement,
    under the 4 mm the team measured);
  - relief pockets clear the three gearbox assembly screws;
  - a cradle supports both the gearbox and the can;
  - a zip-tie tunnel at the encoder end stops the motor drooping;
  - the PH2.0 plugs sit in a 12 mm centre gap and their leads run in a floor channel.

## 2. The hinge joint - from a gap to a wedge
- **Tried:** keep v0's vertical faces and move them closer. That doesn't work: at 20° the top of the shell
  swings back by h·sin20 = 33 mm, so vertical faces always need the full 33 mm gap.
- **Chose:** make both joint faces planes through the hinge axis. A plane through the axis stays the same plane
  as it rotates, so:
  - the gap becomes a 20° wedge that is zero at the pin;
  - the tub's front face leans back exactly 20°, so it *is* the 20° stop (the shell's rear wall lands flat on it);
  - dead volume fell from 516 to 168 cm3.
- **Which body takes the wedge:** the shell needs its full height right at the joint for the pumps, so the tub
  takes it. That is why the tub has its raked front.
- **Print-in-place details:**
  - every bore and pocket has a 45° teardrop roof;
  - the pin and every rotating part have a 45° V or chamfer underneath, so nothing in the hinge needs support
    (the overhang check reads 0 mm2);
  - all clearance cuts are swept over the full -2° to 21° range.
- **Computed result:** free from -1.5° to 20.25°, stops at -1.75° (the 0° catch, under the shell's own weight)
  and 20.5°; as-printed gap 0.396 mm.

## 3. Packing - what forced the layout
- **The pumps decide the front shell.** Lying across the robot, a pump plus its tube-bend zone takes 130 of the
  143 mm inside width, so no other part can share a pump's row.
- **The tank must stand** (its level strip is vertical), so it has to sit on a floor: that floor is the nose bay.
- **The battery must lift straight out**, so nothing can sit above it.
- **Result:** both pumps sit side by side over the front motors, with their barbs facing a "wet corridor" on the
  left that leads straight to the tank. The nose holds the tank, 9 V cells, battery and ESP32, each in its own pocket.
- **Tried:** standing the pumps up (v0). It doesn't work: the tubes cannot bend without raising the roof ~30 mm.
- **Tried:** stacking the pumps two-high. It works, but the shell would be ~30 mm taller than the tub and the
  LiDAR would need raising. Rejected for a higher centre of mass.
- **The tub:**
  - the Pi lies flat, which is the only way its 40 mm plug zone fits across the width;
  - the Yahboom board hangs under the tub lid on four standoffs, components up so the IMU axes are unchanged;
  - the E-stop sits above the plug zone at the rear.
- **Cost:** the body is 33 mm longer than v0, and the tub stays tall.
- **Benefit:**
  - the shell is 16 mm lower;
  - the hinge gap is closed;
  - every component has room for its plugs, fan air and tube bends, which v0 never reserved;
  - the front axle carries 73.8% of the weight (target 70%) because the heaviest parts are in the nose.

## 4. Print 2 had to fit one bed
- **Tried:** full-size decks and a separate board shelf. The parts covered 98% of the plate, which is impossible.
- **Changed:**
  - decks now carry only what sits on them, resting on printed posts;
  - the board shelf became standoffs on the lid;
  - the LiDAR plate screws into the top of two of those standoffs, so the LiDAR stays centred;
  - the lids lie top-down.
- **Tried:** an automatic rectangle packer. It gave up at 77% coverage, so the plate is laid out by hand and
  checked for touching parts.

## 5. Bugs the checks caught (and why the checks exist)
- **Disappearing wall:** the first cavity was opened at the top by stretching vertically, which cut through the
  raked tub wall above Z 82. Only the centreline section showed it.
- **Silent overlaps:** a drain hole tangent to a cradle face, and a board model made of touching boxes, both
  produced meshes that were not watertight. A non-watertight mesh reads "0 overlap" and passes silently, so the
  checker now refuses to run on one.
- **Thickness of the floor offset:** the inside of a 45° belly ramp starts 1 mm before the outside does, so the
  first XY-MOS position sat 1.6 mm into the ramp.

## 6. v2 - the team's review of v1 (Nikhil)
**The review.** The motor mounts and hinge were kept. v1 had got worse on space: 239 mm long against v0's 206, 7% more
volume, and 53% fill against 59%. The new targets:
- 210 mm long or less;
- a nose at most 20 mm past the worst-case wheel, with a 45° underside starting behind the wheel;
- 0.4–0.5 mm per side in every pocket;
- nothing outside the outline;
- tub and shell tops matched;
- 6–10 mm rounds on the outside edges.

**What the short nose forced.**
- With a 45° belly starting behind the wheel, there is no flat floor ahead of the front motors any more.
- The tank (80 tall, it must stand because of its level strip), the battery, the 9 V cells and the ESP32 now stand on
  printed ribs at deck level.
- The two pumps are stacked, with their barbs at opposite ends so each tube-bend zone sits beside the other pump.
- Result: 206.5 mm long and a 19.7 mm overhang.
- The cost is height: the top is now 151.5 mm (v0 137). The front axle carries 61% instead of 74%, because the battery
  can no longer sit 30 mm ahead of the axle.
- The shell still holds itself closed (837 N·mm about the hinge), and that was the reason for the 70% target.

**Tried and rejected.**
- *45° slope starting just 8.8 mm behind the 86.6 mm wheel.* The curb check showed that with the 80 mm wheel a 36 mm
  curb hits the body first. The slope now starts at X 119, and both wheel sizes pass for every curb height up to the axle.
- *Moving the Yahboom board to a lower bay.* Those bays are 24–29 mm deep and the board is 56, so it stays on a shelf
  over the Pi. With one shared top line this costs no height.
- *Hanging the board from the tub cap.* At a 151.5 top, the raked tub face is too narrow for the board's front standoffs.
- *R8 on the long top edges.* The upper pump and ESP32 ends touched the curve, so the long top edges are R6 (still in
  the 6–10 range); the vertical edges and the front/rear top edges are R8.

**What the slicer taught us.** Bambu Studio's command line sliced Print 1 and refused Print 2. Read through the bed →
model mapping, the problems it found were:
- the shell cap sat in the P1S's no-print corner;
- the nose-bay walls over the front motors started in mid-air (they would need a ~140 mm bridge), so they moved onto
  the removable pump deck;
- the cable pass had a flat 20 mm roof, now a 45° point;
- the E-stop hole had a round roof, now a teardrop;
- a deck post stopped 0.5 mm above the sloped tail floor.

**Clearance check.** v1 reported 0.0–0.09 mm on almost everything. That was the checker measuring points right next to
the surface a part rests on. The rule is now: ignore the designed seat plus a 0.6 mm band beside it, and fail anything
under 0.4 mm. Under that rule it caught real touches: the ESP32 top against the cap's side round, the servo body on a
motor pad, and the deck wall against the pumps.

## 7. Final (approved 2026-10-17, no ballast)

**Decisions.**
- The team kept the ~150 mm top. A 140 mm top cannot be met with these parts and rules:
  - Laid out in one layer, the big parts need about 17,900 mm² of floor against 14,750 available.
  - Stacking any two of them needs about 82 mm above the deck.
  - 90° elbow barbs on the pumps don't change that, so they were not bought.
- No ballast. The front axle carries 61% (CG X 57, height 79 mm), and the shell still holds itself closed with 836 N·mm
  about the hinge.

**Slicer as a design tool.** Bambu Studio's command line found problems our own checks didn't:
- a part inside the P1S's no-print corner;
- walls starting in mid-air;
- flat hole roofs;
- a floating ledge.

Each one was traced back from bed coordinates to the model and fixed. Final slice (P1S, PETG, 0.20 mm Strength, no
supports, no warnings):

| Print | Time | Filament |
|---|---|---|
| 1 | 15 h 19 m | 449.7 g |
| 2 | 4 h 50 m | 133.7 g |

Forcing supports on adds only about 1 g, all in the first 6 layers. Nothing lands between the hinge knuckles.

**Lesson for the report.** Three independent checks found different things:
- `checks.py` covered geometry and fit;
- the printability audit covered overhangs, islands and thin walls;
- the slicer covered what the printer will actually do.

None of them alone was enough.
