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

## 8. v3 - the water system, built from scratch (2026-10-17)

**The brief.** The team's v3 list added the missing functions to the approved v2: a fixed spray nozzle, a refill arm with
its intake hose, a sealed wet zone with drains, every cable across the hinge, plus fixes (LiDAR clear of the lifted shell,
ESP32 held on both edges, 4 mm wheel gap, E-stop guard, AS5600 hinge sensor, RGB bar, charging without opening the robot).
v2 stays the fallback (tag `v2-approved`, STLs in `stl/v2/`). The team then asked for v3 to be researched and made from
scratch with no further questions; every assumption is listed in PROGRESS.md.

**Research first.** Two research passes (sources in PROGRESS.md / RESEARCH notes):
- WRO 2026 Future Innovators General Rules §5.8: no mist, at most 1 L of clear water. A straight stream from a blunt
  14 ga needle complies; a spray nozzle would not.
- No size limit for Junior robots except the 2 x 2 x 2 m booth, so a taller body costs judging nothing.
- R385 pump: no dimensioned drawing exists anywhere; listings agree on ~86-90 x 40 x 35 with axial barbs. Barb spacing is an assumption.
- ESP32-S3-DevKitC-1 has no mounting holes (Espressif drawing), so it is held by its edges, clear of the header pins.
- D500/LD19 scan plane is 27.0 mm above the base (two datasheets); no beam divergence is published, so 5 mm is allowed.

**Why from scratch: a tube planner said no.** Hand-placing four 6x9 tubes with R25 bends was guesswork, so we wrote a
planner: A* over positions and headings with straight steps, R25 quarter arcs, climbing helices and gentle ramps, with a
distance field for clearance. Run on v2's packing it found **no route** for the spray inlet and refill outlet: every
pump's bend lane ran into the 9 V cells or the tank. A rigid 20 deg nozzle also cannot sit over a 75 mm battery under a
151.5 top. Routing one tube at a time then failed differently (each tube took a detour that blocked the next), so we used
negotiated routing (the PathFinder method from FPGA design): all tubes routed together, overlaps penalised more each round.

**The v3 layout.** Shell = wet side, tub = dry side.
- Both pumps keep the stack over the front motors, but both now have their barbs to the RIGHT, turned on their axis so
  the barbs are one above the other: every tube gets its own lane up the right wall.
- Right third of the nose = plumbing bay: refill servo lying low at the front (shaft through the wall), ESP32 standing
  above it in a drop-in carrier with C-rails on both edges and a splash wall, USB end up under an open slot in the cap.
- Tank in the middle, lowered to Z 60.4 (its back sits over the motor can, its front over the slope).
- Battery on the left with both 9 V cells lying on it in a cradle under the battery hatch: lift the hatch and the cells come up with it.
- Nozzle centred, 15 deg down (brief: 15-25), tip 0.6 mm inside the nose face, over the tank's front gap.
- Cost: the top rises 151.5 -> 163.0 on both halves (one top line kept). It is set by the nozzle line, the tank fittings, the 9 V stack
  and one tube layer over the pumps.

**What the v3 checker caught on its first run** (and was fixed): a tube routed straight through a 9 V cell (the planner
had not been told about the cells), tubes passing over the ESP32, the nozzle cradle cutting into the battery's edge,
fitting stubs through a wall with no passage, the 9 V cells' corners in the cap's inner round, the AS5600 chip with no
pocket, and a pump-deck lip under the pump. It also caught that the ESP32 ledge printed into the shell made the servo
impossible to fit: the ledge became a separate drop-in carrier.

**Honest limit.** After smoothing (a bending-energy model of the tube, not a taut string), the tightest bends are about
R19-R22 at a few short spots near the barbs, against the R25 target. CHECKS.md lists them; a spring kink guard goes there.

**Second round: what made the checker honest, and what it then found.**
- *Assembly is now proven, not assumed.* The old check pushed each part a fixed distance in a guessed direction. It now
  searches for a straight path that takes every part completely out of the body (above the cap seam or out of the open
  rear), against the printed parts present at that step and everything fitted before it. A sideways "6 mm" exit that
  proved nothing was removed. The honest check then failed: **the Orange Pi could not be fitted at all** - lifting it hit
  the raked front wall after 2 mm, sliding it back hit the deck's locating pins. Fix: the rear wall above the deck became a
  separate **rear hatch** (it carries the E-stop and the charge jack, and the tub cap's two rear screws clamp it); the deck's
  pins became flush posts with M3 self-tap pilots; the Pi is screwed to the deck on the bench and the pair slides in from
  behind (the Pi's top corner passes 1.6 mm under the rake). The Yahboom board rides on its shelf the same way.
- *Printed parts against each other were never checked.* Adding that found 7 overlaps (up to 84 mm3): every cap-boss column
  rose into the caps' rounded inner top edges, as did the splash walls and the pump deck's wall corners - in v2 as well.
  The caps and hatches are now cut by the body they sit on, and the pump deck is trimmed under the caps.
- *The battery hatch had no fastener* once the front boss went (it would have blocked the battery). It now has a tab
  reaching back to a boss on the left wall behind the battery bay; the pump deck is notched round it.
- *The intake hose was modelled with a different length in each pose* (102 mm stowed, 161 mm deployed) - impossible for one
  hose. It is now one length: clipped along the arm to 8 mm from the pivot (the horn screw head sets that), a 65 mm free
  loop, and a wall gland moved to X 124, Z 116 by a search over gland positions. Its length varies 2.6 mm over the swing
  and its tightest bend is R20.5 at the two ends (R27.7 mid-swing); a kink guard goes on the loop.
- *Bambu refused Print 2* ("too close to exclusion area"): it tests the plate's 2D convex hull, not each part. The packer
  now does the same, and the tank hatch moved to Print 3.
- *Height is now derived from the 9 V stack too*: raising the battery 0.15 mm for clearance put the cells into the cap's
  round, so the top is 163.0 (was 162.5).
