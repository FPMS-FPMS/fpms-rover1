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
