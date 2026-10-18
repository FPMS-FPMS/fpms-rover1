# FPMS Rover 1 chassis - CadQuery source, v3 (single file). v2 (approved, tag v2-approved) is fpms_rover1_cad.py.
# Frame: X forward (rear axle at X=0), Y left, Z up, mm. Ground at Z=0.
# Print 1 = main tub + front shell printed ASSEMBLED, hinge printed in place (0.4 mm, teardrop bores).
# Print 2 = the two caps (rounded top 8 mm of each body) + tank hatch + removable decks + board shelf + LiDAR plate.
# Sources for every part dimension: RESEARCH.md. Team measurements win over online numbers.
# Run:  pip install cadquery trimesh manifold3d shapely   then   python fpms_rover1_cad.py [--prints]
#
# v2 (Nikhil's review of v1): <= 210 long, nose <= 20 past the worst-case wheel with a 45 deg underside from behind
# the wheel, 0.45 per side in every pocket, nothing outside the body outline, equal tops, R8 rounds, no add-on boxes.
# v3 (team brief 2026-10-17): water system (nozzle, refill arm, wet zone, drains), cables across the hinge, LiDAR clear
# of the lifted shell, ESP32 held on both edges, 4 mm wheel gap, E-stop guard, AS5600 hinge sensor, RGB bar, charge port.
# Layout re-done from scratch: the shell is the WET side (pumps, tank, tubes, battery), the tub is the DRY side.
import math, os, json
import cadquery as cq
from shapely.geometry import Polygon, Point, box as sbox
from shapely.ops import unary_union
from shapely import affinity

# ---------------- frame (fixed layout facts) ----------------
XR, XF = 0.0, 93.5                 # rear / front axle
XH, ZH = 48.5, 40.0                # transverse hinge: axle height, 45 behind the front axle
ZGB = 47.0                         # gearbox centre = axle + 7 (JGB37 shaft is 7 off-centre, below)
BELLY, T = 26.0, 2.4               # ground clearance, wall
FLOOR = BELLY + T
W = 74.0; WI = W - T               # outer / inner half width (motors back to back: 62 + 6 cable gap + 6 pad)
GAP = 0.4                          # print-in-place clearance everywhere in the hinge
J_TUB, J_SH = XH - GAP / 2, XH + GAP / 2
RAKE = 20.0                        # tub front face leans back 20 deg: the shell's rear face lands on it = 20 deg stop
TANR = math.tan(math.radians(RAKE)); TIN = T / math.cos(math.radians(RAKE))
SWEEP = (-2.0, 21.0)               # hinge range the clearance cuts are swept over (stops land at ~-1.75 and ~20.3)
WHEEL_R_WORST = 86.6 / 2           # Yahboom drawing (team: 80, still to caliper) - every outline check uses the bigger one
WHEEL_FRONT = XF + WHEEL_R_WORST   # 136.8

def tub_face_x(z):                 # outer joint face of the tub
    return J_TUB - max(0.0, z - ZH) * TANR
def tub_face_in(z):                # inner surface of that wall
    return tub_face_x(z) - (TIN if z > ZH else T)

# ---------------- parts (mm) ----------------
M_GB_R, M_GB_L = 18.5, 21.5        # gearbox Ø37 x 21.5 (team: 22)
M_CAN_R = 16.4                     # can Ø32.8
M_LEN = 62.0                       # face -> back of PH2.0 plug (team)
M_FACE = 68.0                      # gearbox face sits on the pad's inner face at |y| 68 -> plug ends at |y| 6 (12 mm cable gap)
PAD = W - M_FACE                   # 6 mm pad = boss length, boss ends flush outside
CLR = 0.45                         # every pocket: component + 0.45 per side (PETG pockets print undersize)
DECK_Z0, DECK_Z1 = 66.0, 68.4      # deck level just over the gearboxes (top Z 65.5)

# hole pattern on the gearbox face (RESEARCH.md): Ø31 about the GEARBOX centre, holes at ±30/±90/±150 deg
# from the shaft direction (shaft is below the centre), assembly screws at 180 and ±60 deg.
def face_pts(angles, r=15.5):
    return [(15.5 * math.sin(math.radians(a)), -15.5 * math.cos(math.radians(a))) for a in angles]
M3_ANG = (30, -30, 90, -90, 150, -150)
ASM_ANG = (180, 60, -60)

# ---------------- front shell layout (v3) ----------------
# Pumps stay stacked over the front motors (the only place a 100 mm pump row + its tube bends fits). Both now have their
# barbs to the RIGHT and are mounted barbs-vertical (rotated on their own axis), so every tube leaves the pump zone in its
# own lane along the right wall. The right third of the nose is the plumbing bay (refill-arm servo low at the front, ESP32
# standing over it behind a splash wall); the tank stands next to it, the battery against the left wall.
PUMP = (50.0, 100.0, 40.0)          # team: 100 x 40 x 50 with fittings; lying: 50 deep (X), 100 across (Y), 40 tall
BEND = 31.15                        # 6x9 silicone: R25 bend + tube radius + 1.6 mm of play beyond the barb tip
P_X0 = J_SH + T + CLR
P_X1 = P_X0 + PUMP[0]
P_XC = (P_X0 + P_X1) / 2            # barbs on the pump axis (pump rotated so the two barbs are one above the other)
P_Z = (DECK_Z1, DECK_Z1 + PUMP[2], DECK_Z1 + 2 * PUMP[2])
P_Y = (-WI + CLR + BEND, -WI + CLR + BEND + PUMP[1])        # both pumps; barb tips at P_Y[0], bend zone to the right wall
BARB_DZ = 7.0                       # barbs +-7 about the pump centre line (assumed; measure the pump)
BARBS = {'spray_out': P_Z[0] + 20 - BARB_DZ, 'spray_in': P_Z[0] + 20 + BARB_DZ,     # lower pump = SPRAY (outlet climbs)
         'refill_in': P_Z[1] + 20 - BARB_DZ, 'refill_out': P_Z[1] + 20 + BARB_DZ}   # upper pump = REFILL
TUBE_R = 4.5                        # 6x9 silicone
P_XC_BARB = 77.5                    # barb axis X (routing grid; ~1 mm ahead of the pump centre - measure)
DIV = 1.2
S2_X0 = P_X1 + CLR + DIV            # nose bay starts behind its rear wall
BATT = (50.0, 65.0, 75.0)           # 65x75x50 pack standing, 50 deep (team measurement)
TANK = (40.0, 40.0, 80.0)
TANK_FIT = 17.0                     # assumed: cap + 2 x 90 deg elbow barbs on the top face, tube leaving sideways
NINE = (48.5, 26.5, 17.5)           # 9 V Li-ion USB cells, lying FLAT on top of the battery in the hatch cradle
ESP = (25.4, 13.0, 68.8)            # ESP32-S3 DevKitC-1 standing: 62.74 PCB + ~6.2 antenna overhang (Espressif drawing)
SERVO = dict(body=(22.8, 12.2, 28.5), ears=32.0, ear_down=18.5, shaft_from_end=5.9, spline=4.0)
NOSE_I = S2_X0 + CLR + BATT[0] + CLR
NOSE = NOSE_I + T
RAMP_X0 = 119.0                     # 45 deg rise starts 14.5 behind the Ø80 wheel front (curb check in checks.py)
NOSE_SLOPE_IN = lambda x: x - RAMP_X0 + BELLY + T * math.sqrt(2)    # inner surface height of the 45 deg nose slope
def NOSE_FLOOR_IN(x):
    """inside surface under the nose: the 45 deg slope, and near the front wall the R(8 - T) inner round of the R8 corner"""
    z = NOSE_SLOPE_IN(x)
    rt = 8.0                                                                            # = R_TOP (defined later)
    xc = NOSE - rt; zc = BELLY + (NOSE - RAMP_X0) - rt + rt * math.sqrt(2)              # centre of the corner round
    ri = rt - T
    if x > xc:
        z = max(z, zc - math.sqrt(max(0.0, ri ** 2 - (x - xc) ** 2)))
    return z
NOSE_Z0 = math.ceil((NOSE_FLOOR_IN(NOSE_I - CLR) + CLR + 0.35) * 10) / 10   # rib tops: the 50 deep battery clears slope + round
BAY = (-WI, -WI + 33.9)             # plumbing bay (servo + ESP32 + tubes)
TANK_SLOT = (BAY[1] + DIV, BAY[1] + DIV + TANK[1] + 2 * CLR)
BAT_SLOT = (TANK_SLOT[1] + DIV, WI)
assert BAT_SLOT[1] - BAT_SLOT[0] >= BATT[1] + 2 * CLR - 1e-6, ('battery slot too narrow', BAT_SLOT)
TANK_X = (S2_X0 + CLR, S2_X0 + CLR + TANK[0])                     # tank at the back of its slot; nozzle runs ahead of it
# the tank's back sits over the right front motor can (top Z 59.9 there), its front over the slope: it can stand lower
# than the battery, which buys the nozzle line room over it
TANK_Z0 = math.ceil((ZGB + math.sqrt(M_CAN_R ** 2 - (TANK_X[0] - XF) ** 2) + CLR) * 10) / 10
TANK_Z = (TANK_Z0, TANK_Z0 + TANK[2])
# spray nozzle: centred, fixed, 15 deg down (brief: 15-25). Rigid stack = female Luer to 1/4" barb adapter + 14 ga blunt
# Luer-lock tip, 1/4" needle (straight stream, McMaster 75165A117 class). Tip stays inside the outline (0.6 behind the nose).
NOZ = dict(angle=15.0, barb=10.0, body=27.0, needle=6.35, r=4.75)
NOZ['L'] = NOZ['barb'] + NOZ['body'] + NOZ['needle']
_c, _s = math.cos(math.radians(NOZ['angle'])), math.sin(math.radians(NOZ['angle']))
NOZ['tip'] = (NOSE - 0.6, 0.0, None)
_zt = (TANK_Z[1] + CLR) + NOZ['r'] / _c - (NOZ['tip'][0] - (TANK_X[1] + CLR)) * _s / _c      # clears the tank top
NOZ['tip'] = (NOSE - 0.6, 0.0, math.ceil(_zt * 10) / 10)
NOZ['rear'] = (NOZ['tip'][0] - NOZ['L'] * _c, 0.0, NOZ['tip'][2] + NOZ['L'] * _s)            # where the tube meets the barb
NOZ['top'] = NOZ['rear'][2] + TUBE_R + 25.0 * (1 - _c)                                       # tube bends 15 deg into it
BATT_Z = (NOSE_Z0, NOSE_Z0 + BATT[2])
# refill-arm servo: lying, case top against the right wall, spline through it, shaft near the FRONT end (front ear cut
# off: it would reach the nose wall). Pivot as far forward and low as the 45 deg slope allows, so the arm clears the wheel.
SERVO_X1 = NOSE_I - CLR - 0.6
SERVO_PX = SERVO_X1 - SERVO['shaft_from_end']
SERVO_Z0 = math.ceil((NOSE_FLOOR_IN(SERVO_X1) + CLR + 0.2) * 10) / 10
SERVO_PZ = SERVO_Z0 + SERVO['body'][1] / 2
SERVO_TOP = SERVO_Z0 + SERVO['body'][1]
SERVO_Y = (-WI, -WI + SERVO['body'][2])
# ESP32 turned to face the nose (board plane YZ), standing at the very front of the bay over the servo: the tube U-turns
# from the upper pump land at X ~127 and must not pass over it
ESP_X = (NOSE_I - CLR - ESP[1], NOSE_I - CLR)
ESP_Y = (-WI + 6.0 + CLR, -WI + 6.0 + CLR + ESP[0])              # clear of the front-right cap boss
ESP_Z0 = SERVO_TOP + CLR + 1.6 + CLR                                 # on a ledge plate over the servo
ESP_Z1 = ESP_Z0 + ESP[2]
H_IN = max(P_Z[2] + CLR + 2 * TUBE_R + CLR,                          # one tube layer over the upper pump
           TANK_Z[1] + TANK_FIT + CLR,                               # tank fittings
           NOZ['top'] + CLR,                                         # nozzle line
           ESP_Z1 + CLR, BATT_Z[1] + CLR,
           BATT_Z[1] + CLR + 0.8 + NINE[2] + CLR + 0.5)              # 9 V cells on the cradle, under the cap's inner round
H = math.ceil((H_IN + T) * 2) / 2                                    # one top line for tub and shell
H_IN = H - T
CAP = 8.0
Z_SPLIT = H - CAP
R_V, R_TOP, R_BELLY = 8.0, 8.0, 4.0
R_TOP_SIDE = 6.0
R_JOINT = 6.0
PAD_HALF = 19.5

# ---------------- main tub layout ----------------
OPI = (65.0, 92.8, 35.8); OPI_PLUG, OPI_AIR = 40.0, 10.0
BOARD = (56.0, 85.0, 20.0)          # Yahboom V3.0 flat on its own removable shelf above the Pi
OPI_Z0 = DECK_Z1
OPI_Z1 = OPI_Z0 + OPI[2]
SHELF_T, STANDOFF = 2.4, 2.0
SHELF_Z0 = OPI_Z1 + CLR + OPI_AIR   # Pi fan gets 10 mm of air under the shelf (shelf is open over the fan)
SHELF_Z1 = SHELF_Z0 + SHELF_T
BOARD_Z0 = SHELF_Z1 + STANDOFF
BOARD_Z1 = BOARD_Z0 + BOARD[2]
SHIFT_Y = 5.6                       # Pi + board moved off the right wall so the right rear corner can be rounded
ESTOP = dict(y=(24.0, 64.0), z=(OPI_Z1 + 0.8, OPI_Z1 + 30.8), depth=45.0, head_r=20.0, head_h=22.0)
OPI_X1 = tub_face_in(OPI_Z1 + CLR) - CLR; OPI_X0 = OPI_X1 - OPI[0]
OPI_Y0 = -WI + CLR + SHIFT_Y;       OPI_Y1 = OPI_Y0 + OPI[1]
BOARD_X1 = tub_face_in(BOARD_Z1 + CLR) - CLR; BOARD_X0 = BOARD_X1 - BOARD[0]
BOARD_Y1 = ESTOP['y'][0] - CLR - 3.0; BOARD_Y0 = BOARD_Y1 - BOARD[1]   # connectors (+3) toward +Y, the cable riser
BOARD_HOLES = [(BOARD_X0 + 3.5 + i * 49.0, y) for i in (0, 1) for y in (BOARD_Y1 - 23.5, BOARD_Y1 - 81.5)]
XRI = min(OPI_X0, BOARD_X0) - CLR   # tub rear wall inner face
XRO = math.floor(XRI - T)
XRI = XRO + T
REG = dict(x=(22.0, 31.5), y=(42.0, 67.4), z=(FLOOR, FLOOR + 25.4))
TAIL_X = -22.0                      # tub belly rises 45 deg behind the rear motors
XYMOS_X, XYMOS_Y = (64.5, 76.5), (-27.5, 27.5)                   # standing in the shell bay behind the front motors
LIDAR = dict(fp=(46.29, 54.0), h=34.8, scan=27.0, holes=(46.8, 31.92), plate=4.0)
LIDAR['cx'] = round((XRO + R_TOP + tub_face_x(H) - R_TOP) / 2, 1)   # middle of the flat part of the tub cap
LIDAR_HOLES = [(LIDAR['cx'] - 8.7, -23.4), (LIDAR['cx'] - 8.7, 23.4), (LIDAR['cx'] + 23.2, 0.0)]
LIDAR_PADS = [(LIDAR['cx'] + dx, dy) for dx in (-15.0, 15.0) for dy in (-17.0, 17.0)]
LIDAR_LEAD = (LIDAR['cx'] - 6, LIDAR['cx'] + 6, -6, 6)
# E-stop = main power switch (latching, already in the battery line; press = off, twist = on). Guard fins either side of
# the head stop TRAVEL short of tripping it: a flat wall met in reverse is held 3 mm before the head bottoms (needs >= 4 mm
# trip travel - measure), a palm slap still trips it.
ESTOP_GUARD = dict(gap=3.0, t=2.4, short=3.0, half_h=16.0)
# charge inlet: DC-099 metal 5.5 x 2.1 panel jack (Ø11 hole, 16 deep behind the panel), rear wall above the board
JACK = dict(y=-30.0, z=None, r_hole=11.2 / 2, depth=16.0, body_r=6.0)
JACK['z'] = BOARD_Z1 + CLR + JACK['body_r'] + 1.5
# AS5600 hinge-angle sensor: 6 x 2.5 diametric magnet in a shell lug ON the hinge axis, board in a tub slot facing it
AS = dict(board=23.0, pcb=1.6, chip_h=1.75, gap=1.0, mag_r=3.0, mag_t=2.5, lug_r=6.0, y_lug0=10.4)
AS['y_chip'] = 23.0                                       # board's chip-side face (chip faces -Y, toward the magnet)
AS['y_mag'] = AS['y_chip'] - AS['chip_h'] - AS['gap']     # magnet face


# ---------------- helpers ----------------
def bx(x0, x1, y0, y1, z0, z1):
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
def cyl_y(x, z, r, y0, y1):
    return cq.Workplane('XZ').center(x, z).circle(r).extrude(-(y1 - y0)).translate((0, y0, 0))
def cyl_x(y, z, r, x0, x1):
    return cq.Workplane('YZ').center(y, z).circle(r).extrude(x1 - x0).translate((x0, 0, 0))
def cyl_z(x, y, r, z0, z1):
    return cq.Workplane('XY').center(x, y).circle(r).extrude(z1 - z0).translate((0, 0, z0))
def teardrop2d(x, z, r):            # circle with a 45 deg point up: prints without support in a vertical wall
    return unary_union([Point(x, z).buffer(r, 64),
                        Polygon([(x - r / math.sqrt(2), z + r / math.sqrt(2)), (x, z + r * math.sqrt(2)), (x + r / math.sqrt(2), z + r / math.sqrt(2))])])
def vdown2d(x, z, r):               # circle with a 45 deg point down: self-supporting underside
    return unary_union([Point(x, z).buffer(r, 64),
                        Polygon([(x - r / math.sqrt(2), z - r / math.sqrt(2)), (x, z - r * math.sqrt(2)), (x + r / math.sqrt(2), z - r / math.sqrt(2))])])
def polys(g):
    g = g.simplify(0.005)
    return [g] if g.geom_type == 'Polygon' else [p for p in g.geoms if p.geom_type == 'Polygon' and p.area > 1e-3]
def ring(coords):                   # drop repeated / near-duplicate vertices (OCC refuses zero-length edges)
    out = []
    for a, b in list(coords)[:-1]:
        p = (round(a, 4), round(b, 4))
        if not out or abs(p[0] - out[-1][0]) + abs(p[1] - out[-1][1]) > 1e-3:
            out.append(p)
    while len(out) > 2 and abs(out[0][0] - out[-1][0]) + abs(out[0][1] - out[-1][1]) <= 1e-3:
        out.pop()
    return out
def _prism(plane, g, a, b, shift):
    out = None
    for p in polys(g):
        s = cq.Workplane(plane).polyline(ring(p.exterior.coords)).close().extrude(b)
        for h in p.interiors:
            s = s.cut(cq.Workplane(plane).polyline(ring(h.coords)).close().extrude(b))
        s = s.translate(shift(a))
        out = s if out is None else out.union(s)
    return out
def prism_y(g, y0, y1):             # shapely XZ region -> solid between y0 and y1
    return _prism('XZ', g, y0, -(y1 - y0), lambda a: (0, a, 0))
def prism_z(g, z0, z1):             # shapely XY region -> solid between z0 and z1
    return _prism('XY', g, z0, z1 - z0, lambda a: (0, 0, a))
def prism_x(g, x0, x1):             # shapely YZ region -> solid between x0 and x1
    return _prism('YZ', g, x0, x1 - x0, lambda a: (a, 0, 0))
def teardrop_y(x, z, r, y0, y1):
    return prism_y(teardrop2d(x, z, r), y0, y1)
def rrect(x0, x1, y0, y1, rads):    # rounded rectangle, rads = (x0y0, x1y0, x1y1, x0y1) corner radii
    corners = [((x0, y0), (1, 1), 180), ((x1, y0), (-1, 1), 270), ((x1, y1), (-1, -1), 0), ((x0, y1), (1, -1), 90)]
    out = []
    for i, ((cx, cy), (sx, sy), a0) in enumerate(corners):
        r = rads[i]
        if r <= 0:
            out.append((cx, cy)); continue
        ox, oy = cx + sx * r, cy + sy * r
        for k in range(13):
            a = math.radians(a0 + 90 * k / 12)
            out.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return Polygon(out)
def round_convex(p, r, keep):
    """round the convex corners of polygon p with radius r, only inside the shapely region `keep`"""
    rp = p.buffer(-r, join_style=1).buffer(r, join_style=1, quad_segs=12)
    return unary_union([rp.intersection(keep), p.difference(keep)]).buffer(0)
def rot2d(g, deg):
    return affinity.rotate(g, deg, origin=(XH, ZH))
def sweep2d(g, sign):               # union of g rotated over the hinge range (sign +1: shell's motion seen from the tub)
    angs = [SWEEP[0] + i * 0.5 for i in range(int((SWEEP[1] - SWEEP[0]) / 0.5) + 1)]
    return unary_union([rot2d(g, sign * a) for a in angs])
def ledge(x0, x1, y0, y1, ztop, d=3.0, axis='y', into=+1):
    """deck ledge: 3 mm flat top, 45 deg gusset under it so it prints without support."""
    if axis == 'y':
        wy = y1 if into < 0 else y0
        prof = Polygon([(wy, ztop), (wy + into * d, ztop), (wy + into * d, ztop - 0.8), (wy, ztop - 0.8 - d)])
        return prism_x(prof, x0, x1)
    wx = x1 if into < 0 else x0
    prof = Polygon([(wx, ztop), (wx + into * d, ztop), (wx + into * d, ztop - 0.8), (wx, ztop - 0.8 - d)])
    return prism_y(prof, y0, y1)
def try_fillet(w, sel, r):
    try:
        return w.edges(sel).fillet(r)
    except Exception as e:
        print('  fillet skipped', sel, r, e.__class__.__name__); return w

# ---------------- hinge (all profiles in the XZ plane, extruded along Y) - unchanged from v1 ----------------
PIN_R, CUP_R, KN_R = 5.0, 12.5, 11.5          # >= 3.9 mm of wall round every bore
CUPS_Y = [(30.0, 42.0), (62.0, W)]          # tub cups, pin fused, both sides (mirrored)
KN_Y = (42.0 + GAP, 62.0 - GAP)              # shell knuckle between them
def pin2d():
    return vdown2d(XH, ZH, PIN_R)
def cup2d():                                 # forward part chamfered 45 deg underneath (starts at the tub face)
    c = Point(XH, ZH).buffer(CUP_R, 64)
    keep = Polygon([(J_TUB - 30, 29.0 - 30), (J_TUB, 29.0), (J_TUB + 40, 29.0 + 40), (J_TUB + 40, 90), (J_TUB - 30, 90)])
    return c.intersection(keep)
def knuckle2d():                             # rearward part chamfered 45 deg underneath (starts at the shell face)
    c = Point(XH, ZH).buffer(KN_R, 64)
    keep = Polygon([(J_SH + 30, 29.0 - 30), (J_SH, 29.0), (J_SH - 40, 29.0 + 40), (J_SH - 40, 90), (J_SH + 30, 90)])
    lug = sbox(J_SH, BELLY + 0.01, J_SH + 12.0, ZH + 8.0)
    return unary_union([c.intersection(keep), lug])
def cap_up(r):                               # teardrop roof for a cylindrical pocket about the hinge axis
    return Polygon([(XH - r / math.sqrt(2), ZH + r / math.sqrt(2)), (XH, ZH + r * math.sqrt(2)), (XH + r / math.sqrt(2), ZH + r / math.sqrt(2))])
def shell_bore2d():                          # pin seen from the shell moves by -theta
    # extra 0.45 under the V tip: the pin's own outline moved down, clipped at the pin centre so no flat roof is left
    tip = affinity.translate(pin2d(), 0, -0.45).buffer(GAP, 32).intersection(sbox(XH - 20, ZH - 20, XH + 20, ZH))
    return unary_union([sweep2d(pin2d().buffer(GAP, 32), -1), sweep2d(tip, -1), cap_up(PIN_R + GAP)])
def shell_cup_pocket2d():
    return unary_union([sweep2d(cup2d().buffer(GAP, 32), -1), cap_up(CUP_R + GAP)])
def tub_knuckle_pocket2d():                  # 1.2 over the knuckle top: the raked wall's edge bridges 19.5 mm there
    top = knuckle2d().buffer(1.2, 32).intersection(sbox(0, ZH + 4, 200, 200))
    return unary_union([sweep2d(knuckle2d().buffer(GAP, 32), +1), sweep2d(top, +1), cap_up(KN_R + 1.2)])

# ---------------- bodies: three rounded profiles intersected, hollowed, top 8 mm split off as the cap ----------------
def tub_profile():
    p = Polygon([(XRO, H), (XRO, BELLY + (TAIL_X - XRO)), (TAIL_X, BELLY), (J_TUB, BELLY), (J_TUB, ZH), (tub_face_x(H), H)])
    return round_convex(p, R_TOP, sbox(-300, 0, 30, 400))           # every corner except the hinge region
def shell_profile():
    p = Polygon([(J_SH, BELLY), (RAMP_X0, BELLY), (NOSE, BELLY + (NOSE - RAMP_X0)), (NOSE, H), (J_SH, H)])
    return round_convex(p, R_TOP, sbox(70, 0, 400, 400))
def yz_profile():
    return rrect(-W, W, BELLY, H, (R_BELLY, R_BELLY, R_TOP_SIDE, R_TOP_SIDE))
def yz_inner():
    return rrect(-WI, WI, FLOOR, H_IN, (0, 0, R_TOP_SIDE - T, R_TOP_SIDE - T))
def body(xz, plan_o, plan_i):
    outer = prism_y(xz, -W - 1, W + 1).intersect(prism_z(plan_o, 0, 400)).intersect(prism_x(yz_profile(), -200, 400))
    inner = prism_y(xz.buffer(-T, join_style=2), -W, W).intersect(prism_z(plan_i, 0, 400)).intersect(prism_x(yz_inner(), -200, 400))
    return outer, inner
def tub_plans():
    return (rrect(XRO, J_TUB, -W, W, (R_V, R_JOINT, R_JOINT, R_V)),
            rrect(XRI, J_TUB + 5, -WI, WI, (R_V - T, 0, 0, R_V - T)))
R_V_NOSE = 6.0                      # nose's vertical corners: R6 outside, R0.5 inside (the battery and servo reach them)
def shell_plans():
    return (rrect(J_SH, NOSE, -W, W, (R_JOINT, R_V_NOSE, R_V_NOSE, R_JOINT)),
            rrect(J_SH - 5, NOSE_I, -WI, WI, (0, 0.5, 0.5, 0)))
BELOW = lambda: bx(-300, 400, -200, 200, -10, Z_SPLIT)
ABOVE = lambda: bx(-300, 400, -200, 200, Z_SPLIT, 400)

def motor_mount(w, xa):
    """JGB37-520 bolted through the side wall: Ø12.9 boss hole, 6 x M3 counterbored flush,
    assembly-screw reliefs, cradle under gearbox + can, zip-tie tunnel at the encoder end."""
    for s in (1, -1):
        ylo, yhi = sorted((s * M_FACE, s * WI))
        w = w.union(bx(xa - PAD_HALF, xa + PAD_HALF, ylo, yhi, FLOOR - 0.5, DECK_Z0))                        # pad
        a, b = sorted((s * (M_FACE - 1), s * (W + 1)))
        w = w.cut(teardrop_y(xa, ZH, 6.0 + CLR, a, b))                                                        # boss locates
        for (dx, dz) in face_pts(M3_ANG):
            w = w.cut(teardrop_y(xa + dx, ZGB + dz, 3.4 / 2, a, b))
            c0, c1 = sorted((s * (W - 3.4), s * (W + 1)))
            w = w.cut(teardrop_y(xa + dx, ZGB + dz, 6.0 / 2, c0, c1))                                         # counterbore: head flush
        for (dx, dz) in face_pts(ASM_ANG):
            r0, r1 = sorted((s * (M_FACE - 1), s * (M_FACE + 1.2)))
            w = w.cut(teardrop_y(xa + dx, ZGB + dz, 3.0, r0, r1))                                             # relief for proud screw heads
        g0, g1 = sorted((s * (M_FACE - M_GB_L + 1.0), s * (M_FACE - 1.5)))
        w = w.union(bx(xa - 15, xa + 15, g0, g1, FLOOR - 0.5, 43).cut(cyl_y(xa, ZGB, M_GB_R + CLR, -100, 100)))
        c0, c1 = sorted((s * 26.0, s * (M_FACE - M_GB_L - 1.0)))
        w = w.union(bx(xa - 14, xa + 14, c0, c1, FLOOR - 0.5, 44).cut(cyl_y(xa, ZGB, M_CAN_R + CLR, -100, 100)))
        t0, t1 = sorted((s * 15.4, s * 23.6))                                                                 # strap block, encoder end
        blk = bx(xa - 14, xa + 14, t0, t1, FLOOR - 0.5, 44).cut(cyl_y(xa, ZGB, M_CAN_R + CLR, -100, 100))
        u0, u1 = sorted((s * 16.6, s * 22.4))
        um = (u0 + u1) / 2                                                                                    # gable roof: no flat ceiling
        blk = blk.cut(prism_x(Polygon([(u0, FLOOR - 0.6), (u1, FLOOR - 0.6), (u1, ZGB - M_CAN_R + 3.0),
                                       (um, ZGB - M_CAN_R + 3.0 + (u1 - u0) / 2), (u0, ZGB - M_CAN_R + 3.0)]), xa - 20, xa + 20))                               # tie tunnel under the can
        w = w.union(blk)
    w = w.cut(cyl_y(xa, ZGB, M_GB_R + CLR, M_FACE - M_GB_L - 1, M_FACE + 0.01)).cut(
        cyl_y(xa, ZGB, M_GB_R + CLR, -M_FACE - 0.01, -(M_FACE - M_GB_L - 1)))                                 # trough so the gearbox clears the floor
    return w

def channel(w, x0, x1):                      # cable channel on the centreline: two ribs, 6 tall
    for s in (1, -1):
        a, b = sorted((s * 6.6, s * 7.8))
        w = w.union(bx(x0, x1, a, b, FLOOR - 0.5, FLOOR + 6))
    return w

def cap_boss(w, x, y_wall, into, along='x'):
    """cap screw column on a wall: rises from a 45 deg gusset up to the cap's underside (M3 pilot Ø2.6)"""
    L, D, z0 = 7.0, 6.0, Z_SPLIT - 6.0
    prof = Polygon([(y_wall, H_IN), (y_wall + into * D, H_IN), (y_wall + into * D, z0), (y_wall, z0 - D)])
    if along == 'x':
        g = prism_x(prof, x - L / 2, x + L / 2); hx, hy = x, y_wall + into * D / 2
    else:
        g = prism_y(prof, x - L / 2, x + L / 2); hx, hy = y_wall + into * D / 2, x
    return w.union(g).cut(cyl_z(hx, hy, 1.3, H_IN - 10, H_IN + 1)), (hx, hy)

TUB_BOSSES, SHELL_BOSSES = [], []
CABLE_PASS = Polygon([(-10, 33), (10, 33), (10, 41), (0, 58.3), (-10, 41)])  # YZ: 20 wide, 60 deg pointed roof (no auto-support)

def deck_post(w, x, y, ztop, floor_z=FLOOR):
    w = w.union(bx(x - 3.0, x + 3.0, y - 3.0, y + 3.0, floor_z - 0.5, ztop))
    return w.cut(cyl_z(x, y, 1.3, ztop - 7.0, ztop + 0.1))                     # M3 x 8 self-tap pilot

TUB_DECK_X1 = OPI_X1 + CLR + 1.6 + 3.0
TUB_DECK_Y1 = OPI_Y1 + 1.5

import functools
@functools.lru_cache(None)
def tub_raw():
    """full tub body (print 1 part + cap), before the split"""
    po, pi = tub_plans()
    outer, inner = body(tub_profile(), po, pi)
    return outer.cut(inner)

REAR_HATCH_X = XRI + 6.5            # tub rear hatch: the rear wall, its rounded corners and its two cap bosses, above the deck
REAR_HATCH_Z0 = DECK_Z0 - 0.5

def rear_region():
    return bx(XRO - 40, REAR_HATCH_X, -W - 40, W + 40, REAR_HATCH_Z0, H + 1)       # to the top: the cap-boss columns too

@functools.lru_cache(None)
def main_tub_full():
    t = tub_raw().intersect(BELOW())
    t = motor_mount(t, XR)
    t = channel(t, M_GB_R + 1, tub_face_in(FLOOR) - 1)
    # Pi deck: rear-wall ledge, right-wall ledge, 2 posts on its free edge
    t = t.union(ledge(XRI + 12, -PAD_HALF, -WI, -WI + 3, DECK_Z0, axis='y', into=+1))
    for (px, py) in ((XRI + 10.0, TUB_DECK_Y1 - 4.0), (TUB_DECK_X1 - 5.0, TUB_DECK_Y1 - 4.0)):
        if px > TAIL_X - 2:
            t = deck_post(t, px, py, DECK_Z0)
        else:                                                                     # foot follows the 45 deg tail, inside the wall
            outer = lambda x: BELLY + (TAIL_X - x) + 0.6
            foot = Polygon([(px - 3.0, outer(px - 3.0)), (px + 3.0, outer(px + 3.0)), (px + 3.0, DECK_Z0), (px - 3.0, DECK_Z0)])
            t = t.union(prism_y(foot, py - 3.0, py + 3.0)).cut(cyl_z(px, py, 1.3, DECK_Z0 - 7.0, DECK_Z0 + 0.1))   # M3 pilot
    # board shelf ledges: right wall, rear wall, and a gusset on the raked front wall
    t = t.union(ledge(XRI + 10, tub_face_in(SHELF_Z0) - 1, -WI, -WI + 3, SHELF_Z0, axis='y', into=+1))
    zf = SHELF_Z0
    gus = Polygon([(tub_face_in(zf) + 0.2, zf), (tub_face_in(zf) - 3.5, zf), (tub_face_in(zf) - 3.5, zf - 0.8),
                   (tub_face_in(zf - 6) + 0.2, zf - 6.5)])
    t = t.union(prism_y(gus, -WI, BOARD_Y1 - 2))
    # 5 V regulator holder: open frame on the floor ahead of the left rear motor, board standing on edge
    rx0, rx1 = REG['x']; ry0, ry1 = REG['y']
    t = t.union(bx(rx0 - 1.2 - CLR, rx1 + CLR + 1.2, ry0 - 1.2 - CLR, ry1 + CLR + 1.2, FLOOR - 0.5, FLOOR + 8).cut(
        bx(rx0 - CLR, rx1 + CLR, ry0 - CLR, ry1 + CLR, FLOOR, FLOOR + 10)))
    # E-stop: Ø22.4 panel hole + 3.2 anti-rotation notch through the rear wall
    ey, ez = sum(ESTOP['y']) / 2, sum(ESTOP['z']) / 2
    t = t.cut(prism_x(teardrop2d(ey, ez, 22.4 / 2), XRO - 1, XRI + 1))                 # teardrop: the nut hides the point
    t = t.cut(bx(XRO - 1, XRI + 1, ey - 1.6, ey + 1.6, ez, ez + 24.1 - 11.2))
    # Pi fan air: vent slots in both side walls between the Pi and the shelf
    for i in range(6):
        vx = OPI_X0 + 6 + i * 9.5
        for s in (1, -1):
            a, b = sorted((s * (WI - 1), s * (W + 1)))
            t = t.cut(prism_y(Point(vx, OPI_Z1 + 3).buffer(1.6, 16).union(sbox(vx - 1.6, OPI_Z1 + 3, vx + 1.6, SHELF_Z0 - 3)).union(
                teardrop2d(vx, SHELF_Z0 - 3, 1.6)), a, b))
    # cap bosses: 2 per side wall, clear of the board and the E-stop
    TUB_BOSSES.clear()
    for xx in (XRI + 12.0, tub_face_in(H_IN) - 6.0):
        for s in (1, -1):
            t, h = cap_boss(t, xx, s * WI, -s); TUB_BOSSES.append(h)
    for yy in (8.0, -58.0):                     # rear wall (= the rear hatch): the tub cap clamps the hatch; clear of E-stop + jack
        t, h = cap_boss(t, yy, XRI, +1, along='y'); TUB_BOSSES.append(h)
    # hinge: inner + outer cups with the pin fused through them, pocket for the shell knuckle
    for s in (1, -1):
        for (y0, y1) in CUPS_Y:
            a, b = sorted((s * y0, s * y1))
            t = t.union(prism_y(cup2d(), a, b))
            t = t.union(prism_y(sbox(J_TUB - 14.0, BELLY + 0.01, J_TUB, ZH + 6), a, b))
        a, b = sorted((s * (KN_Y[0] - GAP), s * (KN_Y[1] + GAP)))
        t = t.cut(prism_y(tub_knuckle_pocket2d(), a, b))
        a, b = sorted((s * 30.0, s * W))
        t = t.union(prism_y(pin2d(), a, b))
    t = t.cut(prism_x(CABLE_PASS, J_TUB - 12, J_TUB + 1))                                 # cable pass through the joint
    t = t.cut(prism_y(Polygon([(J_TUB + 0.1, BELLY - 0.1), (J_TUB - 0.9, BELLY - 0.1), (J_TUB + 0.1, BELLY + 0.9)]), -W - 2, W + 2))  # no first-layer weld
    for (dx, dy) in ((-17, 0), (38, 55), (38, -55)):
        t = t.cut(cyl_z(dx, dy, 2, BELLY - 1, FLOOR + 1))
    t = estop_guard(t)
    # charge inlet hole (DC-099) through the rear wall, above the board
    t = t.cut(prism_x(teardrop2d(JACK['y'], JACK['z'], JACK['r_hole']), XRO - 1, XRI + 1))
    t = as5600_tub(t)
    t = strain_reliefs(t, 'tub')
    return t

def main_tub():
    """tub without its rear hatch: the dry stack (Pi deck, Pi, shelf, board) slides in straight from behind"""
    return main_tub_full().cut(rear_region())

def rear_hatch():
    """rear wall above the deck with the E-stop (main switch), its guard fins and the charge jack. Held by the tub cap on
    top and a 1.6 mm lip that drops inside the rear-wall stub; its own ledges carry the deck's and the shelf's rear edges"""
    h = main_tub_full().intersect(rear_region())
    LIP_Y = (32.0, WI - R_V)                                   # clear of the Pi deck (Y <= 28.8) and the shelf (Y <= 21.5)
    lip = bx(XRI - 1.2, XRI + CLR + 1.6, LIP_Y[0], LIP_Y[1], REAR_HATCH_Z0, REAR_HATCH_Z0 + 2.0).union(
        bx(XRI + CLR, XRI + CLR + 1.6, LIP_Y[0], LIP_Y[1], REAR_HATCH_Z0 - 3.0, REAR_HATCH_Z0 + 0.1))   # drops inside the stub
    h = h.union(lip)
    big = max(h.solids().vals(), key=lambda so: so.Volume())          # drop slivers cut off at the boundary
    return cq.Workplane(obj=big)

def estop_guard(t):
    """two fins either side of the E-stop head, 45 deg underside (prints with no support), stop 3 mm short of its top"""
    ey, ez = sum(ESTOP['y']) / 2, sum(ESTOP['z']) / 2
    g = ESTOP_GUARD
    reach = ESTOP['head_h'] - g['short']
    for s in (1, -1):
        y0 = ey + s * (ESTOP['head_r'] + g['gap']); a, b = sorted((y0, y0 + s * g['t']))
        prof = Polygon([(XRO + 0.6, ez - g['half_h'] - reach), (XRO - reach, ez - g['half_h']), (XRO - reach, ez + g['half_h']),
                        (XRO + 0.6, ez + g['half_h'])])
        prof = round_convex(prof, 6.0, sbox(XRO - reach - 1, ez - 100, XRO - 0.5, ez + 100))   # R6 outer corners
        fin = prism_y(prof, a, b)
        t = t.union(fin)
    return t

def as5600_tub(t):
    """slot for the 23 x 23 board standing in an XZ plane at Y y_chip, chip on the hinge axis; coaxial pocket for the
    shell's magnet lug (0.4 print-in-place gap, teardrop roof)"""
    b, ys = AS['board'], AS['y_chip']
    pocket = Point(XH, ZH).buffer(AS['lug_r'] + GAP, 64).union(cap_up(AS['lug_r'] + GAP))
    t = t.cut(prism_y(pocket, AS['y_lug0'] - GAP, AS['y_mag'] + GAP))
    # board slot: through the joint wall, plus a 1 mm recess in the floor under the board
    t = t.cut(bx(XH - b / 2 - CLR, J_TUB + 1, ys - AS['chip_h'] - CLR, ys + AS['pcb'] + CLR, BELLY - 1.0, ZH + b / 2 + CLR))   # in from below
    # holder: two rails on the tub side gripping the board's -X edge region (board slides in from above)
    for (y0, y1) in ((ys - CLR - 2.0, ys - CLR), (ys + AS['pcb'] + CLR, ys + AS['pcb'] + CLR + 2.0)):
        rail = bx(XH - b / 2 - CLR - 2.0, XH - b / 2 + 4.0, y0, y1, FLOOR - 0.5, ZH + b / 2)
        t = t.union(rail)
    t = t.union(bx(XH - b / 2 - CLR - 2.0, XH - b / 2 - CLR, ys - CLR - 2.0, ys + AS['pcb'] + CLR + 2.0, FLOOR - 0.5, ZH + b / 2))
    return t

def strain_reliefs(w, which):
    """zip-tie anchor bridges on the floor either side of the cable pass (cables tied at the hinge axis height)"""
    x = J_TUB - 18.0 if which == 'tub' else J_SH + 16.0
    for yy in (-6.0, 6.0):
        br = prism_x(Polygon([(yy - 3.0, FLOOR - 0.5), (yy + 3.0, FLOOR - 0.5), (yy + 3.0, FLOOR + 3.5), (yy, FLOOR + 6.5),
                              (yy - 3.0, FLOOR + 3.5)]), x - 2.0, x + 2.0)
        tunnel = prism_x(Polygon([(yy - 1.8, FLOOR - 0.6), (yy + 1.8, FLOOR - 0.6), (yy + 1.8, FLOOR + 1.6), (yy, FLOOR + 3.4),
                                  (yy - 1.8, FLOOR + 1.6)]), x - 3.0, x + 3.0)
        w = w.union(br).cut(tunnel)
    return w

@functools.lru_cache(None)
def shell_raw():
    po, pi = shell_plans()
    outer, inner = body(shell_profile(), po, pi)
    return outer.cut(inner), inner

def load_tubes():
    """solved tube centrelines (tube_router.py output): name -> list of (x, y, z). Empty if not routed yet."""
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tubes_v3.json')
    if not os.path.exists(p):
        return {}
    d = json.load(open(p))
    return {k: [tuple(q) for q in v] for k, v in d['paths'].items()}
TUBES = load_tubes()

def tube_solid(pts, r, step=8.0):
    """circle swept along a smooth spline through the routed centreline (one clean solid, for ghosts and clearance cuts)"""
    import numpy as np
    P = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1); s_ = np.concatenate([[0], np.cumsum(seg)])
    n = max(3, int(math.ceil(s_[-1] / step)) + 1)
    t = np.linspace(0, s_[-1], n)
    Q = np.stack([np.interp(t, s_, P[:, k]) for k in range(3)], 1)
    edge = cq.Edge.makeSpline([cq.Vector(*q) for q in Q])
    path = cq.Wire.assembleEdges([edge])
    t0 = edge.tangentAt(0)
    prof = cq.Wire.makeCircle(r, cq.Vector(*Q[0]), t0)
    solid = cq.Solid.sweep(prof, [], path, makeSolid=True, isFrenet=False, mode=None, transitionMode='right')
    return cq.Workplane(obj=solid)

def noz_axis():
    c, s_ = math.cos(math.radians(NOZ['angle'])), math.sin(math.radians(NOZ['angle']))
    return NOZ['rear'], (c, 0.0, -s_)

def noz_z(x):
    (x0, _, z0), (dx, _, dz) = noz_axis()
    return z0 + (x - x0) * dz / dx

def nozzle_solid(r_extra=0.0):
    """rigid nozzle: barb (under the tube) + Luer adapter + tip hub (r 4.75) and the 14 ga needle (r 1.1)"""
    (x0, y0, z0), d = noz_axis()
    L1 = NOZ['barb'] + NOZ['body']
    back = 15.0 if r_extra > 0 else 0.0
    x0, z0 = x0 - d[0] * back, z0 - d[2] * back
    a = cq.Workplane(obj=cq.Solid.makeCylinder(NOZ['r'] + r_extra, L1 + back, cq.Vector(x0, y0, z0), cq.Vector(*d)))
    L1 += back
    p1 = (x0 + d[0] * L1, y0, z0 + d[2] * L1)
    return a.union(cq.Workplane(obj=cq.Solid.makeCylinder(1.1 + r_extra, NOZ['needle'], cq.Vector(*p1), cq.Vector(*d))))

# --- nose bay supports ---
X_RIB0 = XF + M_GB_R + 1.5                 # ahead of the front motors a wall may reach the floor
BATT_RIBS = (BAT_SLOT[0] + 9.0, sum(BAT_SLOT) / 2, BAT_SLOT[1] - 9.0)
TANK_RIBS = (TANK_SLOT[0] + 6.0, TANK_SLOT[1] - 6.4)
VL53_X, VL53_Y = NOSE_I - 14.0, sum(TANK_RIBS) / 2
SERVO_EAR = (SERVO_X1 - SERVO['body'][0] - (SERVO['ears'] - SERVO['body'][0]) / 2, SERVO_X1 - SERVO['body'][0],
             SERVO_Y[1] - SERVO['ear_down'] - 2.5, SERVO_Y[1] - SERVO['ear_down'])          # rear ear X0, X1, Y0, Y1
# ESP32 cross-section along Y (from the wall side): pins, header plastic, PCB, components (Espressif drawing)
ESP_LAYERS = dict(pins=(ESP_X[0], ESP_X[0] + 6.0), plastic=(ESP_X[0] + 6.0, ESP_X[0] + 8.5), pcb=(ESP_X[0] + 8.5, ESP_X[0] + 10.1),
                  comp=(ESP_X[0] + 10.1, ESP_X[1]))           # along X: pins face the bay, components face the nose wall
ESP_RAIL_Z1 = ESP_Z0 + 50.0
WIN = None                                             # refill-hose window: computed below from the modelled hose
DRAINS_SHELL = [(100.0, 0.0), (116.0, -60.0), (116.0, -16.0), (116.0, 40.0), (116.0, 62.0)]
DRAIN_R = 1.75

@functools.lru_cache(None)
def front_shell():
    raw, cav = shell_raw()
    f = raw.intersect(BELOW())
    f = motor_mount(f, XF)
    f = channel(f, J_SH + T + 1, XF - M_GB_R - 1)
    # pump deck: rear-wall ledge, side-wall ledges behind the pads (pad tops carry it over the motors)
    f = f.union(ledge(J_SH + T, J_SH + T + 3, -WI + 4, WI - 4, DECK_Z0, axis='x', into=+1))
    for s in (1, -1):
        y0, y1 = sorted((s * WI, s * (WI - 3)))
        f = f.union(ledge(J_SH + T + 14, XF - PAD_HALF, y0, y1, DECK_Z0, axis='y', into=-s))
    # ribs: battery on rib tops at NOSE_Z0, tank lower (its back is over the motor can)
    for yc in BATT_RIBS:
        f = f.union(bx(X_RIB0, NOSE_I + 0.5, yc - 0.8, yc + 0.8, FLOOR - 0.5, BATT_Z[0]).intersect(cav))
    for yc in TANK_RIBS:
        f = f.union(bx(X_RIB0, NOSE_I + 0.5, yc - 0.8, yc + 0.8, FLOOR - 0.5, TANK_Z[0]).intersect(cav))
    f = f.cut(bx(S2_X0, XF + PAD_HALF + 1.0, BAT_SLOT[0], WI - 0.01, BATT_Z[0] - CLR, DECK_Z0 + 1.0))   # pad corner under the battery
    # splash walls: bay | tank and tank | battery, full height ahead of the motors (tube passages cut below)
    for yb in (BAY[1], TANK_SLOT[1]):
        f = f.union(bx(X_RIB0 - 0.01, NOSE_I + 0.5, yb, yb + DIV, FLOOR - 0.5, H_IN).intersect(cav))
        f = f.union(bx(S2_X0 - 0.01, X_RIB0, yb, yb + DIV, 100.0, H_IN).intersect(cav))   # upper part over the motor
    f = servo_mount(f, cav)
    pass                                                   # the ESP32 carrier rests on the servo case
    f = nozzle_mount(f, cav)
    # refill-hose window in the right wall, pointed roof
    xm = sum(WIN['x']) / 2
    f = f.cut(prism_y(unary_union([rrect(WIN['x'][0], WIN['x'][1], WIN['z'][0], WIN['z'][1], (4, 4, 4, 4)),
                                   Polygon([(WIN['x'][0] + 4, WIN['z'][1] - 1), (WIN['x'][1] - 4, WIN['z'][1] - 1), (xm, WIN['z'][1] + 9)])]),
                      -W - 5, -WI + 0.5))
    # XY-MOS x2 standing in the bay behind the front motors (under the sealed pump-deck tray)
    for yc in XYMOS_Y:
        x0, x1 = XYMOS_X[0] - CLR, XYMOS_X[1] + CLR
        y0, y1 = yc - 17 - CLR, yc + 17 + CLR
        f = f.union(bx(x0 - 1.2, x1 + 1.2, y0 - 1.2, y1 + 1.2, FLOOR - 0.5, FLOOR + 8).cut(bx(x0, x1, y0, y1, FLOOR, FLOOR + 10)))
    # VL53L0X: window through the 45 deg nose slope, sensor lies on the slope looking 45 deg down-forward
    vx, vy = VL53_X, VL53_Y
    vz = NOSE_SLOPE_IN(vx)
    f = f.cut(bx(-4.5, 4.5, -4.0, 4.0, -6, 6).rotate((0, 0, 0), (0, 1, 0), -45).translate((vx + 1.2, vy, vz - 1.2)))
    for dy in (-7.6, 7.6):
        rail = bx(-14, 14, -1.0, 1.0, 0, 2.2).rotate((0, 0, 0), (0, 1, 0), -45).translate((vx, vy + dy, vz))
        f = f.union(rail.intersect(cav))
    # cap bosses: rear wall on the dry left side, both front corners
    SHELL_BOSSES.clear()
    f, h = cap_boss(f, WI - 4.1, J_SH + T, +1, along='y'); SHELL_BOSSES.append(h)
    f, h = cap_boss(f, -22.0, NOSE_I, -1, along='y'); SHELL_BOSSES.append(h)        # front wall, ahead of the tank
    f, _ = cap_boss(f, BATT_BOSS[0], WI, -1)                                           # battery hatch, behind the battery bay
    # hinge: knuckles with teardrop bore, pockets for the tub's cups (cut through the side wall too)
    for s in (1, -1):
        a, b = sorted((s * KN_Y[0], s * KN_Y[1]))
        f = f.union(prism_y(knuckle2d(), a, b))
        for (y0, y1) in CUPS_Y:
            a, b = sorted((s * (y0 - GAP), s * (y1 + (GAP if y1 < W else 2))))
            f = f.cut(prism_y(shell_cup_pocket2d(), a, b))
        a, b = sorted((s * (KN_Y[0] - 1), s * (KN_Y[1] + 1)))
        f = f.cut(prism_y(shell_bore2d(), a, b))
    f = f.cut(prism_x(CABLE_PASS, J_SH - 1, J_SH + T + 1))                               # cable pass
    f = f.cut(prism_y(Polygon([(J_SH - 0.1, BELLY - 0.1), (J_SH + 0.9, BELLY - 0.1), (J_SH - 0.1, BELLY + 0.9)]), -W - 2, W + 2))
    f = as5600_shell(f)
    f = strain_reliefs(f, 'shell')
    for (dx, dy) in DRAINS_SHELL:                                                         # wet-zone low points
        f = f.cut(cyl_z(dx, dy, DRAIN_R, BELLY - 1, FLOOR + 2))
    for name, pts in TUBES.items():                                                       # tube passages through ribs/walls
        f = f.cut(tube_solid(pts, TUBE_R + CLR + 0.3).intersect(bx(S2_X0 - 1, NOSE_I + 0.4, -WI + 0.3, WI - 0.3, FLOOR, H_IN)))
    for (fx, fy, fz) in TANK_FIT_PTS:                                                     # fitting stubs through the wall
        if fy < TANK_SLOT[0] + 1.0:
            f = f.cut(cyl_y(fx, fz, 4.0 + CLR, BAY[1] - 1.0, TANK_SLOT[0] + 1.0))
            f = f.cut(bx(fx - 4.0 - CLR, fx + 4.0 + CLR, BAY[1] - 1.0, TANK_SLOT[0] + 1.0, fz, H_IN + 1))   # open upward
    return f

def servo_mount(f, cav):
    """MG90S lying, case top on the right wall, spline + horn hub through a teardrop hole; rear ear screwed (M2) to a post
    on the wall (45 deg underside); a rib from the slope cradles the body; the front ear is cut off (it would hit the nose)"""
    f = f.cut(teardrop_y(SERVO_PX, SERVO_PZ, 5.5, -W - 1, -WI + 0.5))
    sw = 3.0                                                   # spline slot: the servo drops straight in from above
    f = f.cut(prism_y(Polygon([(SERVO_PX - sw, SERVO_PZ), (SERVO_PX + sw, SERVO_PZ), (SERVO_PX + sw, SERVO_TOP + 10.0),
                               (SERVO_PX, SERVO_TOP + 10.0 + sw), (SERVO_PX - sw, SERVO_TOP + 10.0)]), -W - 1, -WI + 0.5))
    e0, e1, ey0, ey1 = SERVO_EAR
    prof = Polygon([(-WI - 0.5, SERVO_Z0 - 9.0), (ey0, SERVO_Z0 - 1.5), (ey0, SERVO_TOP), (-WI - 0.5, SERVO_TOP)])
    f = f.union(prism_x(prof, e0, e1))
    f = f.cut(cyl_y((e0 + e1) / 2, SERVO_PZ, 0.9, -WI - 0.1, ey1 + 0.1))
    f = f.union(bx(SERVO_X1 - 18.0, SERVO_X1 - 2.0, SERVO_Y[0] + 14.0, SERVO_Y[0] + 15.6, FLOOR - 0.5, SERVO_Z0).intersect(cav))
    return f

ESP_CAR_Z0 = SERVO_TOP                                           # carrier plate rests on the servo case

def esp_support(f, cav):
    """gusset ledge along the bay | tank wall that the ESP32 carrier sits on (45 deg underside, clear of the servo)"""
    z0 = ESP_CAR_Z0 - 0.01
    w = BAY[1] - (SERVO_Y[1] + CLR)
    gus = Polygon([(SERVO_Y[1] + CLR, z0), (BAY[1] + 0.01, z0), (BAY[1] + 0.01, z0 - w)])
    return f.union(prism_x(gus, ESP_X[0] - CLR - 1.2, NOSE_I + 0.5))

def esp_carrier():
    """drop-in ESP32 carrier (fitted after the servo): plate on the gusset ledge, C-rails on BOTH long edges (back face on
    the header plastic, 0.8 lip on the component side, clear of the pins), splash wall on the wet side (toward the tubes)"""
    z0 = ESP_CAR_Z0
    xs = ESP_X[0] - CLR - 1.2
    y0, y1 = -WI + CLR, BAY[1] - CLR                                  # wall to wall: located by the side wall and the bay | tank wall
    c = bx(xs, NOSE_I - CLR, y0, y1, z0, z0 + 1.6)
    pl, lp = ESP_LAYERS['plastic'], ESP_LAYERS['comp']
    z1 = z0 + 1.6
    for edge, out in ((ESP_Y[0], -1), (ESP_Y[1], +1)):
        yo = (edge - 2.4, edge) if out < 0 else (edge, y1)
        yi = (edge, edge + 0.8) if out < 0 else (edge - 0.8, edge)
        c = c.union(bx(xs, NOSE_I - CLR, yo[0], yo[1], z1 - 0.01, ESP_RAIL_Z1))                          # web
        c = c.union(bx(xs, pl[0], min(yi), max(yi), z1 - 0.01, ESP_RAIL_Z1))                               # back part
        c = c.union(bx(lp[0], min(lp[0] + 1.45, NOSE_I - CLR), min(yi), max(yi), z1 - 0.01, ESP_RAIL_Z1))  # lip
    c = c.union(bx(xs, ESP_X[0] - CLR, y0, y1, z0, ESP_Z1 + 0.5))                                          # splash wall
    return c

def nozzle_mount(f, cav):
    """cradle hanging off the front wall under the tip hub (45 deg underside), needle exits through a teardrop hole and
    ends 0.6 behind the nose face; a zip tie through the cradle holds the hub down"""
    f = f.cut(nozzle_solid(CLR))
    xa, xb = TANK_X[1] + CLR + 0.5, NOSE_I + 0.5             # starts ahead of the tank
    za, zb = noz_z(xa), noz_z(xb)
    rr = NOZ['r'] + CLR + 2.0
    prof = Polygon([(xa, za), (xb, zb), (xb, za - rr - (xb - xa)), (xa, za - rr)])
    c = prism_y(prof, -rr, TANK_SLOT[1] + DIV).cut(nozzle_solid(CLR))
    c = c.cut(prism_y(Polygon([(xa - 1, za), (xb + 1, zb), (xb + 1, zb + 30), (xa - 1, za + 30)]), -rr - 1, rr + 1))   # open top
    xt = (xa + xb) / 2 - 3
    c = c.cut(bx(xt - 1.5, xt + 1.5, -rr - 1, rr + 1, noz_z(xt) - NOZ['r'] - CLR - 1.6, noz_z(xt) - NOZ['r'] - CLR))  # tie slot
    f = f.union(c)
    f = f.cut(prism_x(teardrop2d(0.0, noz_z(NOSE_I + 1.2), 2.4), NOSE_I - 1, NOSE + 1))
    return f

def as5600_shell(f):
    """magnet lug on the hinge axis (print-in-place in the tub's coaxial pocket), web to the shell's rear wall; swept
    clearance for the tub-mounted board's forward half"""
    lug = Point(XH, ZH).buffer(AS['lug_r'], 64)
    web = sbox(XH + 0.4, ZH - 4.0, J_SH + T + 2.0, ZH + 4.0)
    f = f.union(prism_y(unary_union([lug, web]), AS['y_lug0'], AS['y_mag']))
    f = f.cut(cyl_y(XH, ZH, AS['mag_r'] + 0.1, AS['y_mag'] - AS['mag_t'] - 0.1, AS['y_mag'] + 0.1))     # magnet pocket
    b = AS['board']
    board = sbox(XH - b / 2, ZH - b / 2, XH + b / 2, ZH + b / 2).buffer(CLR, join_style=2)
    f = f.cut(prism_y(sweep2d(board, -1), AS['y_chip'] - AS['chip_h'] - CLR, AS['y_chip'] + AS['pcb'] + CLR))
    f = f.cut(bx(J_SH - 1, XH + b / 2 + CLR, AS['y_chip'] - AS['chip_h'] - CLR, AS['y_chip'] + AS['pcb'] + CLR, BELLY - 1.0, ZH))
    return f

# ---------------- print 2 ----------------
def cap_of(raw, trim_at_joint):
    c = raw.intersect(ABOVE()).cut(trim_at_joint)
    return c

def shell_max_z(deg):
    """highest point of the shell's side profile with the shell lifted deg about the hinge (the cap is part of it)"""
    p = shell_profile()
    a = math.radians(deg)
    return max(ZH + (x - XH) * math.sin(a) + (z - ZH) * math.cos(a) for x, z in p.exterior.coords)

# LiDAR: scan plane >= 10 mm above the lifted shell at 20 deg, plus the beam's height near the robot. The D500/STL-19P
# datasheets give no vertical divergence; the 2 deg pitch tolerance alone is +-0.6 mm at 17 mm, so 5 mm is allowed.
LIDAR['beam'] = 5.0
LIDAR['scan_min'] = shell_max_z(20.0) + 10.0 + LIDAR['beam']
RGB = dict(len=97.0, w=12.0, t=1.8)                              # Yahboom 14-LED flexible strip (PH2.0-3P lead)
LIDAR['riser'] = max(RGB['w'] + 2 * 2.5 + 3.0,                    # the strip needs its own band of column (+ top plate)
                     math.ceil((LIDAR['scan_min'] - (H + LIDAR['plate'] + LIDAR['scan'])) * 2) / 2)
LIDAR['base'] = H + LIDAR['plate'] + LIDAR['riser']             # LiDAR base plane

def tub_cap():
    c = cap_of(tub_raw(), prism_y(Polygon([(tub_face_x(Z_SPLIT - 30) - 0.6, Z_SPLIT - 30), (200, Z_SPLIT - 30), (200, H + 50),
                                           (tub_face_x(H + 50) - 0.6, H + 50)]), -W - 5, W + 5))
    c = c.cut(main_tub_full())
    for (hx, hy) in TUB_BOSSES:
        c = c.cut(cyl_z(hx, hy, 1.7, H_IN - 1, H + 1))
    for (px, py) in LIDAR_PADS:                                       # LiDAR riser screws: pads under the cap (prints top-down)
        c = c.union(cyl_z(px, py, 3.5, H_IN - 5.0, H_IN + 0.1)).cut(cyl_z(px, py, 1.25, H_IN - 6, H + 1))
    c = c.cut(prism_z(rrect(*LIDAR_LEAD, (2, 2, 2, 2)), H_IN - 1, H + 1))
    return c

BATT_HATCH = (S2_X0 - DIV - 1.0, TANK_SLOT[1] + DIV / 2)         # x0, y0 of the battery hatch: its rear edge covers the
                                                                  # 9 V cradle's rear wall (which sits over the nose-bay wall)
TANK_HATCH = (S2_X0 - DIV / 2, BAY[1] + DIV / 2, TANK_SLOT[1] + DIV / 2)
NINE_Z0 = BATT_Z[1] + CLR + 0.8                                   # 9 V cells lie flat on the hatch cradle's floor

BATT_BOSS = (S2_X0 - DIV - CLR - 3.8, WI - 3.0)                    # hole centre of the battery hatch's boss

def batt_hatch_box(shrink):
    return bx(BATT_HATCH[0] + shrink, NOSE + 10, BATT_HATCH[1] + shrink, W + 10, Z_SPLIT - 1, H + 5).union(
        bx(BATT_BOSS[0] - 6.0 + shrink, BATT_HATCH[0] + shrink + 0.5, WI - 10.0 + shrink, W + 10, Z_SPLIT - 1, H + 5))   # tab

def tank_hatch_box(shrink):
    return bx(TANK_HATCH[0] + shrink, TANK_X[1] + CLR + DIV / 2 - shrink, TANK_HATCH[1] + shrink, TANK_HATCH[2] - shrink,
              Z_SPLIT - 1, H + 5)

@functools.lru_cache(None)
def shell_cap():
    c = cap_of(shell_raw()[0], bx(J_SH - 10, J_SH + 0.6, -W - 5, W + 5, Z_SPLIT - 1, H + 5))
    c = c.cut(batt_hatch_box(0.0)).cut(tank_hatch_box(0.0)).cut(front_shell())
    for (hx, hy) in SHELL_BOSSES:
        c = c.cut(cyl_z(hx, hy, 1.7, H_IN - 1, H + 1))
    # ESP32 USB slot right over the board's top end: plug in with nothing removed
    c = c.cut(prism_z(rrect(ESP_X[0] + 1.0, ESP_X[1] - 1.0, ESP_Y[0] + 3.0, ESP_Y[1] - 3.0, (2, 2, 2, 2)), H_IN - 1, H + 1))
    return c

@functools.lru_cache(None)
def batt_hatch():
    """'the small hatch': the cap over the battery. Underneath it hangs the 9 V cradle, so lifting it brings both 9 V
    cells (and their USB charge ports) up with it; the battery below lifts straight out after it"""
    c = shell_raw()[0].intersect(ABOVE()).intersect(batt_hatch_box(CLR)).cut(front_shell())
    c = c.cut(cyl_z(BATT_BOSS[0], BATT_BOSS[1], 1.7, H_IN - 1, H + 1))
    x0, x1 = NINE_X
    y0, y1 = NINE_Y[0][0] - CLR - 1.2, NINE_Y[1][1] + 1.6
    zf = NINE_Z0 - 0.8
    tray = bx(x0 - 1.2, x1 + 1.6, y0, y1, zf, zf + 0.8)
    for (xa, xb) in ((x0 - 1.2, x0), (x1, x1 + 1.6)):                 # end walls carry the tray from the hatch
        tray = tray.union(bx(xa, xb, y0, y1, zf, H_IN + 0.5))
    for (ya, yb) in NINE_Y:                                             # port cut-outs in the end walls (port face unknown)
        tray = tray.cut(bx(x0 - 2, x0 + 0.1, ya + 3, yb - 3, NINE_Z0 + 2, NINE_Z0 + NINE[2] - 2))
    tray = tray.union(bx(x0, x1, NINE_Y[0][1] + CLR, NINE_Y[1][0] - CLR, zf, NINE_Z0 + 6.0))   # divider rib
    tray = tray.union(bx(x0 - 1.2, x1 + 1.6, y0, NINE_Y[0][0] - CLR, zf, H_IN + 0.5))           # splash wall toward the nozzle
    return c.union(tray)

@functools.lru_cache(None)
def tank_hatch():
    """cap piece over the tank: the tank lifts out once the nozzle is lifted from its cradle and the tubes pulled off"""
    return shell_raw()[0].intersect(ABOVE()).intersect(tank_hatch_box(CLR)).cut(front_shell())

def tub_deck():
    d = prism_z(rrect(XRI + 0.3, TUB_DECK_X1, -WI + 0.3, TUB_DECK_Y1, (max(0.5, R_V - T - 0.3), 1, 2, 2)), DECK_Z0, DECK_Z1)
    fr = bx(OPI_X0 - CLR - 1.6, OPI_X1 + CLR + 1.6, OPI_Y0 - CLR - 1.6, OPI_Y1 - 2.0, DECK_Z1 - 0.1, DECK_Z1 + 6).cut(
        bx(OPI_X0 - CLR, OPI_X1 + CLR, OPI_Y0 - CLR, OPI_Y1 + 5, DECK_Z1 - 1, DECK_Z1 + 7))
    d = d.union(fr)
    cx, cy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    for dy in (-32.5, 32.5):
        d = d.cut(cyl_z(cx, cy + dy, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))
    for (px, py) in ((XRI + 10.0, TUB_DECK_Y1 - 4.0), (TUB_DECK_X1 - 5.0, TUB_DECK_Y1 - 4.0)):
        d = d.cut(cyl_z(px, py, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))
    return d.cut(prism_z(rrect(cx - 22, cx + 22, cy - 30, cy + 30, (6, 6, 6, 6)), DECK_Z0 - 1, DECK_Z1 + 1))

def tub_shelf():
    x1 = tub_face_in(SHELF_Z1) - 0.3
    y1 = BOARD_Y1 + 1.0
    s = prism_z(rrect(XRI + 0.3, x1, -WI + 0.3, y1, (max(0.5, R_V - T - 0.3), 1, 1, 1)), SHELF_Z0, SHELF_Z1)
    s = s.union(bx(XRI + 4, x1 - 4, y1 - 2.0, y1, SHELF_Z0 - 7, SHELF_Z0))
    for (hx, hy) in BOARD_HOLES:
        s = s.cut(cyl_z(hx, hy, 1.35, SHELF_Z0 - 1, BOARD_Z0 + 1))
    fx, fy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    return s.cut(prism_z(rrect(fx - 18, fx + 18, fy - 18, fy + 18, (4, 4, 4, 4)), SHELF_Z0 - 1, SHELF_Z1 + 1))

def pump_deck():
    """sealed tray under the pumps (lips on three sides, 2 mm): a leak runs forward over its open front edge onto the
    nose floor and out of the belly drains, never down onto the motors' plugs or the XY-MOS boards below. It carries the
    nose-bay rear wall (tube passages cut where the routed tubes cross)"""
    x0, x1 = J_SH + T + 0.3, S2_X0 - 0.3
    d = prism_z(rrect(x0, x1, -WI + 0.3, WI - 0.3, (1, 1, 1, 1)), DECK_Z0, DECK_Z1)
    lip = 2.0
    for s in (1, -1):
        a, b = sorted((s * (WI - 0.3), s * (WI - 1.5)))
        d = d.union(bx(x0, P_X1, a, b, DECK_Z1 - 0.1, DECK_Z1 + lip))
    w0 = P_X1 + CLR
    d = d.union(bx(w0, x1, -WI + 0.3, WI - 0.3, DECK_Z1 - 0.1, H_IN - 0.05))           # nose-bay rear wall
    for yy in (-55.0, -20.0, 20.0, 50.0):                                               # drain notches at its foot
        d = d.cut(bx(w0 - 1, x1 + 1, yy - 2.0, yy + 2.0, DECK_Z1 - 0.2, DECK_Z1 + 1.6))
    # gussets on the pump side where no pump/tube is: the dry left column
    for (ya, yb) in ((P_Y[1] + CLR, WI - 0.3),):
        d = d.union(prism_y(Polygon([(w0 + 0.01, DECK_Z1), (w0 - 8, DECK_Z1 + 8), (w0 + 0.01, DECK_Z1 + 8)]), ya, yb).union(
            bx(w0 - 2.5, w0 + 0.01, ya, yb, DECK_Z1, H_IN - 0.05)))
    for yy in (P_Y[0] + 22.0, P_Y[1] - 22.0):                                           # two straps round the pump stack
        for xx in (x0 + 2.0, w0 - 9.8):
            d = d.cut(bx(xx, xx + 3.0, yy - 3.5, yy + 3.5, DECK_Z0 - 1, DECK_Z1 + 1))
    d = d.cut(prism_z(rrect(XF - 10, XF + 6, -6, 6, (2, 2, 2, 2)), DECK_Z0 - 1, DECK_Z1 + 1))   # plug access (under the pump)
    bx_, by_ = SHELL_BOSSES[0] if SHELL_BOSSES else (J_SH + T + 3.0, WI - 4.1)
    d = d.cut(bx(J_SH, bx_ + 3.5 + CLR, by_ - 3.5 - CLR, WI + 1, DECK_Z0 - 1, H_IN + 1))       # clears the rear cap boss on the way in
    d = d.cut(bx(BATT_BOSS[0] - 3.5 - CLR, BATT_BOSS[0] + 3.5 + CLR, WI - 6.0 - CLR, WI + 1, DECK_Z0 - 1, H_IN + 1))   # and the battery hatch's boss
    for name, pts in TUBES.items():
        d = d.cut(tube_solid(pts, TUBE_R + CLR + 0.3).intersect(bx(w0 - 1, x1 + 1, -WI, WI, DECK_Z1, H_IN)))
    d = d.cut(bx(w0 - 1, x1 + 1, NINE_Y[0][0] - 1.6 - CLR, NINE_Y[1][1] + 1.6 + CLR, NINE_Z0 - 0.8 - CLR, H_IN + 1))   # 9 V cradle notch
    return d.cut(shell_cap()).cut(batt_hatch()).cut(tank_hatch())       # its wall stays under the caps' rounded inner edges

def lidar_mount():
    """riser for the D500: base plate screwed (4 x M3) to the cap pads, a hollow rounded column, top plate with the
    3 x M2 holes (Ø2.5 in the LiDAR: M2, heads in its Ø4.46 counterbores). The RGB strip wraps the column front + sides
    in a 1.8 mm recess, below the scan plane. Column height sets the scan plane >= 10 above the shell lifted 20 deg."""
    cx, z0 = LIDAR['cx'], H
    fx, fy = LIDAR['fp'][0] / 2, LIDAR['fp'][1] / 2
    plate = rrect(cx - fx - 1.5, cx + fx + 5.5, -fy - 1.5, fy + 1.5, (5, 5, 5, 5))
    m = prism_z(plate, z0, z0 + LIDAR['plate'])
    zt = z0 + LIDAR['plate'] + LIDAR['riser']
    if LIDAR['riser'] > 0:
        col = rrect(cx - 20.0, cx + 20.0, -22.0, 22.0, (6, 6, 6, 6))
        m = m.union(prism_z(col, z0 + LIDAR['plate'] - 0.1, zt - 3.0))
        m = m.union(prism_z(plate, zt - 3.0, zt))
        m = m.cut(prism_z(rrect(cx - 16.0, cx + 16.0, -18.0, 18.0, (3, 3, 3, 3)), z0 + 2.0, zt - 3.0))   # hollow
        rz = z0 + LIDAR['plate'] + (LIDAR['riser'] - 3.0 - RGB['w']) / 2
        m = m.cut(prism_z(col.difference(col.buffer(-RGB['t'], join_style=2)).intersection(sbox(cx - 25, -30, cx + 21, 30)),
                          rz, rz + RGB['w'] + 0.4))                                                     # RGB strip recess
    for (hx, hy) in LIDAR_HOLES:
        m = m.cut(cyl_z(hx, hy, 1.1, zt - 3.2, zt + 1))
    for (px, py) in LIDAR_PADS:
        m = m.cut(cyl_z(px, py, 1.7, z0 - 1, z0 + LIDAR['plate'] + 1)).cut(cyl_z(px, py, 3.0, z0 + LIDAR['plate'] - 1.8, z0 + LIDAR['plate'] + 1))
    m = m.cut(prism_z(rrect(*LIDAR_LEAD, (2, 2, 2, 2)), z0 - 1, zt + 1))
    return m

# ---------------- refill arm (outside the right wall, ahead of the front wheel) ----------------
ARM = dict(len=80.0, w=15.0, t=15.0, back=7.5, y=(-W - 2.5 - 15.0, -W - 2.5), strainer=(6.0, 10.0))  # strainer Ø12 x 10
ARM['reach'] = ARM['len'] - ARM['back']

def arm_angle_deployed():
    """angle below horizontal at which the strainer's lowest point is 1 mm off the ground (forward-down)"""
    best = None
    for a10 in range(300, 900):
        a = math.radians(a10 / 10)
        tip = (SERVO_PX + (ARM['reach'] + ARM['strainer'][1]) * math.cos(a), SERVO_PZ - (ARM['reach'] + ARM['strainer'][1]) * math.sin(a))
        low = tip[1] - ARM['strainer'][0] * math.cos(a) - 0.0
        if low <= 1.0:
            best = a10 / 10; break
    return best

ARM['deployed_deg'] = arm_angle_deployed()                       # below horizontal, forward
ARM['stowed_deg'] = -90.0                                        # pointing straight up

def arm_solid(deg_below, with_strainer=True):
    """printed arm (80 x 15 x 15) on the servo horn: horn pocket on its inner face, hose groove on its outer face"""
    a = cq.Workplane('XZ').center(ARM['reach'] / 2 - ARM['back'] / 2, 0).rect(ARM['len'], ARM['w']).extrude(-(ARM['y'][1] - ARM['y'][0]))
    a = a.translate((0, ARM['y'][0], 0))
    try:
        a = a.edges('|Y').fillet(6.0)                                  # R6 on the long edges (brief: R6-R8 on everything new)
    except Exception:
        a = a.edges('|Y').fillet(3.0)
    a = a.cut(cyl_y(0, 0, 4.2, ARM['y'][1] - 3.0, ARM['y'][1] + 1))                                 # horn hub
    a = a.cut(bx(0, 18.0, ARM['y'][1] - 2.0, ARM['y'][1] + 1, -3.0, 3.0))                           # horn arm
    a = a.cut(cyl_y(0, 0, 1.1, ARM['y'][0] - 1, ARM['y'][1] + 1))                                   # horn screw
    a = a.cut(cyl_x(ARM['y'][0] - 0.01, 0, TUBE_R + 0.3, HOSE_S0, ARM['reach'] + 1))   # hose seat, to 8 mm from the pivot
    if with_strainer:
        a = a.union(cyl_x(sum(ARM['y']) / 2, 0, ARM['strainer'][0], ARM['reach'], ARM['reach'] + ARM['strainer'][1]))
    return a.rotate((0, 0, 0), (0, 1, 0), deg_below).translate((SERVO_PX, 0, SERVO_PZ))

def coupling_spacer():
    """sets the wheel 4 mm off the body: sits on the shaft between the boss face and the hex coupling"""
    return cq.Workplane('XY').circle(6.0).circle(3.2).extrude(WHEEL_SPACER)

# ---------------- components (ghosts) ----------------
WHEEL_W = 31.0                     # Yahboom 85 mm wheel measured 86.6 x 31 (retailer)
WHEEL_GAP = 4.0
WHEEL_Y0 = W + WHEEL_GAP           # wheel inner face
COUPLING = dict(len=18.0, af=12.0, recess=4.0)
WHEEL_SPACER = WHEEL_Y0 + COUPLING['recess'] - W                 # 8 mm: coupling end 4 mm inside the wheel hub
SHAFT_PAST_BOSS = 15.5
SHAFT_GRIP = W + SHAFT_PAST_BOSS - (W + WHEEL_SPACER)            # length of shaft inside the coupling

def components():
    """name -> (cq solid, body, bay, mass g, mass source, seat). body = 'tub' | 'shell' | 'wheel'.
    seat = how it is held: 'bottom', a dict, 'free' (nothing may touch) or None (not clearance-checked)."""
    C = {}
    def motor(xa, s):
        y0 = s * M_FACE; yg = s * (M_FACE - M_GB_L); yb = s * (M_FACE - M_LEN)
        g = cyl_y(xa, ZGB, M_GB_R, *sorted((y0, yg))).union(cyl_y(xa, ZGB, M_CAN_R, *sorted((yg, yb))))
        return g.union(cyl_y(xa, ZH, 6.0, *sorted((s * M_FACE, s * W)))).union(cyl_y(xa, ZH, 3.0, *sorted((s * W, s * (W + SHAFT_PAST_BOSS)))))
    for (xa, body, nm) in ((XR, 'tub', 'rear'), (XF, 'shell', 'front')):
        bay = 'tub_lower' if body == 'tub' else 'shell_lower'
        for s, side in ((1, 'L'), (-1, 'R')):
            C[f'motor_{nm}_{side}'] = (motor(xa, s), body, bay, 150, 'team', dict(face_y=s * M_FACE))
            C[f'wheel_{nm}_{side}'] = (cyl_y(xa, ZH, WHEEL_R_WORST, *sorted((s * WHEEL_Y0, s * (WHEEL_Y0 + WHEEL_W)))), 'wheel', None, 60, 'estimate', None)
    C['orange_pi_n515'] = (bx(OPI_X0, OPI_X1, OPI_Y0, OPI_Y1, OPI_Z0, OPI_Z1), 'tub', 'tub_upper', 200, 'estimate (Pi 5 Max + metal case)', 'bottom')
    C['yahboom_board'] = (bx(BOARD_X0, BOARD_X1, BOARD_Y0, BOARD_Y1 + 3.0, BOARD_Z0, BOARD_Z1), 'tub', 'tub_upper', 50, 'estimate', 'bottom')
    C['estop_body'] = (bx(XRI, XRI + ESTOP['depth'], *ESTOP['y'], *ESTOP['z']), 'tub', 'tub_upper', 60, 'estimate', dict(face_x=XRI))
    ey, ez = sum(ESTOP['y']) / 2, sum(ESTOP['z']) / 2
    C['estop_head'] = (cyl_x(ey, ez, ESTOP['head_r'], XRO - ESTOP['head_h'], XRO), 'tub', None, 0, '', None)
    C['charge_jack_dc099'] = (cyl_x(JACK['y'], JACK['z'], JACK['body_r'], XRI, XRI + JACK['depth']), 'tub', 'tub_upper', 6, 'estimate', dict(face_x=XRI))
    C['regulator_5v'] = (bx(*REG['x'], *REG['y'], *REG['z']), 'tub', 'tub_lower', 5, 'Pololu', 'bottom')
    b = AS['board']
    C['as5600_board'] = (bx(XH - b / 2, XH + b / 2, AS['y_chip'], AS['y_chip'] + AS['pcb'], ZH - b / 2, ZH + b / 2).union(
        bx(XH - 2.5, XH + 2.5, AS['y_chip'] - AS['chip_h'], AS['y_chip'], ZH - 2.5, ZH + 2.5)), 'tub', 'tub_lower', 3, 'estimate', None)
    C['as5600_magnet'] = (cyl_y(XH, ZH, AS['mag_r'], AS['y_mag'] - AS['mag_t'], AS['y_mag']), 'shell', None, 1, 'N35 6x2.5', None)
    cx, lz = LIDAR['cx'], LIDAR['base']
    C['lidar_d500'] = (bx(cx - LIDAR['fp'][0] / 2, cx + LIDAR['fp'][0] / 2, -LIDAR['fp'][1] / 2, LIDAR['fp'][1] / 2, lz, lz + 34.8 - 12.6).union(
        cyl_z(cx, 0, 35.29 / 2, lz + 34.8 - 12.6, lz + 34.8)), 'tub', None, 42, 'LD19 datasheet', None)
    for i in range(2):
        z0 = P_Z[i]
        C[('pump_lower', 'pump_upper')[i]] = (bx(P_X0, P_X1, P_Y[0], P_Y[1], z0, z0 + PUMP[2]), 'shell', 'shell_upper', 75, 'estimate', 'bottom')
    tank = bx(TANK_X[0], TANK_X[1], TANK_SLOT[0] + CLR, TANK_SLOT[1] - CLR, *TANK_Z)
    for (fx, fyy, fz) in TANK_FIT_PTS:                         # 90 deg elbow barbs on the tank top, outlets toward the bay
        fy = fyy + 7.0                                         # elbow's vertical leg 7 mm into the tank from the tube end
        tank = tank.union(cyl_z(fx, fy, 4.5, TANK_Z[1] - 0.01, fz)).union(cyl_y(fx, fz, 4.0, fyy + 1.0, fy))
    C['tank_water'] = (tank, 'shell', 'nose', 125, 'estimate (tank ~30 g + ~95 ml water)', 'bottom')
    C['battery_3s2p'] = (bx(S2_X0 + CLR, S2_X0 + CLR + BATT[0], BAT_SLOT[0] + CLR, BAT_SLOT[0] + CLR + BATT[1], *BATT_Z),
                         'shell', 'nose', 310, 'team', 'bottom')
    for i, (ya, yb) in enumerate(NINE_Y):
        held = [[NINE_X[0] - 1.0, NINE_X[0] + 1.0, ya - 1, yb + 1, NINE_Z0, NINE_Z0 + NINE[2]],      # cradle end walls
                [NINE_X[1] - 1.0, NINE_X[1] + 1.0, ya - 1, yb + 1, NINE_Z0, NINE_Z0 + NINE[2]],
                [NINE_X[0], NINE_X[1], NINE_Y[0][1] - 1.0, NINE_Y[1][0] + 1.0, NINE_Z0, NINE_Z0 + 6.5]]   # divider rib
        C[f'nine_volt_{i + 1}'] = (bx(NINE_X[0], NINE_X[1], ya, yb, NINE_Z0, NINE_Z0 + NINE[2]), 'shell', 'nose', 30, 'estimate',
                                   dict(boxes=held))
    pins, pl, pcb, comp = ESP_LAYERS['pins'], ESP_LAYERS['plastic'], ESP_LAYERS['pcb'], ESP_LAYERS['comp']
    e0, e1 = ESP_Y
    esp = bx(*pcb, e0, e1, ESP_Z0, ESP_Z1 - 6.2).union(bx(*comp, e0 + 1.0, e1 - 1.0, ESP_Z0, ESP_Z1))   # PCB + module/antenna
    for (a_, b_) in ((e0, e0 + 2.54), (e1 - 2.54, e1)):
        esp = esp.union(bx(*pl, a_, b_, ESP_Z0, ESP_Z1 - 6.2))
    for (a_, b_) in ((e0 + 0.95, e0 + 1.59), (e1 - 1.59, e1 - 0.95)):
        esp = esp.union(bx(*pins, a_, b_, ESP_Z0, ESP_Z1 - 6.2))
    C['esp32_s3'] = (esp, 'shell', 'nose', 10, 'estimate', dict(boxes=[[ESP_X[0] - 2, ESP_X[1] + 2, e0 - 1.0, e0 + 1.0, ESP_Z0, ESP_Z1],
                                                                       [ESP_X[0] - 2, ESP_X[1] + 2, e1 - 1.0, e1 + 1.0, ESP_Z0, ESP_Z1]]))
    for i, yc in enumerate(XYMOS_Y):
        C[f'xy_mos_{i + 1}'] = (bx(*XYMOS_X, yc - 17, yc + 17, FLOOR, FLOOR + 17), 'shell', 'shell_lower', 8, 'estimate', 'bottom')
    vz = NOSE_SLOPE_IN(VL53_X)
    C['vl53l0x'] = (bx(-12.5, 12.5, -6, 6, 0, 3.2).rotate((0, 0, 0), (0, 1, 0), -45).translate((VL53_X, VL53_Y, vz + CLR * 1.42)),
                    'shell', 'shell_lower', 1, 'datasheet', dict(plane=[-1 / math.sqrt(2), 0, 1 / math.sqrt(2), (vz - VL53_X) / math.sqrt(2)]))
    sb = SERVO['body']
    e0_, e1_, ey0, ey1 = SERVO_EAR
    servo = bx(SERVO_X1 - sb[0], SERVO_X1, SERVO_Y[0], SERVO_Y[1], SERVO_Z0, SERVO_TOP).union(
        bx(e0_, e1_, ey0, ey1, SERVO_Z0, SERVO_TOP)).union(cyl_y(SERVO_PX, SERVO_PZ, 2.4, -W - 1.6, SERVO_Y[0] + 0.01))
    C['servo_mg90s'] = (servo, 'shell', 'nose', 13, 'datasheet', dict(boxes=[[SERVO_X1 - sb[0] - 1, SERVO_X1 + 1, -WI - 1, -WI + 0.7, SERVO_Z0, SERVO_TOP],
                                                                             [e0_ - 1, e1_ + 1, -WI - 1, ey0 + 0.7, SERVO_Z0 - 1, SERVO_TOP + 1],
                                                                             [SERVO_X1 - 19, SERVO_X1 - 1, SERVO_Y[0] + 13, SERVO_Y[0] + 17, SERVO_Z0 - 1, SERVO_Z0 + 0.7],
                                                                             [SERVO_PX - 6, SERVO_PX + 6, -W - 2, -WI + 1, SERVO_PZ - 6, SERVO_PZ + 6],
                                                                             [ESP_X[0] - 2, NOSE_I, -WI, SERVO_Y[1], SERVO_TOP - 0.7, SERVO_TOP + 1]]))
    # 90 deg barbed elbow (on a short tube stub) on the spray pump's inlet barb, outlet pointing forward
    C['elbow_spray_in'] = (cyl_y(P_XC_BARB, BARBS['spray_in'], 4.5, P_Y[0] - 12.0, P_Y[0]).union(
        cyl_x(P_Y[0] - 10.0, BARBS['spray_in'], 4.5, P_XC_BARB, P_XC_BARB + 12.5)), 'shell', None, 2, 'estimate', None)
    C['nozzle_luer'] = (nozzle_solid(), 'shell', 'nose', 4, 'estimate', dict(boxes=[[NOSE_I - 15, NOSE + 1, -8, 8, 120, 160]]))
    C['refill_arm_stowed'] = (arm_solid(ARM['stowed_deg']), 'shell', None, 9, 'printed', None)
    C['refill_arm_deployed'] = (arm_solid(ARM['deployed_deg']), 'shell', None, 0, 'same arm, deployed', None)
    rx = LIDAR['cx']
    C['rgb_bar'] = (rgb_ghost(), 'tub', None, 3, 'Yahboom 14-LED strip', None)
    for name, pts in TUBES.items():
        C['tube_' + name.replace('-', '_').lower()] = (tube_solid(trim_ends(pts, 6.0), TUBE_R), 'shell', None,
                                                       round(polylen(pts) * 0.03, 1), 'silicone 6x9 + water', 'free')
    return C

def rgb_ghost():
    cx = LIDAR['cx']
    col = rrect(cx - 20.0, cx + 20.0, -22.0, 22.0, (6, 6, 6, 6))
    rz = H + LIDAR['plate'] + (LIDAR['riser'] - 3.0 - RGB['w']) / 2
    ring = col.difference(col.buffer(-RGB['t'] + 0.2, join_style=2)).intersection(sbox(cx - 25, -30, cx + 21, 30))
    return prism_z(ring, rz + 0.2, rz + RGB['w'] + 0.2)

def polylen(pts):
    return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))

def trim_ends(pts, d):
    """drop d mm at each end: those lengths sit on a barb / fitting inside the pump or tank envelope"""
    import numpy as np
    P = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1); s = np.concatenate([[0], np.cumsum(seg)])
    t = np.linspace(d, s[-1] - d, max(3, int((s[-1] - 2 * d) / 2)))
    return [tuple(np.interp(tt, s, P[:, k]) for k in range(3)) for tt in t]

def tube_end_at_tank(name, end):
    q = TUBES.get(name)
    return None if not q else q[end]
TANK_FIT_PTS = [tuple(q) for q in (tube_end_at_tank('R-out', -1), tube_end_at_tank('S-in', 0)) if q] or \
               [(117.5, -37.5, 152.5), (107.5, -37.5, 150.0)]
TANK_FITS = [(q[0], (TANK_SLOT[0] + CLR) + 6.0) for q in TANK_FIT_PTS]

HOSE_S0 = 8.0                       # hose clipped along the arm from the strainer to 8 mm from the pivot (clears the M2 horn screw head)

def _bez(p0, t0, p3, t3, h0, h3, n):
    import numpy as np
    b1, b2 = p0 + t0 * h0, p3 - t3 * h3
    ts = np.linspace(0, 1, n)[:, None]
    return (1 - ts) ** 3 * p0 + 3 * (1 - ts) ** 2 * ts * b1 + 3 * (1 - ts) * ts ** 2 * b2 + ts ** 3 * p3

def _minr(P, span=12):
    import numpy as np
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1); s_ = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s_[-1], 1.0); Q = np.stack([np.interp(t, s_, P[:, k]) for k in range(3)], 1)
    best = 1e9
    for i in range(span, len(Q) - span, 2):
        a_, b_, c_ = Q[i - span], Q[i], Q[i + span]
        A, B, C_ = np.linalg.norm(b_ - c_), np.linalg.norm(a_ - c_), np.linalg.norm(a_ - b_)
        ar = np.linalg.norm(np.cross(b_ - a_, c_ - a_)) / 2
        if ar > 1e-9:
            best = min(best, A * B * C_ / (4 * ar))
    return best

GLAND = (124.0, -W - 0.5, 116.0)    # where the intake hose passes the right wall (fixed): a small oblique slot
GLAND_DIR = (-math.sin(math.radians(50.0)), math.cos(math.radians(50.0)), 0.0)   # into the body, leaning back 50 deg
HOSE_LF = 65.0                      # the free loop's length (clip -> gland): ONE hose length for every arm angle

def _wheel_gap(P):
    """distance from hose centreline points to the front-right wheel (cylinder about Y), minus nothing"""
    import numpy as np
    d_r = np.hypot(P[:, 0] - XF, P[:, 2] - ZH) - WHEEL_R_WORST
    y0, y1 = -(WHEEL_Y0 + WHEEL_W), -WHEEL_Y0
    d_y = np.maximum(y0 - P[:, 1], P[:, 1] - y1)
    inside = (d_r < 0) & (d_y < 0)
    out = np.where(inside, np.maximum(d_r, d_y), np.hypot(np.maximum(d_r, 0), np.maximum(d_y, 0)))
    return float(out.min())

@functools.lru_cache(None)
def hose_outside(deg_below, n=60):
    """intake hose: clipped along the arm's outer face from the strainer to HOSE_S0 from the pivot, then a free loop of the
    FIXED length HOSE_LF to the wall gland (the only part that flexes as the arm swings), then a fixed curve inside into the
    routed refill-in tube. At each arm angle the loop's shape (cubic Bezier handles) is the one of that length with the
    largest smallest bend radius that clears the front wheel by 4, stays off the ground and outside the wall."""
    import numpy as np
    a = math.radians(deg_below); u = np.array([math.cos(a), 0, -math.sin(a)])
    piv = np.array([SERVO_PX, ARM['y'][0], SERVO_PZ])
    p_tip = piv + u * (ARM['reach'] + ARM['strainer'][1] - 2.0); p0 = piv + u * HOSE_S0
    g = np.array(GLAND); tg = np.array(GLAND_DIR)
    q = TUBES.get('R-in')
    p3 = np.array(q[0]) if q else np.array([115.0, -66.6, 115.0])
    inside = _bez(g, tg, p3, np.array([-1.0, 0, 0]), 0.45 * np.linalg.norm(p3 - g), 0.45 * np.linalg.norm(p3 - g), 24)
    best = None
    for h0 in range(3, 125, 2):
        for h3 in range(3, 125, 2):
            B = _bez(p0, -u, g, tg, h0, h3, 50)
            if abs(float(np.linalg.norm(np.diff(B, axis=0), axis=1).sum()) - HOSE_LF) > 1.5:
                continue
            if B[:, 2].min() < 1.0 or (B[:-6, 1] > -W - 0.5 - 4.5).any() or _wheel_gap(B) < 4.0:
                continue
            r = _minr(np.vstack([B, inside[1:]]))
            if best is None or r > best[0]:
                best = (r, h0, h3)
    assert best, f'no {HOSE_LF} mm hose loop at {deg_below} deg'
    B = _bez(p0, -u, g, tg, best[1], best[2], n)
    return np.vstack([np.linspace(p_tip, p0, 12), B[1:], inside[1:]])

def _window():
    """slot where the intake hose's centreline is inside the right wall (both arm positions share the fixed gland)"""
    import numpy as np
    pts = []
    for deg in (ARM['stowed_deg'], ARM['deployed_deg']):
        H_ = hose_outside(deg, 400)
        pts.append(H_[(H_[:, 1] > -W - 1.0) & (H_[:, 1] < -WI + 1.0)])
    P_ = np.vstack(pts)
    mx = (TUBE_R + CLR) / 0.6 + 1.0; mz = TUBE_R + CLR + 0.5
    return dict(x=(float(P_[:, 0].min()) - mx, float(P_[:, 0].max()) + mx), z=(float(P_[:, 2].min()) - mz, float(P_[:, 2].max()) + mz))
WIN = _window()
NINE_X = (S2_X0, S2_X0 + NINE[0])                                # rear end wall sits in a notch of the nose-bay wall
NINE_Y = [(BAT_SLOT[0] + CLR + 2.0, BAT_SLOT[0] + CLR + 2.0 + NINE[1]),
          (BAT_SLOT[0] + CLR + 2.0 + NINE[1] + 2 * CLR + 1.6, BAT_SLOT[0] + CLR + 2.0 + 2 * NINE[1] + 2 * CLR + 1.6)]

def keepouts():
    """space a component needs but does not fill: plugs, fan air"""
    K = {}
    K['pi_plug_zone'] = (bx(OPI_X0, OPI_X1, OPI_Y1, OPI_Y1 + OPI_PLUG, OPI_Z0, OPI_Z1), 'tub', 'tub_upper')
    fx, fy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    K['pi_fan_air'] = (bx(fx - 20, fx + 20, fy - 20, fy + 20, OPI_Z1, OPI_Z1 + OPI_AIR), 'tub', 'tub_upper')
    return K

def bays():
    return {
        'tub_lower':   ('tub',   bx(XRI - 1, J_TUB, -WI, WI, FLOOR, DECK_Z0)),
        'tub_upper':   ('tub',   bx(XRI - 1, J_TUB, -WI, WI, DECK_Z0, H_IN)),
        'shell_lower': ('shell', bx(J_SH, NOSE_I + 1, -WI, WI, FLOOR, DECK_Z0)),
        'shell_upper': ('shell', bx(J_SH, S2_X0 - DIV, -WI, WI, DECK_Z0, H_IN)),
        'nose':        ('shell', bx(S2_X0, NOSE_I + 1, -WI, WI, DECK_Z0, H_IN)),
    }

def cavities():
    po, pi = tub_plans(); _, tc = body(tub_profile(), po, pi)
    po, pi = shell_plans(); _, sc = body(shell_profile(), po, pi)
    return {'tub': tc, 'shell': sc}

# ---------------- build + export ----------------
PRINT1 = ('main_tub', 'front_shell')
PRINT2 = ('tub_cap', 'shell_cap', 'batt_hatch', 'tub_deck', 'tub_shelf', 'pump_deck')
PRINT3 = ('rear_hatch', 'tank_hatch', 'lidar_mount', 'refill_arm', 'esp_carrier', 'spacer_1', 'spacer_2', 'spacer_3', 'spacer_4')
BUILDERS = dict(main_tub=main_tub, rear_hatch=rear_hatch, front_shell=front_shell, tub_cap=tub_cap, shell_cap=shell_cap, batt_hatch=batt_hatch,
                tank_hatch=tank_hatch, tub_deck=tub_deck, tub_shelf=tub_shelf, pump_deck=pump_deck, lidar_mount=lidar_mount,
                refill_arm=lambda: arm_solid(ARM['stowed_deg'], with_strainer=False), esp_carrier=lambda: esp_carrier(),
                **{f'spacer_{i}': (lambda i=i: coupling_spacer().translate((XF if i <= 2 else XR, 0, 0))) for i in range(1, 5)})
BODY_OF = dict(main_tub='tub', rear_hatch='tub', tub_cap='tub', tub_deck='tub', tub_shelf='tub', lidar_mount='tub',
               front_shell='shell', shell_cap='shell', batt_hatch='shell', tank_hatch='shell', pump_deck='shell', refill_arm='shell',
               esp_carrier='shell',
               spacer_1='none', spacer_2='none', spacer_3='none', spacer_4='none')
BED_FLIP = dict(rear_hatch=0, tub_cap=180, shell_cap=180, batt_hatch=180, tank_hatch=180, tub_deck=0, tub_shelf=180, pump_deck=0, lidar_mount=0,
                refill_arm=0, esp_carrier=0, spacer_1=0, spacer_2=0, spacer_3=0, spacer_4=0)

def summary():
    return dict(version='v3', XRO=XRO, NOSE=NOSE, H=H, TOP_T=H, TOP_S=H, Z_SPLIT=Z_SPLIT, length=round(NOSE - XRO, 1),
                nose_overhang=round(NOSE - WHEEL_FRONT, 1), wheel_front=WHEEL_FRONT, ramp_x0=RAMP_X0,
                OPI=(OPI_X0, OPI_X1, OPI_Y0, OPI_Y1), BOARD=(BOARD_X0, BOARD_X1, BOARD_Z0, BOARD_Z1), S2_X0=S2_X0,
                slots=dict(bay=BAY, tank=TANK_SLOT, bat=BAT_SLOT), lidar_cx=LIDAR['cx'],
                scan_plane=round(LIDAR['base'] + LIDAR['scan'], 2), scan_min=round(LIDAR['scan_min'], 2), lidar_riser=LIDAR['riser'],
                shell_max_z_20=round(shell_max_z(20.0), 2), servo=dict(px=SERVO_PX, pz=SERVO_PZ),
                arm=dict(stowed=ARM['stowed_deg'], deployed=ARM['deployed_deg'], y=ARM['y'], reach=ARM['reach']),
                nozzle=dict(tip=NOZ['tip'], rear=NOZ['rear'], angle=NOZ['angle']),
                wheel=dict(gap=WHEEL_GAP, spacer=WHEEL_SPACER, shaft_grip=SHAFT_GRIP, inner_y=WHEEL_Y0, width=WHEEL_W),
                esp=dict(x=ESP_X, y=ESP_Y, z=(ESP_Z0, ESP_Z1)), tank_z=TANK_Z, batt_z=BATT_Z, barbs=BARBS, pump_y=P_Y,
                tubes={k: round(polylen(v), 1) for k, v in TUBES.items()})

def export_all(outdir='out_v3'):
    import trimesh
    os.makedirs(outdir, exist_ok=True)
    for f in os.listdir(outdir):
        if f.endswith('.stl') or f.endswith('.step'):
            os.remove(os.path.join(outdir, f))                       # no stale parts from older versions
    parts = {}
    for name in PRINT1 + PRINT2 + PRINT3:
        s = BUILDERS[name](); parts[name] = s
        cq.exporters.export(s, f'{outdir}/{name}.stl', tolerance=0.05, angularTolerance=0.15)
        cq.exporters.export(s, f'{outdir}/{name}.step')
        mm = trimesh.load(f'{outdir}/{name}.stl')
        print(' built', name, round(s.val().Volume() / 1000, 1), 'cm3', 'solids', len(s.solids().vals()),
              'valid', s.val().isValid(), 'watertight', mm.is_watertight)
    comps = components()
    os.makedirs(f'{outdir}/comp', exist_ok=True)
    for f in os.listdir(f'{outdir}/comp'):
        os.remove(os.path.join(f'{outdir}/comp', f))
    for name, (s, body, bay, m, src, seat) in comps.items():
        cq.exporters.export(s, f'{outdir}/comp/{name}.stl', tolerance=0.1, angularTolerance=0.3)
    for name, (s, body, bay) in keepouts().items():
        cq.exporters.export(s, f'{outdir}/comp/keepout_{name}.stl', tolerance=0.1, angularTolerance=0.3)
    meta = dict(summary=summary(), body_of=BODY_OF, print1=PRINT1, print2=PRINT2, print3=PRINT3, bed_flip=BED_FLIP,
                components={k: dict(body=v[1], bay=v[2], mass=v[3], src=v[4], seat=v[5]) for k, v in comps.items()},
                keepouts={k: dict(body=v[1], bay=v[2]) for k, v in keepouts().items()},
                hinge=dict(x=XH, z=ZH, gap=GAP, rake=RAKE, sweep=SWEEP))
    with open(f'{outdir}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return parts

BED, BED_GAP = 256.0, 3.0
P1S_EXCL = (18.0, 28.0)             # P1S bed_exclude_area: front-left corner X 0-18, Y 0-28 must stay empty
from shapely.geometry import MultiPoint, box as shapely_box

def print_layouts(meshes, prints=(2, 3)):
    """meshes: name -> trimesh in the model frame. Returns {1: [...], 2: [...], 3: [...]} of (name, mesh on bed).
    Print 1: tub + shell exactly as assembled at 0 deg, belly down, centred. Prints 2 and 3: every part turned to its
    printing face (BED_FLIP), then placed first-fit (largest first, 0/90 deg) on a 2 mm grid with 3 mm between bounding
    boxes, never in the P1S's front-left keep-out corner."""
    import trimesh, numpy as np
    T_ = trimesh.transformations
    out = {1: [], 2: [], 3: []}
    p1 = [meshes[n].copy() for n in PRINT1]
    allm = trimesh.util.concatenate(p1); lo, hi = allm.bounds
    for n, m in zip(PRINT1, p1):
        m.apply_translation([BED / 2 - (lo[0] + hi[0]) / 2, BED / 2 - (lo[1] + hi[1]) / 2, -lo[2]]); out[1].append((n, m))
    for k, names in ((2, PRINT2), (3, PRINT3)):
        if k not in prints:
            continue
        placed = []                                                   # (x0, y0, x1, y1)
        oriented = []
        for n in names:
            m = meshes[n].copy()
            if BED_FLIP.get(n):
                m.apply_transform(T_.rotation_matrix(math.radians(BED_FLIP[n]), [1, 0, 0]))
            oriented.append((n, m))
        oriented.sort(key=lambda nm: -nm[1].extents[0] * nm[1].extents[1])
        for n, m in oriented:
            done = False
            for turn in (0, 90):
                mm = m.copy()
                if turn:
                    mm.apply_transform(T_.rotation_matrix(math.radians(turn), [0, 0, 1]))
                ex, ey = mm.extents[0], mm.extents[1]
                for y in np.arange(1.0, BED - ey - 1.0, 2.0):
                    for x in np.arange(1.0, BED - ex - 1.0, 2.0):
                        r = (x, y, x + ex, y + ey)
                        if x < P1S_EXCL[0] + 2 and y < P1S_EXCL[1] + 2:
                            continue
                        if any(not (r[2] + BED_GAP <= q[0] or q[2] + BED_GAP <= r[0] or r[3] + BED_GAP <= q[1] or q[3] + BED_GAP <= r[1]) for q in placed):
                            continue
                        # Bambu tests the plate's 2D convex hull (one STL = one object) against the keep-out corner
                        hull = MultiPoint([(a, b) for q in placed + [r] for a in (q[0], q[2]) for b in (q[1], q[3])]).convex_hull
                        if hull.intersects(shapely_box(0, 0, P1S_EXCL[0] + 3, P1S_EXCL[1] + 3)):
                            continue
                        mm.apply_translation([x - mm.bounds[0][0], y - mm.bounds[0][1], -mm.bounds[0][2]])
                        placed.append(r); out[k].append((n, mm)); done = True
                        break
                    if done:
                        break
                if done:
                    break
            if not done:
                raise RuntimeError(f'print {k}: {n} does not fit on the bed')
    return out

PRINT_NAMES = {1: 'FPMS_Rover1_Print1_Chassis_Assembled_v3.stl', 2: 'FPMS_Rover1_Print2_Caps_Decks_LiDAR_v3.stl',
               3: 'FPMS_Rover1_Print3_Small_Parts_v3.stl'}

def export_prints(outdir='out_v3', dest='stl'):
    """the deliverables, as single STL files (+ the test coupon)"""
    import trimesh
    os.makedirs(dest, exist_ok=True)
    meshes = {n: trimesh.load(f'{outdir}/{n}.stl') for n in PRINT1 + PRINT2 + PRINT3}
    L = print_layouts(meshes)
    for k in (1, 2, 3):
        m = trimesh.util.concatenate([mm for _, mm in L[k]])
        m.export(f'{dest}/{PRINT_NAMES[k]}'); print(PRINT_NAMES[k], m.extents.round(1), 'bed max', m.bounds[1].round(1))
    c = test_coupon()
    cq.exporters.export(c, f'{dest}/FPMS_Rover1_TestCoupon.stl', tolerance=0.05, angularTolerance=0.15)
    print('FPMS_Rover1_TestCoupon.stl')

def test_coupon():
    """print BEFORE the 15 h job (< 1 h): (a) a 20 mm slice of the real hinge - two tub cups with the fused pin, the shell
    knuckle between them at 0.4 mm, same teardrops and chamfers, on short tub/shell floor slabs; (b) one motor pad exactly as
    in the chassis: 6 x M3 counterbored on Ø31, Ø12.9 boss hole, assembly-screw reliefs. Both lie as they print in the chassis."""
    y0, y1 = 0.0, 20.0
    k0, k1 = 6.0 + GAP, 14.0 - GAP
    tub = prism_y(cup2d(), y0, 6.0).union(prism_y(cup2d(), 14.0, y1)).union(prism_y(pin2d(), y0, y1))
    tub = tub.union(prism_y(sbox(J_TUB - 16.0, BELLY, J_TUB, BELLY + T), y0, y1))
    for (a, b) in ((y0, 6.0), (14.0, y1)):
        tub = tub.union(prism_y(sbox(J_TUB - 14.0, BELLY + 0.01, J_TUB, ZH + 6), a, b))
    tub = tub.cut(prism_y(tub_knuckle_pocket2d(), k0 - GAP, k1 + GAP))
    sh = prism_y(knuckle2d(), k0, k1).union(prism_y(sbox(J_SH, BELLY, J_SH + 16.0, BELLY + T), y0, y1))
    sh = sh.cut(prism_y(shell_cup_pocket2d(), y0 - 1, 6.0 + GAP)).cut(prism_y(shell_cup_pocket2d(), 14.0 - GAP, y1 + 1))
    sh = sh.cut(prism_y(shell_bore2d(), k0 - 1, k1 + 1))
    tub = tub.cut(prism_y(Polygon([(J_TUB + 0.1, BELLY - 0.1), (J_TUB - 0.9, BELLY - 0.1), (J_TUB + 0.1, BELLY + 0.9)]), y0 - 1, y1 + 1))
    sh = sh.cut(prism_y(Polygon([(J_SH - 0.1, BELLY - 0.1), (J_SH + 0.9, BELLY - 0.1), (J_SH - 0.1, BELLY + 0.9)]), y0 - 1, y1 + 1))
    hinge = tub.union(sh).translate((-XH, 0, -BELLY))
    # motor pad, printed flat (outer face down so the counterbores are on top)
    pad = cq.Workplane('XY').rect(2 * PAD_HALF, 2 * PAD_HALF).extrude(PAD)
    pad = pad.cut(cyl_z(0, -7.0, 6.0 + CLR, -1, PAD + 1))                                       # boss hole (shaft 7 below)
    for (dx, dz) in face_pts(M3_ANG):
        pad = pad.cut(cyl_z(dx, dz, 3.4 / 2, -1, PAD + 1)).cut(cyl_z(dx, dz, 6.0 / 2, -1, 3.4))   # counterbore on the outer face
    for (dx, dz) in face_pts(ASM_ANG):
        pad = pad.cut(cyl_z(dx, dz, 3.0, PAD - 1.2, PAD + 1))                                   # relief, inner face
    pad = pad.translate((0, 50.0, 0))
    return hinge.union(pad)

if __name__ == '__main__':
    import sys
    print(json.dumps(summary(), indent=1, default=str))
    export_all()
    if '--prints' in sys.argv:
        export_prints()
