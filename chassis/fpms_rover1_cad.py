# FPMS Rover 1 chassis - CadQuery source, v1 (single file).
# Frame: X forward (rear axle at X=0), Y left, Z up, mm. Ground at Z=0.
# Print 1 = main tub + front shell printed ASSEMBLED, hinge printed in place (0.4 mm, teardrop bores).
# Print 2 = lids + removable decks + LiDAR mount (+ tank hatch).
# Sources for every part dimension: RESEARCH.md. Team measurements win over online numbers.
# Run:  pip install cadquery trimesh manifold3d shapely   then   python fpms_rover1_cad.py
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
SWEEP = (-2.0, 21.0)               # hinge range the clearance cuts are swept over (stops land at ~-1.6 and ~20.3)

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
CLR = 0.5                          # every holder: component + 0.5 per side
DECK_Z0, DECK_Z1 = 66.0, 68.4      # removable decks just over the gearboxes (top Z 65.5)

# hole pattern on the gearbox face (RESEARCH.md): Ø31 about the GEARBOX centre, holes at ±30/±90/±150 deg
# from the shaft direction (shaft is below the centre), assembly screws at 180 and ±60 deg.
def face_pts(angles, r=15.5):
    return [(15.5 * math.sin(math.radians(a)), -15.5 * math.cos(math.radians(a))) for a in angles]
M3_ANG = (30, -30, 90, -90, 150, -150)
ASM_ANG = (180, 60, -60)

# ---------------- layout ----------------
OPI = (65.0, 92.8, 35.8); OPI_PLUG, OPI_AIR = 40.0, 10.0
BOARD = (56.0, 85.0, 20.0)         # Yahboom V3.0, components up, hanging from the tub lid on 4 standoffs
OPI_Z0 = DECK_Z1
OPI_Z1 = OPI_Z0 + OPI[2]
BOARD_Z0 = OPI_Z1 + CLR + OPI_AIR  # Pi fan gets its 10 mm of air under the board
BOARD_Z1 = BOARD_Z0 + BOARD[2]
TOP_T = math.ceil(BOARD_Z1 + CLR)  # tub rim = lid underside
SHIFT_Y = 5.6                      # Pi + board moved off the right wall so the right rear corner can be rounded
OPI_X1 = tub_face_in(OPI_Z1) - CLR; OPI_X0 = OPI_X1 - OPI[0]
OPI_Y0 = -WI + CLR + SHIFT_Y;       OPI_Y1 = OPI_Y0 + OPI[1]
BOARD_X1 = tub_face_in(BOARD_Z1) - CLR; BOARD_X0 = BOARD_X1 - BOARD[0]
XRI = min(OPI_X0, BOARD_X0) - CLR   # tub rear wall inner face
XRO = math.floor(XRI - T)           # tub rear outer face
XRI = XRO + T
ESTOP = dict(y=(24.0, 64.0), z=(OPI_Z1 + 0.8, OPI_Z1 + 30.8), depth=45.0, head_r=20.0, head_h=22.0)
REG = dict(x=(22.0, 31.5), y=(42.0, 67.4), z=(FLOOR, FLOOR + 25.4))
BOARD_Y1 = ESTOP['y'][0] - CLR - 3.0; BOARD_Y0 = BOARD_Y1 - BOARD[1]   # connectors toward +Y (the cable riser)
PCB_Z = BOARD_Z0 + 2.0              # 2 mm of solder pins under the PCB, components above
# board holes: 58 x 49 pattern, 23.5 in from the connector edge, 3.5 from the others
BOARD_HOLES = [(BOARD_X0 + 3.5 + i * 49.0, y) for i in (0, 1) for y in (BOARD_Y1 - 23.5, BOARD_Y1 - 81.5)]
TAIL_X = -22.0                      # tub belly rises 45 deg behind the rear motors (departure angle, no dead corner)

# front shell
PUMP = (40.0, 100.0, 50.0)          # X, Y (barbs at +Y end), Z
BEND = 30.0                         # 6x9 silicone, R25 bend + tube radius beyond the barb tip
P1_X0 = J_SH + T + CLR              # pump 1 against the shell's rear wall
P1_X1 = P1_X0 + PUMP[0]
DIV = 1.6
P2_X0 = P1_X1 + CLR + DIV + CLR
P2_X1 = P2_X0 + PUMP[0]
S2_X0 = P2_X1 + CLR + DIV          # nose bay (battery, tank, 9V, ESP32) starts behind its own divider
S2_FLOOR = 38.4                     # nose bay floor raised 10 mm: steps up to ~35 mm meet the wheel first
BATT = (50.0, 65.0, 75.0)           # 65x75x50 pack standing, 50 deep so the nose stays short
TANK = (40.0, 40.0, 80.0)
NINE = (48.5, 17.5, 26.5)           # 9 V Li-ion lying, two stacked
ESP = (25.4, 13.0, 69.0)            # ESP32-S3 DevKitC-1 standing, USB up (antenna overhang included)
DIV_S2 = 1.2
S2_DEPTH = BATT[0] + 2 * CLR
NOSE_I = S2_X0 + S2_DEPTH
NOSE = NOSE_I + T
TOP_S = math.ceil(max(DECK_Z1 + PUMP[2], S2_FLOOR + TANK[2]) + CLR)
RAMP_X0 = 124.0                     # belly 26 -> 36 at 45 deg under the front pocket
# nose bay slots across Y, right (-Y) to left: ESP32 | battery | 9V | tank
_y = -WI
ESP_SLOT = (_y, _y + ESP[1] + 2 * CLR); _y = ESP_SLOT[1] + DIV_S2
BAT_SLOT = (_y, _y + BATT[1] + 2 * CLR); _y = BAT_SLOT[1] + DIV_S2
NINE_SLOT = (_y, _y + NINE[1] + 2 * CLR); _y = NINE_SLOT[1] + DIV_S2
TANK_SLOT = (_y, WI)
assert TANK_SLOT[1] - TANK_SLOT[0] >= TANK[1] + 2 * CLR - 1e-6, ('nose bay too narrow', TANK_SLOT)

LIDAR = dict(fp=(54.0, 46.29), h=34.8, scan=27.0, holes=(46.8, 31.92), cx=None, cy=0.0, plate=4.0)
LID_T = 2.4
# removable decks carry only what sits on them (print 2 has to fit one P1S bed)
TUB_DECK_X1 = OPI_X1 + CLR + 1.6 + 3.0
TUB_DECK_Y1 = OPI_Y1 + 1.5
SH_DECK_Y1 = -WI + CLR + PUMP[1] + CLR + 2.0
XYMOS_X, XYMOS_Y = (110.6, 122.6), (-27.5, 27.5)   # inner belly ramp starts at X 123.0
SERVO_CX, SERVO_Z0 = NOSE - 20.0, BELLY + 24.0
TUBE_Z = 92.0                       # straight tube run along the left wall, above the bend zones' reach
# D500 on a small plate fixed through the two board standoffs nearest the centreline -> LiDAR centred on the tub
LIDAR_HOLE_Y = BOARD_Y1 - 23.5
LIDAR['cx'] = round(BOARD_X0 + 3.5 + 24.5, 2)           # rotation axis between those standoffs, on Y = 0
# D500 holes: pair 46.8 apart along X, third 31.92 to the left of the pair; rotation axis 8.7 from the pair, on Y = 0
LIDAR_HOLES = [(LIDAR['cx'] - 23.4, -8.7), (LIDAR['cx'] + 23.4, -8.7), (LIDAR['cx'], -8.7 + 31.92)]
LIDAR_LEAD = (-1.0, 9.0, 22.0, 32.0)                    # cable slot in the lid: over the open column ahead of the E-stop

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
def prism_y(g, y0, y1):             # shapely XZ region -> solid between y0 and y1
    out = None
    for p in polys(g):
        s = cq.Workplane('XZ').polyline(ring(p.exterior.coords)).close().extrude(-(y1 - y0)).translate((0, y0, 0))
        for h in p.interiors:
            s = s.cut(cq.Workplane('XZ').polyline(ring(h.coords)).close().extrude(-(y1 - y0)).translate((0, y0, 0)))
        out = s if out is None else out.union(s)
    return out
def prism_z(g, z0, z1):             # shapely XY region -> solid between z0 and z1
    out = None
    for p in polys(g):
        s = cq.Workplane('XY').polyline(ring(p.exterior.coords)).close().extrude(z1 - z0).translate((0, 0, z0))
        out = s if out is None else out.union(s)
    return out
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
def rot2d(g, deg):
    return affinity.rotate(g, deg, origin=(XH, ZH))
def sweep2d(g, sign):               # union of g rotated over the hinge range (sign +1: shell's motion seen from the tub)
    angs = [SWEEP[0] + i * 0.5 for i in range(int((SWEEP[1] - SWEEP[0]) / 0.5) + 1)]
    return unary_union([rot2d(g, sign * a) for a in angs])
def ledge(x0, x1, y0, y1, ztop, d=3.0, axis='y', into=+1):
    """deck ledge: 3 mm flat top, 45 deg gusset under it so it prints without support.
    axis: the wall normal ('y' for side walls, 'x' for end walls); into: direction from the wall into the bay."""
    if axis == 'y':
        wy = y1 if into < 0 else y0          # wall side
        prof = Polygon([(wy, ztop), (wy + into * d, ztop), (wy + into * d, ztop - 0.8), (wy, ztop - 0.8 - d)])
        pts = list(prof.exterior.coords)[:-1]
        return cq.Workplane('YZ').polyline(pts).close().extrude(x1 - x0).translate((x0, 0, 0))
    wx = x1 if into < 0 else x0
    prof = Polygon([(wx, ztop), (wx + into * d, ztop), (wx + into * d, ztop - 0.8), (wx, ztop - 0.8 - d)])
    return prism_y(prof, y0, y1)
def try_fillet(w, sel, r):
    try:
        return w.edges(sel).fillet(r)
    except Exception as e:
        print('  fillet skipped', sel, r, e.__class__.__name__); return w

# ---------------- hinge (all profiles in the XZ plane, extruded along Y) ----------------
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
    return unary_union([sweep2d(pin2d().buffer(GAP, 32), -1), cap_up(PIN_R + GAP)])
def shell_cup_pocket2d():
    return unary_union([sweep2d(cup2d().buffer(GAP, 32), -1), cap_up(CUP_R + GAP)])
def tub_knuckle_pocket2d():
    return unary_union([sweep2d(knuckle2d().buffer(GAP, 32), +1), cap_up(KN_R + GAP)])

# ---------------- bodies ----------------
def tub_profile(top=TOP_T):
    return Polygon([(XRO, top), (XRO, BELLY + (TAIL_X - XRO)), (TAIL_X, BELLY), (J_TUB, BELLY), (J_TUB, ZH),
                    (tub_face_x(top), top)])
def shell_profile(top=TOP_S, chamfer=3.0):  # chamfer only on the outside: inside stays square for the battery
    return Polygon([(J_SH, BELLY), (RAMP_X0, BELLY), (RAMP_X0 + 10, BELLY + 10), (NOSE - chamfer, BELLY + 10),
                    (NOSE, BELLY + 10 + chamfer), (NOSE, top), (J_SH, top)])
R_TUB = (11.4, 3.0, 3.0, 10.0)              # rear-right, front-right, front-left, rear-left plan radii
R_SH = (3.0, 12.0, 12.0, 3.0)
def hollow(profile, top, plan_out, plan_in, y_half=W, profile_tall=None):
    outer = prism_y(profile, -y_half, y_half).intersect(prism_z(plan_out, 0, 300))
    # open top: offset a copy of the profile that continues 20 mm above the rim ALONG its own faces
    # (stretching the top vertically would cut through the raked wall)
    inner2d = profile_tall.buffer(-T, join_style=2)
    inner = prism_y(inner2d, -(y_half - T), y_half - T).intersect(prism_z(plan_in, 0, 300))
    return outer, outer.cut(inner), inner

def motor_mount(w, xa, label):
    """JGB37-520 bolted through the side wall: Ø12.5 boss hole, 6 x M3 counterbored flush,
    assembly-screw reliefs, cradle under gearbox + can, zip-tie tunnel at the encoder end."""
    for s in (1, -1):
        y_in, y_out = s * M_FACE, s * W
        ylo, yhi = sorted((y_in, s * WI))
        w = w.union(bx(xa - 21, xa + 21, ylo, yhi, FLOOR - 0.5, DECK_Z0))                                    # pad
        a, b = sorted((s * (M_FACE - 1), s * (W + 1)))
        w = w.cut(teardrop_y(xa, ZH, 12.5 / 2, a, b))                                                         # boss locates
        for (dx, dz) in face_pts(M3_ANG):
            w = w.cut(teardrop_y(xa + dx, ZGB + dz, 3.4 / 2, a, b))
            c0, c1 = sorted((s * (W - 3.4), s * (W + 1)))
            w = w.cut(teardrop_y(xa + dx, ZGB + dz, 6.0 / 2, c0, c1))                                         # counterbore: head flush
        for (dx, dz) in face_pts(ASM_ANG):
            r0, r1 = sorted((s * (M_FACE - 1), s * (M_FACE + 1.2)))
            w = w.cut(teardrop_y(xa + dx, ZGB + dz, 3.0, r0, r1))                                             # relief for proud screw heads
        # cradles
        g0, g1 = sorted((s * (M_FACE - M_GB_L + 1.0), s * (M_FACE - 1.5)))
        w = w.union(bx(xa - 15, xa + 15, g0, g1, FLOOR - 0.5, 43).cut(cyl_y(xa, ZGB, M_GB_R + CLR, -100, 100)))
        c0, c1 = sorted((s * 26.0, s * (M_FACE - M_GB_L - 1.0)))
        w = w.union(bx(xa - 14, xa + 14, c0, c1, FLOOR - 0.5, 44).cut(cyl_y(xa, ZGB, M_CAN_R + CLR, -100, 100)))
        t0, t1 = sorted((s * 16.0, s * 23.0))                                                                 # strap block, encoder end
        blk = bx(xa - 14, xa + 14, t0, t1, FLOOR - 0.5, 44).cut(cyl_y(xa, ZGB, M_CAN_R + CLR, -100, 100))
        u0, u1 = sorted((s * 16.6, s * 22.4))
        blk = blk.cut(bx(xa - 20, xa + 20, u0, u1, FLOOR - 0.6, ZGB - M_CAN_R))                               # tie tunnel under the can
        w = w.union(blk)
    w = w.cut(cyl_y(xa, ZGB, M_GB_R + CLR, M_FACE - M_GB_L - 1, M_FACE + 0.01)).cut(
        cyl_y(xa, ZGB, M_GB_R + CLR, -M_FACE - 0.01, -(M_FACE - M_GB_L - 1)))                                 # trough so the gearbox clears the floor
    return w

def channel(w, x0, x1):                      # cable channel on the centreline: two ribs, 6 tall
    for s in (1, -1):
        a, b = sorted((s * 6.6, s * 7.8))
        w = w.union(bx(x0, x1, a, b, FLOOR - 0.5, FLOOR + 6))
    return w

def rim_boss(w, x, y_wall, top, along='x', into=+1, size=(8.0, 5.0)):
    """lid screw boss hanging just under the rim, 45 deg gusset beneath (prints without support), M3 pilot Ø2.6.
    along: the wall runs along this axis; y_wall: wall inner face coordinate; into: direction into the bay"""
    L, D = size
    if along == 'x':
        prof = Polygon([(y_wall, top), (y_wall + into * D, top), (y_wall + into * D, top - 6), (y_wall, top - 6 - D)])
        g = cq.Workplane('YZ').polyline(ring(list(prof.exterior.coords))).close().extrude(L).translate((x - L / 2, 0, 0))
        hx, hy = x, y_wall + into * D / 2
    else:
        prof = Polygon([(y_wall, top), (y_wall + into * D, top), (y_wall + into * D, top - 6), (y_wall, top - 6 - D)])
        g = prism_y(prof, x - L / 2, x + L / 2)
        hx, hy = y_wall + into * D / 2, x
    return w.union(g).cut(cyl_z(hx, hy, 1.3, top - 9, top + 1)), (hx, hy)

TUB_BOSSES, SHELL_BOSSES = [], []

def deck_post(w, x, y, ztop, floor_z=FLOOR):
    """printed column the removable deck rests on (5 x 5, locating peg on top)"""
    w = w.union(bx(x - 2.5, x + 2.5, y - 2.5, y + 2.5, floor_z - 0.5, ztop))
    return w.union(cyl_z(x, y, 1.4, ztop - 0.1, ztop + 2.0))

def main_tub():
    plan_o = rrect(XRO, J_TUB + 1, -W, W, R_TUB)
    plan_i = rrect(XRI, J_TUB + 1, -WI, WI, tuple(max(0.5, r - T) for r in R_TUB))
    outer, t, cav = hollow(tub_profile(), TOP_T, plan_o, plan_i, profile_tall=tub_profile(TOP_T + 20))
    t = motor_mount(t, XR, 'rear')
    t = channel(t, M_GB_R + 1, tub_face_in(FLOOR) - 1)                       # starts clear of the motor cans
    # deck: rear-wall ledge, right-wall ledge (the Pi deck spans Y -71.3 .. the plug zone), 2 posts on the free edge
    t = t.union(ledge(XRI, XRI + 3, -WI + 10, OPI_Y1 - 4, DECK_Z0, axis='x', into=+1))
    t = t.union(ledge(XRI + 12, -21.0, -WI, -WI + 3, DECK_Z0, axis='y', into=+1))
    for (px, py) in ((XRI + 8.0, TUB_DECK_Y1 - 4.0), (TUB_DECK_X1 - 5.0, TUB_DECK_Y1 - 4.0)):
        floor_here = FLOOR if px > TAIL_X - 2 else FLOOR + (TAIL_X - px) + 2
        t = deck_post(t, px, py, DECK_Z0, floor_here)
    # lid bosses: 2 on each side wall, clear of the board and the E-stop
    TUB_BOSSES.clear()
    for xx in (XRI + 14.0, 2.0):
        for s in (1, -1):
            t, h = rim_boss(t, xx, s * WI, TOP_T, along='x', into=-s); TUB_BOSSES.append(h)
    # 5 V regulator holder: open frame on the floor ahead of the left rear motor, board standing on edge
    rx0, rx1 = REG['x']; ry0, ry1 = REG['y']
    t = t.union(bx(rx0 - 1.2 - CLR, rx1 + CLR + 1.2, ry0 - 1.2 - CLR, ry1 + CLR + 1.2, FLOOR - 0.5, FLOOR + 8).cut(
        bx(rx0 - CLR, rx1 + CLR, ry0 - CLR, ry1 + CLR, FLOOR, FLOOR + 10)))
    # E-stop: Ø22.4 panel hole + 3.2 anti-rotation notch through the rear wall
    ey, ez = sum(ESTOP['y']) / 2, sum(ESTOP['z']) / 2
    t = t.cut(cyl_x(ey, ez, 22.4 / 2, XRO - 1, XRI + 1))
    t = t.cut(bx(XRO - 1, XRI + 1, ey - 1.6, ey + 1.6, ez, ez + 24.1 - 11.2))
    # Pi fan air: vent slots in both side walls between the Pi and the board
    for i in range(6):
        vx = OPI_X0 + 6 + i * 9.5
        for s in (1, -1):
            a, b = sorted((s * (WI - 1), s * (W + 1)))
            t = t.cut(prism_y(Point(vx, OPI_Z1 + 3).buffer(1.6, 16).union(sbox(vx - 1.6, OPI_Z1 + 3, vx + 1.6, BOARD_Z0 - 3)).union(
                teardrop2d(vx, BOARD_Z0 - 3, 1.6)), a, b))
    # hinge: inner + outer cups with the pin fused through them, pocket for the shell knuckle
    for s in (1, -1):
        for (y0, y1) in CUPS_Y:
            a, b = sorted((s * y0, s * y1))
            t = t.union(prism_y(cup2d(), a, b))
            t = t.union(prism_y(sbox(J_TUB - 14.0, BELLY + 0.01, J_TUB, ZH + 6), a, b))
        a, b = sorted((s * (KN_Y[0] - GAP), s * (KN_Y[1] + GAP)))
        t = t.cut(prism_y(tub_knuckle_pocket2d(), a, b))
        a, b = sorted((s * 30.0, s * W))
        t = t.union(prism_y(pin2d(), a, b))                                  # pin runs cup to cup through the pocket
    # cable pass through the joint, on the centreline just above the pin (least stretch)
    t = t.cut(bx(J_TUB - 12, J_TUB + 1, -10, 10, 33, 57))
    for (dx, dy) in ((-17, 0), (38, 55), (38, -55)):
        t = t.cut(cyl_z(dx, dy, 2, BELLY - 1, FLOOR + 1))
    return t

def front_shell():
    plan_o = rrect(J_SH - 1, NOSE, -W, W, R_SH)
    plan_i = rrect(J_SH - 1, NOSE_I, -WI, WI, tuple(max(0.5, r - T) for r in R_SH))
    outer, f, cav = hollow(shell_profile(), TOP_S, plan_o, plan_i, profile_tall=shell_profile(TOP_S + 20, chamfer=0.01))
    f = motor_mount(f, XF, 'front')
    f = channel(f, J_SH + T + 1, XF - M_GB_R - 1)
    # deck: rear-wall ledge, right-wall ledges, front ledge on the nose-bay divider, 2 posts on the free edge
    f = f.union(ledge(J_SH + T, J_SH + T + 3, -WI + 4, SH_DECK_Y1 - 4, DECK_Z0, axis='x', into=+1))
    for (x0, x1) in ((J_SH + T + 14, XF - 21), (XF + 21, S2_X0 - DIV - 1)):
        f = f.union(ledge(x0, x1, -WI, -WI + 3, DECK_Z0, axis='y', into=+1))
    f = f.union(bx(S2_X0 - DIV, S2_X0, -WI, WI, FLOOR - 0.5, 100))
    f = f.union(ledge(S2_X0 - DIV - 3, S2_X0 - DIV, -WI + 4, SH_DECK_Y1 - 4, DECK_Z0, axis='x', into=-1))
    for px in (J_SH + T + 9.0, S2_X0 - DIV - 6.0):
        floor_here = FLOOR if px < RAMP_X0 else FLOOR + (px - RAMP_X0) + 1
        f = deck_post(f, px, SH_DECK_Y1 - 4.0, DECK_Z0, floor_here)
    # nose bay: slot dividers, floor raised to S2_FLOOR
    for yb in (ESP_SLOT[1], BAT_SLOT[1], NINE_SLOT[1]):
        f = f.union(bx(S2_X0, NOSE_I + 0.5, yb, yb + DIV_S2, S2_FLOOR - 0.5, 100))
    f = f.union(bx(S2_X0 + ESP[0] + 2 * CLR, S2_X0 + ESP[0] + 2 * CLR + DIV_S2, ESP_SLOT[0], ESP_SLOT[1], S2_FLOOR - 0.5, S2_FLOOR + 30))
    f = f.union(bx(S2_X0 + TANK[0] + 2 * CLR, S2_X0 + TANK[0] + 2 * CLR + DIV_S2, TANK_SLOT[0], TANK_SLOT[1], S2_FLOOR - 0.5, S2_FLOOR + 30))
    # XY-MOS x2 standing in the front pocket under pump 2 (next to the pumps they switch)
    for yc in XYMOS_Y:
        x0, x1 = XYMOS_X[0] - CLR, XYMOS_X[1] + CLR
        y0, y1 = yc - 17 - CLR, yc + 17 + CLR
        f = f.union(bx(x0 - 1.2, x1 + 1.2, y0 - 1.2, y1 + 1.2, FLOOR - 0.5, FLOOR + 8).cut(bx(x0, x1, y0, y1, FLOOR, FLOOR + 10)))
    # lid bosses: rear wall in the wet corridor, front corners of the nose bay
    SHELL_BOSSES.clear()
    f, h = rim_boss(f, 64.0, J_SH + T, TOP_S, along='y', into=+1); SHELL_BOSSES.append(h)          # past pump 1's bend zone
    for s in (1, -1):
        f, h = rim_boss(f, NOSE_I - 5.0, s * WI, TOP_S, along='x', into=-s); SHELL_BOSSES.append(h)    # ahead of tank / ESP32
    # tube clips on the left wall: the wet corridor's straight run (beyond the bend zones), open-top U, 45 deg gusset
    for cxp in (70.0, 112.0):
        prof = unary_union([Polygon([(WI, TUBE_Z - 19), (WI - 12, TUBE_Z - 7), (WI - 12, TUBE_Z + 5), (WI, TUBE_Z + 5)])]).difference(
            unary_union([Point(WI - 6.5, TUBE_Z).buffer(4.9, 32), sbox(WI - 9.5, TUBE_Z, WI - 3.5, TUBE_Z + 10)]))
        g = cq.Workplane('YZ').polyline(ring(list(polys(prof)[0].exterior.coords))).close().extrude(6.0).translate((cxp - 3, 0, 0))
        f = f.union(g)
    # VL53L0X window in the nose (sensor rides on the lid bracket, 45 deg down)
    vy = sum(NINE_SLOT) / 2
    f = f.cut(bx(NOSE_I - 1, NOSE + 1, vy - 7, vy + 7, TOP_S - 22, TOP_S - 6))
    # servo pod (refill arm) outside the right nose wall, ahead of the front wheel: body pocket, ears on the rim
    pcx = SERVO_CX
    pod = rrect(pcx - 17.5, pcx + 17.5, -W - 14.6, -W + 1, (4, 4, 0.5, 0.5))
    f = f.union(prism_z(pod, BELLY + 10, SERVO_Z0 + 18.5))
    f = f.cut(bx(pcx - 11.9, pcx + 11.9, -W - 13.0, -W, SERVO_Z0, SERVO_Z0 + 40))                          # body pocket 22.8 + 1
    for sx in (-1, 1):
        f = f.cut(cyl_z(pcx + sx * 13.9, -W - 6.5, 1.0, SERVO_Z0 + 8, SERVO_Z0 + 19))                         # ear screws (M2 self-tap)
    f = f.cut(teardrop_y(pcx - 8, SERVO_Z0 + 4, 3.0, -W - 1, -WI + 1))                                       # servo lead into the ESP32 slot
    # hinge: knuckles with teardrop bore, pockets for the tub's cups (cut through the side wall too)
    for s in (1, -1):
        a, b = sorted((s * KN_Y[0], s * KN_Y[1]))
        f = f.union(prism_y(knuckle2d(), a, b))
        for (y0, y1) in CUPS_Y:
            a, b = sorted((s * (y0 - GAP), s * (y1 + (GAP if y1 < W else 2))))
            f = f.cut(prism_y(shell_cup_pocket2d(), a, b))
        a, b = sorted((s * (KN_Y[0] - 1), s * (KN_Y[1] + 1)))
        f = f.cut(prism_y(shell_bore2d(), a, b))
    f = f.cut(bx(J_SH - 1, J_SH + T + 1, -10, 10, 33, 57))                                                  # cable pass
    for (dx, dy) in ((100, 0), (150, -20)):
        f = f.cut(cyl_z(dx, dy, 2, BELLY - 1, S2_FLOOR + 1))
    return f

# ---------------- removable parts (print 2) ----------------
def tub_deck():
    """Pi deck: just the Pi's footprint + its holder, resting on the rear/right ledges and two posts"""
    d = prism_z(rrect(XRI + 0.3, TUB_DECK_X1, -WI + 0.3, TUB_DECK_Y1, (max(0.5, R_TUB[0] - T - 0.3), 1, 2, 2)), DECK_Z0, DECK_Z1)
    # holder: 3 walls (port end open so plugs go straight in), 6 tall
    fr = bx(OPI_X0 - CLR - 1.6, OPI_X1 + CLR + 1.6, OPI_Y0 - CLR - 1.6, OPI_Y1 - 2.0, DECK_Z1 - 0.1, DECK_Z1 + 6).cut(
        bx(OPI_X0 - CLR, OPI_X1 + CLR, OPI_Y0 - CLR, OPI_Y1 + 5, DECK_Z1 - 1, DECK_Z1 + 7))
    d = d.union(fr)
    cx, cy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    for dy in (-32.5, 32.5):                                         # N515 keyhole slots, 65 apart on the long axis
        d = d.cut(cyl_z(cx, cy + dy, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))
    for (px, py) in ((XRI + 8.0, TUB_DECK_Y1 - 4.0), (TUB_DECK_X1 - 5.0, TUB_DECK_Y1 - 4.0)):
        d = d.cut(cyl_z(px, py, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))     # posts' locating pegs
    d = d.cut(prism_z(rrect(cx - 22, cx + 22, cy - 30, cy + 30, (6, 6, 6, 6)), DECK_Z0 - 1, DECK_Z1 + 1))   # lightening + air
    return d

def shell_deck():
    """pump deck: two pump pockets (rear wall / divider / nose-bay divider are the stops), straps, tube clips"""
    x0, x1 = J_SH + T + 0.3, S2_X0 - DIV - 0.3
    d = prism_z(rrect(x0, x1, -WI + 0.3, SH_DECK_Y1, (1, 1, 2, 2)), DECK_Z0, DECK_Z1)
    d = d.union(bx(P1_X1 + CLR, P2_X0 - CLR, -WI + 0.3, SH_DECK_Y1 - 3, DECK_Z1 - 0.1, DECK_Z1 + 12))
    for (px0, px1) in ((P1_X0, P1_X1), (P2_X0, P2_X1)):
        for yy in (-50.0, 0.0):                                                     # strap slots either side of each pump
            for xx in (max(px0 - 0.2, x0 + 0.8), px1 - 3.6):
                d = d.cut(bx(xx, xx + 3.0, yy - 3, yy + 3, DECK_Z0 - 1, DECK_Z1 + 1))
    for px in (J_SH + T + 9.0, S2_X0 - DIV - 6.0):
        d = d.cut(cyl_z(px, SH_DECK_Y1 - 4.0, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))
    for px in ((P1_X0 + P1_X1) / 2, (P2_X0 + P2_X1) / 2):
        d = d.cut(prism_z(rrect(px - 12, px + 12, -40, 20, (5, 5, 5, 5)), DECK_Z0 - 1, DECK_Z1 + 1))     # lightening
    return d

def lid_tub():
    """tub lid. Prints top-down: board standoffs hang from it (board components up, IMU unchanged);
    the LiDAR plate screws into the top of the two standoffs nearest the centreline."""
    xf = tub_face_x(TOP_T + LID_T) - 1.5
    l = prism_z(rrect(XRO, xf, -W, W, (R_TUB[0], 2, 2, R_TUB[3])), TOP_T, TOP_T + LID_T)
    l = try_fillet(l, '>Z', 1.2)
    for (hx, hy) in TUB_BOSSES:
        l = l.cut(cyl_z(hx, hy, 1.7, TOP_T - 1, TOP_T + LID_T + 1))
    for (hx, hy) in BOARD_HOLES:
        l = l.union(cyl_z(hx, hy, 2.9, PCB_Z + 1.6, TOP_T + 0.1))                   # standoff down to the PCB
        l = l.cut(cyl_z(hx, hy, 1.1, PCB_Z, TOP_T - 6))                              # M2.5 pilot from below
        if abs(hy - LIDAR_HOLE_Y) < 1:
            l = l.cut(cyl_z(hx, hy, 1.1, TOP_T - 8, TOP_T + LID_T + 1))              # ... and from the top (LiDAR plate)
    l = l.cut(prism_z(rrect(*LIDAR_LEAD, (2, 2, 2, 2)), TOP_T - 1, TOP_T + LID_T + 1))   # LiDAR lead
    return l

def lidar_mount():
    """D500 plate: LiDAR centred on the tub, 3 x M2.5 from the LiDAR into the plate, 2 x M2.5 through into the standoffs"""
    cx, z0 = LIDAR['cx'], TOP_T + LID_T
    xs = [hx for (hx, hy) in BOARD_HOLES if abs(hy - LIDAR_HOLE_Y) < 1]
    m = prism_z(rrect(min(xs) - 4.5, max(xs) + 3.7, -LIDAR['fp'][1] / 2 - 1, LIDAR['fp'][1] / 2 + 1, (5, 5, 5, 5)), z0, z0 + LIDAR['plate'])
    for (hx, hy) in LIDAR_HOLES:
        m = m.cut(cyl_z(hx, hy, 1.1, z0 + 0.6, z0 + 5))
    for hx in xs:
        m = m.cut(cyl_z(hx, LIDAR_HOLE_Y, 1.4, z0 - 1, z0 + 5)).cut(cyl_z(hx, LIDAR_HOLE_Y, 2.6, z0 + LIDAR['plate'] - 1.5, z0 + 5))
    
    return m

def lid_shell():
    x0 = J_SH + 1.5
    l = prism_z(rrect(x0, NOSE, -W, W, (2, R_SH[1], R_SH[2], 2)), TOP_S, TOP_S + LID_T)
    l = try_fillet(l, '>Z', 1.2)
    tx0, tx1 = S2_X0 - 0.2, S2_X0 + TANK[0] + 2 * CLR + 0.2
    l = l.cut(bx(tx0, tx1, TANK_SLOT[0] - 0.2, WI - 0.6, TOP_S - 1, TOP_S + LID_T + 1))                      # tank lifts out here
    l = l.cut(prism_z(rrect(S2_X0 + 2, S2_X0 + ESP[0] - 2, ESP_SLOT[0] + 1.5, ESP_SLOT[1] - 1.5, (2, 2, 2, 2)), TOP_S - 1, TOP_S + LID_T + 1))
    for (hx, hy) in SHELL_BOSSES:
        l = l.cut(cyl_z(hx, hy, 1.7, TOP_S - 1, TOP_S + LID_T + 1))
    # VL53L0X bracket: 45 deg plate hanging at the nose, behind the window
    vy = sum(NINE_SLOT) / 2; zc = TOP_S - 14
    plate = Polygon([(NOSE_I - 0.6, zc + 8), (NOSE_I - 12.5, TOP_S + 0.1), (NOSE_I - 15.5, TOP_S + 0.1), (NOSE_I - 3.6, zc + 5.0)])
    l = l.union(prism_y(plate, vy - 7.5, vy + 7.5))
    return l

def tank_hatch():
    tx0, tx1 = S2_X0 - 0.2, S2_X0 + TANK[0] + 2 * CLR + 0.2
    ty0, ty1 = TANK_SLOT[0] - 0.2, WI - 0.6
    h = prism_z(rrect(tx0 - 3, tx1 + 3, ty0 - 3, min(ty1 + 3, W - 0.2), (3, 3, 3, 3)), TOP_S + LID_T, TOP_S + LID_T + 2.0)
    h = h.union(bx(tx0 + 0.25, tx1 - 0.25, ty0 + 0.25, ty1 - 0.25, TOP_S + 0.6, TOP_S + LID_T + 0.1).cut(
        bx(tx0 + 1.85, tx1 - 1.85, ty0 + 1.85, ty1 - 1.85, TOP_S, TOP_S + LID_T)))
    h = h.cut(cyl_z((tx0 + tx1) / 2, (ty0 + ty1) / 2 - 8, 5.5, TOP_S, TOP_S + 10))             # tube grommet (Ø11)
    return h

# ---------------- components (ghosts) ----------------
def components():
    """name -> (cq solid, body, bay, mass g, mass source). body = 'tub' | 'shell' | 'wheel'."""
    C = {}
    def motor(xa, s):
        y0 = s * M_FACE; yg = s * (M_FACE - M_GB_L); yb = s * (M_FACE - M_LEN)
        g = cyl_y(xa, ZGB, M_GB_R, *sorted((y0, yg))).union(cyl_y(xa, ZGB, M_CAN_R, *sorted((yg, yb))))
        return g.union(cyl_y(xa, ZH, 6.0, *sorted((s * M_FACE, s * W)))).union(cyl_y(xa, ZH, 3.0, *sorted((s * W, s * (W + 15)))))
    for (xa, body, nm) in ((XR, 'tub', 'rear'), (XF, 'shell', 'front')):
        bay = 'tub_lower' if body == 'tub' else 'shell_lower'
        for s, side in ((1, 'L'), (-1, 'R')):
            C[f'motor_{nm}_{side}'] = (motor(xa, s), body, bay, 150, 'team')
            C[f'wheel_{nm}_{side}'] = (cyl_y(xa, ZH, 86.6 / 2, *sorted((s * 76.0, s * 114.0))), 'wheel', None, 60, 'estimate')
    C['orange_pi_n515'] = (bx(OPI_X0, OPI_X1, OPI_Y0, OPI_Y1, OPI_Z0, OPI_Z1), 'tub', 'tub_upper', 200, 'estimate (Pi 5 Max + metal case)')
    brd = bx(BOARD_X0, BOARD_X1, BOARD_Y0, BOARD_Y1 + 3.0, BOARD_Z0, BOARD_Z1)   # one solid: pins, PCB, parts, +3 connector overhang
    for (hx, hy) in BOARD_HOLES:
        brd = brd.cut(cyl_z(hx, hy, 3.2, PCB_Z + 1.6, BOARD_Z1 + 1))              # keep-clear round each mounting hole
    C['yahboom_board'] = (brd, 'tub', 'tub_upper', 50, 'estimate')
    C['estop_body'] = (bx(XRI, XRI + ESTOP['depth'], *ESTOP['y'], *ESTOP['z']), 'tub', 'tub_upper', 60, 'estimate')
    ey, ez = sum(ESTOP['y']) / 2, sum(ESTOP['z']) / 2
    C['estop_head'] = (cyl_x(ey, ez, ESTOP['head_r'], XRO - ESTOP['head_h'], XRO), 'tub', None, 0, '')
    C['regulator_5v'] = (bx(*REG['x'], *REG['y'], *REG['z']), 'tub', 'tub_lower', 5, 'Pololu')
    lz = TOP_T + LID_T + LIDAR['plate']; cx = LIDAR['cx']
    C['lidar_d500'] = (bx(cx - 27.0, cx + 27.0, -46.29 / 2, 46.29 / 2, lz, lz + 34.8 - 12.6).union(
        cyl_z(cx, 0, 35.29 / 2, lz + 34.8 - 12.6, lz + 34.8)), 'tub', None, 42, 'LD19 datasheet')
    py0 = -WI + CLR
    C['pump_1'] = (bx(P1_X0, P1_X1, py0, py0 + PUMP[1], DECK_Z1, DECK_Z1 + PUMP[2]), 'shell', 'shell_upper', 75, 'estimate')
    C['pump_2'] = (bx(P2_X0, P2_X1, py0, py0 + PUMP[1], DECK_Z1, DECK_Z1 + PUMP[2]), 'shell', 'shell_upper', 75, 'estimate')
    C['tank_water'] = (bx(S2_X0 + CLR, S2_X0 + CLR + TANK[0], TANK_SLOT[0] + CLR, TANK_SLOT[0] + CLR + TANK[1], S2_FLOOR, S2_FLOOR + TANK[2]),
                       'shell', 'nose', 125, 'estimate (tank ~30 g + ~95 ml water)')
    C['battery_3s2p'] = (bx(S2_X0 + CLR, S2_X0 + CLR + BATT[0], BAT_SLOT[0] + CLR, BAT_SLOT[0] + CLR + BATT[1], S2_FLOOR, S2_FLOOR + BATT[2]),
                         'shell', 'nose', 310, 'team')
    for i in range(2):
        z0 = S2_FLOOR + i * NINE[2]
        C[f'nine_volt_{i + 1}'] = (bx(S2_X0 + CLR, S2_X0 + CLR + NINE[0], NINE_SLOT[0] + CLR, NINE_SLOT[0] + CLR + NINE[1], z0, z0 + NINE[2]),
                                   'shell', 'nose', 30, 'estimate')
    C['esp32_s3'] = (bx(S2_X0 + CLR, S2_X0 + CLR + ESP[0], ESP_SLOT[0] + CLR, ESP_SLOT[0] + CLR + ESP[1], S2_FLOOR, S2_FLOOR + ESP[2]),
                     'shell', 'nose', 10, 'estimate')
    for i, yc in enumerate(XYMOS_Y):
        C[f'xy_mos_{i + 1}'] = (bx(*XYMOS_X, yc - 17, yc + 17, FLOOR, FLOOR + 17), 'shell', 'shell_lower', 8, 'estimate')
    vy = sum(NINE_SLOT) / 2
    v = bx(-12.5, 12.5, -6, 6, -1.6, 1.6).rotate((0, 0, 0), (0, 1, 0), 45).translate((NOSE_I - 7.0, vy, TOP_S - 10.5))
    C['vl53l0x'] = (v, 'shell', 'nose', 1, 'datasheet')
    C['servo_mg90s'] = (bx(SERVO_CX - 11.4, SERVO_CX + 11.4, -W - 12.7, -W - 0.5, SERVO_Z0, SERVO_Z0 + 28.5).union(
        bx(SERVO_CX - 16, SERVO_CX + 16, -W - 12.7, -W - 0.5, SERVO_Z0 + 18.6, SERVO_Z0 + 21)), 'shell', None, 13, 'datasheet')
    return C

def keepouts():
    """space a component needs but does not fill: plugs, fan air, tube bends"""
    K = {}
    K['pi_plug_zone'] = (bx(OPI_X0, OPI_X1, OPI_Y1, OPI_Y1 + OPI_PLUG, OPI_Z0, OPI_Z1), 'tub', 'tub_upper')
    fx, fy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    K['pi_fan_air'] = (bx(fx - 20, fx + 20, fy - 20, fy + 20, OPI_Z1, OPI_Z1 + OPI_AIR), 'tub', 'tub_upper')
    py1 = -WI + CLR + PUMP[1]
    K['pump_1_bends'] = (bx(P1_X0, P1_X1, py1, py1 + BEND, DECK_Z1, DECK_Z1 + PUMP[2]), 'shell', 'shell_upper')
    K['pump_2_bends'] = (bx(P2_X0, P2_X1, py1, py1 + BEND, DECK_Z1, DECK_Z1 + PUMP[2]), 'shell', 'shell_upper')
    return K

def bays():
    """interior regions for fill %: (body, cq region box) - intersected with the cavity by the checker"""
    return {
        'tub_lower':   ('tub',   bx(XRI - 1, J_TUB, -WI, WI, FLOOR, DECK_Z0)),
        'tub_upper':   ('tub',   bx(XRI - 1, J_TUB, -WI, WI, DECK_Z1, TOP_T)),
        'shell_lower': ('shell', bx(J_SH, S2_X0 - DIV, -WI, WI, FLOOR, DECK_Z0)),
        'shell_upper': ('shell', bx(J_SH, S2_X0 - DIV, -WI, WI, DECK_Z1, TOP_S)),
        'nose':        ('shell', bx(S2_X0, NOSE_I, -WI, WI, S2_FLOOR, TOP_S)),
    }

def cavities():
    plan_i = rrect(XRI, J_TUB + 1, -WI, WI, tuple(max(0.5, r - T) for r in R_TUB))
    _, _, tc = hollow(tub_profile(), TOP_T, rrect(XRO, J_TUB + 1, -W, W, R_TUB), plan_i, profile_tall=tub_profile(TOP_T + 20))
    plan_i = rrect(J_SH - 1, NOSE_I, -WI, WI, tuple(max(0.5, r - T) for r in R_SH))
    _, _, sc = hollow(shell_profile(), TOP_S, rrect(J_SH - 1, NOSE, -W, W, R_SH), plan_i, profile_tall=shell_profile(TOP_S + 20, chamfer=0.01))
    return {'tub': tc.intersect(bx(-200, 200, -200, 200, -10, TOP_T)), 'shell': sc.intersect(bx(-200, 300, -200, 200, -10, TOP_S))}

# ---------------- build + export ----------------
PRINT1 = ('main_tub', 'front_shell')
PRINT2 = ('lid_tub', 'lid_shell', 'tank_hatch', 'tub_deck', 'shell_deck', 'lidar_mount')
BUILDERS = dict(main_tub=main_tub, front_shell=front_shell, lid_tub=lid_tub, lid_shell=lid_shell, tank_hatch=tank_hatch,
                tub_deck=tub_deck, shell_deck=shell_deck, lidar_mount=lidar_mount)
BODY_OF = dict(main_tub='tub', lid_tub='tub', tub_deck='tub', lidar_mount='tub',
               front_shell='shell', lid_shell='shell', tank_hatch='shell', shell_deck='shell')
# how each print-2 part lies on the bed: rotation about X in degrees (lids and hatch print top-down)
BED_FLIP = dict(lid_tub=180, lid_shell=180, tank_hatch=180, tub_deck=0, shell_deck=0, lidar_mount=0)

def summary():
    return dict(XRO=XRO, NOSE=NOSE, TOP_T=TOP_T, TOP_S=TOP_S, OPI=(OPI_X0, OPI_X1, OPI_Y0, OPI_Y1), BOARD=(BOARD_X0, BOARD_X1, BOARD_Z0, BOARD_Z1),
                S2_X0=S2_X0, slots=dict(esp=ESP_SLOT, bat=BAT_SLOT, nine=NINE_SLOT, tank=TANK_SLOT), lidar_cx=LIDAR['cx'],
                scan_plane=TOP_T + LID_T + LIDAR['plate'] + LIDAR['scan'])

def export_all(outdir='out'):
    os.makedirs(outdir, exist_ok=True)
    parts = {}
    for name, fn in BUILDERS.items():
        s = fn(); parts[name] = s
        cq.exporters.export(s, f'{outdir}/{name}.stl', tolerance=0.05, angularTolerance=0.15)
        cq.exporters.export(s, f'{outdir}/{name}.step')
        import trimesh
        mm = trimesh.load(f'{outdir}/{name}.stl')
        print(' built', name, round(s.val().Volume() / 1000, 1), 'cm3', 'solids', len(s.solids().vals()),
              'valid', s.val().isValid(), 'watertight', mm.is_watertight)
    comps = components()
    os.makedirs(f'{outdir}/comp', exist_ok=True)
    for name, (s, body, bay, m, src) in comps.items():
        cq.exporters.export(s, f'{outdir}/comp/{name}.stl', tolerance=0.1, angularTolerance=0.3)
    for name, (s, body, bay) in keepouts().items():
        cq.exporters.export(s, f'{outdir}/comp/keepout_{name}.stl', tolerance=0.1, angularTolerance=0.3)
    meta = dict(summary=summary(), body_of=BODY_OF, print1=PRINT1, print2=PRINT2, bed_flip=BED_FLIP,
                components={k: dict(body=v[1], bay=v[2], mass=v[3], src=v[4]) for k, v in comps.items()},
                keepouts={k: dict(body=v[1], bay=v[2]) for k, v in keepouts().items()},
                hinge=dict(x=XH, z=ZH, gap=GAP, rake=RAKE, sweep=SWEEP))
    with open(f'{outdir}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return parts

BED, BED_GAP = 256.0, 3.0
def print_layouts(meshes):
    """meshes: name -> trimesh in the model frame. Returns {1: [(name, mesh on bed)], 2: [...]}.
    Print 1: tub + shell exactly as assembled at 0 deg, belly down, centred.
    Print 2: hand-placed (a rectangle packer gave up at 77% bed cover):
      column A - shell lid, tub lid (both top-down, turned 90 deg), then hatch + LiDAR plate in the strip below;
      column B - pump deck, Pi deck."""
    import trimesh
    T_ = trimesh.transformations
    out = {1: [], 2: []}
    p1 = [meshes[n].copy() for n in PRINT1]
    allm = trimesh.util.concatenate(p1); lo, hi = allm.bounds
    for n, m in zip(PRINT1, p1):
        m.apply_translation([BED / 2 - (lo[0] + hi[0]) / 2, BED / 2 - (lo[1] + hi[1]) / 2, -lo[2]]); out[1].append((n, m))
    def lay(n, flip, turn, x, y):
        m = meshes[n].copy()
        if flip:
            m.apply_transform(T_.rotation_matrix(math.radians(flip), [1, 0, 0]))
        if turn:
            m.apply_transform(T_.rotation_matrix(math.radians(turn), [0, 0, 1]))
        m.apply_translation([x - m.bounds[0][0], y - m.bounds[0][1], -m.bounds[0][2]])
        out[2].append((n, m)); return m.bounds[1]
    g, x0 = BED_GAP, 1.0
    top = lay('lid_shell', 180, 90, x0, 1.0)
    top = lay('lid_tub', 180, 90, x0, top[1] + g)
    hx = lay('tank_hatch', 180, 0, x0, top[1] + g)
    lay('lidar_mount', 0, 0, hx[0] + g, top[1] + g)
    xb = max(m.bounds[1][0] for n, m in out[2] if n.startswith('lid')) + g
    d = lay('shell_deck', 0, 0, xb, 1.0)
    lay('tub_deck', 0, 0, xb, d[1] + g)
    return out

def export_prints(outdir='out', dest='.'):
    """the two deliverables, as single STL files"""
    import trimesh
    meshes = {n: trimesh.load(f'{outdir}/{n}.stl') for n in PRINT1 + PRINT2}
    L = print_layouts(meshes)
    names = {1: 'FPMS_Rover1_Print1_Chassis_Assembled.stl', 2: 'FPMS_Rover1_Print2_Lids_Decks_LiDAR.stl'}
    for k in (1, 2):
        m = trimesh.util.concatenate([mm for _, mm in L[k]])
        m.export(f'{dest}/{names[k]}'); print(names[k], m.extents.round(1), 'bed max', m.bounds[1].round(1))

if __name__ == '__main__':
    import sys
    print(json.dumps(summary(), indent=1))
    export_all()
    if '--prints' in sys.argv:
        export_prints()
