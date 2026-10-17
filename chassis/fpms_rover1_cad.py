# FPMS Rover 1 chassis - CadQuery source, v2 (single file).
# Frame: X forward (rear axle at X=0), Y left, Z up, mm. Ground at Z=0.
# Print 1 = main tub + front shell printed ASSEMBLED, hinge printed in place (0.4 mm, teardrop bores).
# Print 2 = the two caps (rounded top 8 mm of each body) + tank hatch + removable decks + board shelf + LiDAR plate.
# Sources for every part dimension: RESEARCH.md. Team measurements win over online numbers.
# Run:  pip install cadquery trimesh manifold3d shapely   then   python fpms_rover1_cad.py [--prints]
#
# v2 (Nikhil's review of v1): <= 210 long, nose <= 20 past the worst-case wheel with a 45 deg underside from behind
# the wheel, 0.45 per side in every pocket, nothing outside the body outline, equal tops, R8 rounds, no add-on boxes.
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

# ---------------- front shell layout ----------------
# The nose may only reach 20 past the worst-case wheel and its underside rises at 45 deg from behind the wheel,
# so nothing big can stand on a floor ahead of the front motors any more. Everything stands at deck level:
#   rear half (over the front motors): the two pumps STACKED, barbs at opposite ends so each bend zone sits
#   beside the other pump;  front half (nose bay): battery, tank, 9 V cells, ESP32, standing on printed ribs.
PUMP = (50.0, 100.0, 40.0)          # lying: 50 deep (X), 100 across (Y, barbs at one end), 40 tall
BEND = 30.0                         # 6x9 silicone, R25 bend + tube radius beyond the barb tip
P_X0 = J_SH + T + CLR
P_X1 = P_X0 + PUMP[0]
P_Z = (DECK_Z1, DECK_Z1 + PUMP[2], DECK_Z1 + 2 * PUMP[2])   # lower pump on the pump deck, upper pump on the lower one
DIV = 1.2
S2_X0 = P_X1 + CLR + DIV            # nose bay starts behind its divider
BATT = (50.0, 65.0, 75.0)           # 65x75x50 pack standing, 50 deep
TANK = (40.0, 40.0, 80.0)
TANK_Z0 = DECK_Z0                   # tank on the ribs like the battery (the R6 side round clears it)
PUMP_UP_SHIFT = 4.5                 # upper pump moved off the left wall for the same reason (its bend zone still fits)
NINE = (48.5, 17.5, 26.5)           # 9 V Li-ion lying along X, two stacked
ESP = (25.4, 13.0, 68.8)            # ESP32-S3 DevKitC-1 standing, USB up: 62.74 PCB + ~6 antenna overhang
SERVO = dict(body=(22.8, 12.2, 28.5), ears=32.0, ear_z=18.6, shaft_from_front=5.9)
NOSE_I = S2_X0 + CLR + BATT[0] + CLR
NOSE = NOSE_I + T
RAMP_X0 = 119.0                     # 45 deg rise starts 14.5 behind the Ø80 wheel front: belly is above the axle by X 133.5,
                                    # so a curb meets the wheel first with either wheel (checked in checks.py)
_y = -WI                            # nose bay slots across Y, right (-Y) to left
ESP_OFF = 0.4                       # ESP32 stands 0.85 off the wall: its top corner clears the cap's side round
ESP_SLOT = (_y, _y + ESP[1] + 2 * CLR + ESP_OFF); _y = ESP_SLOT[1] + DIV
BAT_SLOT = (_y, _y + BATT[1] + 2 * CLR); _y = BAT_SLOT[1] + DIV
NINE_SLOT = (_y, _y + NINE[1] + 2 * CLR); _y = NINE_SLOT[1] + DIV
TANK_SLOT = (_y, WI)
assert TANK_SLOT[1] - TANK_SLOT[0] >= TANK[1] + 2 * CLR - 1e-6, ('nose bay too narrow', TANK_SLOT)
H_IN = max(P_Z[2], TANK_Z0 + TANK[2], DECK_Z0 + BATT[2]) + CLR      # inside of both caps
H = math.ceil((H_IN + T) * 2) / 2                                    # one top line for tub and shell
H_IN = H - T
CAP = 8.0                           # the rounded top 8 mm of each body is a separate cap (print 2)
Z_SPLIT = H - CAP
R_V, R_TOP, R_BELLY = 8.0, 8.0, 4.0 # outside vertical edges, front/rear top edges, belly edges (belly: outside only)
R_TOP_SIDE = 6.0                    # long top edges: R6 keeps the pump and ESP32 ends clear of the round
R_JOINT = 6.0                       # vertical edges at the joint faces
# servo: inside the right front corner, shaft up near its front end so the refill arm swings out ahead of the wheel
NOSE_SLOPE_IN = lambda x: x - RAMP_X0 + BELLY + T * math.sqrt(2)    # inner surface height of the 45 deg nose slope
ESP_X1 = None                                                        # set below once the nose bay is known
PAD_HALF = 19.5                                                      # motor pad: 1 mm of wall round the outer counterbores

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
XYMOS_X, XYMOS_Y = (62.5, 74.5), (-27.5, 27.5)                   # standing in the shell bay behind the front motors
LIDAR = dict(fp=(46.29, 54.0), h=34.8, scan=27.0, holes=(46.8, 31.92), plate=4.0)
LIDAR['cx'] = round((XRO + R_TOP + tub_face_x(H) - R_TOP) / 2, 1)   # middle of the flat part of the tub cap
LIDAR_HOLES = [(LIDAR['cx'] - 8.7, -23.4), (LIDAR['cx'] - 8.7, 23.4), (LIDAR['cx'] + 23.2, 0.0)]
LIDAR_PADS = [(LIDAR['cx'] + dx, dy) for dx in (-15.0, 15.0) for dy in (-17.0, 17.0)]
LIDAR_LEAD = (LIDAR['cx'] - 6, LIDAR['cx'] + 6, -6, 6)

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
    tip = affinity.translate(pin2d(), 0, -0.45).buffer(GAP, 32).intersection(sbox(XH - 4, ZH - 20, XH + 4, ZH - PIN_R))
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
def shell_plans():
    return (rrect(J_SH, NOSE, -W, W, (R_JOINT, R_V, R_V, R_JOINT)),
            rrect(J_SH - 5, NOSE_I, -WI, WI, (0, R_V - T, R_V - T, 0)))
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
        blk = blk.cut(bx(xa - 20, xa + 20, u0, u1, FLOOR - 0.6, ZGB - M_CAN_R + 3.0))                               # tie tunnel under the can
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
    L, D, z0 = 7.0, 6.0, Z_SPLIT - 9.0
    prof = Polygon([(y_wall, H_IN), (y_wall + into * D, H_IN), (y_wall + into * D, z0), (y_wall, z0 - D)])
    if along == 'x':
        g = prism_x(prof, x - L / 2, x + L / 2); hx, hy = x, y_wall + into * D / 2
    else:
        g = prism_y(prof, x - L / 2, x + L / 2); hx, hy = y_wall + into * D / 2, x
    return w.union(g).cut(cyl_z(hx, hy, 1.3, H_IN - 10, H_IN + 1)), (hx, hy)

TUB_BOSSES, SHELL_BOSSES = [], []
CABLE_PASS = Polygon([(-10, 33), (10, 33), (10, 47), (0, 57), (-10, 47)])   # YZ: 20 wide, 45 deg pointed roof

def deck_post(w, x, y, ztop, floor_z=FLOOR):
    w = w.union(bx(x - 2.5, x + 2.5, y - 2.5, y + 2.5, floor_z - 0.5, ztop))
    return w.union(cyl_z(x, y, 1.4, ztop - 0.1, ztop + 2.0))

TUB_DECK_X1 = OPI_X1 + CLR + 1.6 + 3.0
TUB_DECK_Y1 = OPI_Y1 + 1.5

import functools
@functools.lru_cache(None)
def tub_raw():
    """full tub body (print 1 part + cap), before the split"""
    po, pi = tub_plans()
    outer, inner = body(tub_profile(), po, pi)
    return outer.cut(inner)

def main_tub():
    t = tub_raw().intersect(BELOW())
    t = motor_mount(t, XR)
    t = channel(t, M_GB_R + 1, tub_face_in(FLOOR) - 1)
    # Pi deck: rear-wall ledge, right-wall ledge, 2 posts on its free edge
    t = t.union(ledge(XRI, XRI + 3, -WI + 10, OPI_Y1 - 4, DECK_Z0, axis='x', into=+1))
    t = t.union(ledge(XRI + 12, -PAD_HALF, -WI, -WI + 3, DECK_Z0, axis='y', into=+1))
    for (px, py) in ((XRI + 8.0, TUB_DECK_Y1 - 4.0), (TUB_DECK_X1 - 5.0, TUB_DECK_Y1 - 4.0)):
        floor_here = FLOOR if px > TAIL_X - 2 else FLOOR + (TAIL_X - px) - 7      # sinks well into the sloped tail wall
        t = deck_post(t, px, py, DECK_Z0, floor_here)
    # board shelf ledges: right wall, rear wall, and a gusset on the raked front wall
    t = t.union(ledge(XRI + 10, tub_face_in(SHELF_Z0) - 1, -WI, -WI + 3, SHELF_Z0, axis='y', into=+1))
    t = t.union(ledge(XRI, XRI + 3, -WI + 10, BOARD_Y1 - 2, SHELF_Z0, axis='x', into=+1))
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
    return t

@functools.lru_cache(None)
def shell_raw():
    po, pi = shell_plans()
    outer, inner = body(shell_profile(), po, pi)
    return outer.cut(inner), inner

def front_shell():
    raw, cav = shell_raw()
    f = raw.intersect(BELOW())
    f = motor_mount(f, XF)
    f = channel(f, J_SH + T + 1, XF - M_GB_R - 1)
    # pump deck: rear-wall ledge, side-wall ledges behind the pads (pad tops carry it over the motors), front ledge
    f = f.union(ledge(J_SH + T, J_SH + T + 3, -WI + 4, WI - 4, DECK_Z0, axis='x', into=+1))
    for s in (1, -1):
        y0, y1 = sorted((s * WI, s * (WI - 3)))
        f = f.union(ledge(J_SH + T + 14, XF - PAD_HALF, y0, y1, DECK_Z0, axis='y', into=-s))
    # nose-bay rear divider: full height (it also carries the cap and the tank hatch)
    # (the nose-bay rear wall rides on the pump deck: over the motors it would start in mid-air)
    # ribs: items in the nose bay stand on rib tops at DECK_Z0 (vertical walls, no bridge, nothing over the motor)
    for yc in RIBS_Y:
        ztop = TANK_Z0 if TANK_SLOT[0] < yc < TANK_SLOT[1] else DECK_Z0
        f = f.union(bx(XF + M_GB_R + 1.5, NOSE_I + 0.5, yc - 0.8, yc + 0.8, FLOOR - 0.5, ztop).intersect(cav))
    # slot dividers (Y walls): servo/ESP | battery | 9V | tank. 9V|tank divider and the tank's front wall go to the cap
    xm = XF + M_GB_R + 1.5                                    # ahead of the front motors a wall may reach the floor
    for yb, ztop in ((ESP_SLOT[1], 118.0), (BAT_SLOT[1], 118.0), (NINE_SLOT[1], H_IN)):
        f = f.union(bx(xm - 0.01, NOSE_I + 0.5, yb, yb + DIV, FLOOR - 0.5, ztop).intersect(cav))
    tx1 = S2_X0 + TANK[0] + 2 * CLR
    f = f.union(bx(tx1, tx1 + DIV, TANK_SLOT[0], WI + 0.5, TANK_Z0 - 4, H_IN).intersect(cav))
    # ESP32: ledges on both slot walls carry it above the servo; USB-C comes up to the cap slot
    ez0 = ESP_Z0 - 0.0
    for s, yw in ((+1, ESP_SLOT[0]), (-1, ESP_SLOT[1])):
        y0, y1 = sorted((yw, yw + s * 2.5))
        f = f.union(ledge(S2_X0 + 1, SERVO_X1 - SERVO['body'][0] - CLR, y0, y1, ez0, d=2.5, axis='y', into=s))
    # servo holder: two posts under the ears with M2 pilots, body pocket between them
    sb0, sb1 = SERVO_X1 - SERVO['body'][0], SERVO_X1
    f = f.union(bx(SERVO_EARS[0], XF + PAD_HALF, -WI, -M_FACE, DECK_Z0 - 0.1, DECK_Z0 + SERVO_SHIM))   # shim on the pad
    for (p0, p1) in ((sb1 + CLR, SERVO_EARS[1]),):            # front ear on a post; the rear ear sits on the pad shim
        post = bx(p0, p1, ESP_SLOT[0], ESP_SLOT[1], FLOOR - 0.5, SERVO_Z0 + SERVO['ear_z']).intersect(cav)
        f = f.union(post).cut(cyl_z((p0 + p1) / 2, sum(ESP_SLOT) / 2, 0.9, SERVO_Z0 + SERVO['ear_z'] - 8, SERVO_Z0 + 30))
    # refill-arm window in the right wall, ahead of the wheel, at horn height
    wm = (ARM_WIN[0] + ARM_WIN[1]) / 2
    f = f.cut(prism_y(Polygon([(ARM_WIN[0], SERVO_TOP + 1), (ARM_WIN[1], SERVO_TOP + 1), (ARM_WIN[1], SERVO_TOP + 7),
                               (wm, SERVO_TOP + 7 + (ARM_WIN[1] - wm)), (ARM_WIN[0], SERVO_TOP + 7)]), -W - 5, -WI + 0.5))
    # XY-MOS x2 standing in the bay behind the front motors (wired once, next to the pumps they switch)
    for yc in XYMOS_Y:
        x0, x1 = XYMOS_X[0] - CLR, XYMOS_X[1] + CLR
        y0, y1 = yc - 17 - CLR, yc + 17 + CLR
        f = f.union(bx(x0 - 1.2, x1 + 1.2, y0 - 1.2, y1 + 1.2, FLOOR - 0.5, FLOOR + 8).cut(bx(x0, x1, y0, y1, FLOOR, FLOOR + 10)))
    # VL53L0X: window through the 45 deg nose slope, sensor lies on the slope looking 45 deg down-forward
    vx, vy = VL53_X, VL53_Y
    vz = NOSE_SLOPE_IN(vx)
    win = bx(-4.5, 4.5, -4.0, 4.0, -6, 6).rotate((0, 0, 0), (0, 1, 0), -45).translate((vx + 1.2, vy, vz - 1.2))
    f = f.cut(win)
    for dy in (-7.6, 7.6):     # two rails along the slope hold the 12 mm board edges
        rail = bx(-14, 14, -1.0, 1.0, 0, 2.2).rotate((0, 0, 0), (0, 1, 0), -45).translate((vx, vy + dy, vz))
        f = f.union(rail.intersect(cav))
    # cap bosses: rear wall (right of the upper pump's bend zone), both front corners
    SHELL_BOSSES.clear()
    f, h = cap_boss(f, -WI + 4.1, J_SH + T, +1, along='y'); SHELL_BOSSES.append(h)
    for s in (1, -1):
        f, h = cap_boss(f, NOSE_I - 5.0, s * WI, -s); SHELL_BOSSES.append(h)
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
    for (dx, dy) in ((100, 0), (118, -20)):
        f = f.cut(cyl_z(dx, dy, 2, BELLY - 1, FLOOR + 2))
    return f

# derived positions used above
ESP_X1 = S2_X0 + CLR + ESP[0]
SERVO_X1 = ESP_X1 + CLR + 2.4 + SERVO['shaft_from_front']            # shaft (Ø4.8) just ahead of the ESP32 above it
SERVO_SHAFT_X = SERVO_X1 - SERVO['shaft_from_front']
SERVO_Z0 = NOSE_SLOPE_IN(SERVO_X1) + CLR                             # body front-bottom edge 0.45 above the nose slope
SERVO_SHIM = SERVO_Z0 + SERVO['ear_z'] - DECK_Z0                     # rear ear rests on a shim on the right front motor pad
RIBS_Y = (-47.0, -24.0, 2.0, sum(NINE_SLOT) / 2, TANK_SLOT[0] + 9.0, TANK_SLOT[1] - 9.0)
SERVO_TOP = SERVO_Z0 + SERVO['body'][2]
SERVO_EARS = (SERVO_X1 - SERVO['body'][0] / 2 - SERVO['ears'] / 2, SERVO_X1 - SERVO['body'][0] / 2 + SERVO['ears'] / 2)
ESP_Z0 = SERVO_TOP + CLR
ESP_Z1 = ESP_Z0 + ESP[2]
OVER_MOTOR_Z = 64.2                 # walls above the front motors start here (gearbox top 65.5 at X 93.5, 63.4 at X 102)
ARM_WIN = (SERVO_SHAFT_X - 8.0, NOSE_I - 1.0)   # refill-arm window, right wall, at horn height
# at the arm's height the worst-case wheel only reaches X = 93.5 + sqrt(R^2 - (z - 40)^2): the window must start ahead of that
ARM_WHEEL_X = XF + math.sqrt(max(0.0, WHEEL_R_WORST ** 2 - (SERVO_TOP + 1 - ZH) ** 2))
assert ARM_WIN[0] > ARM_WHEEL_X + 2, ('refill arm would meet the wheel', ARM_WIN, ARM_WHEEL_X)
VL53_X, VL53_Y = NOSE_I - 14.0, -35.5
assert ESP_Z1 <= H_IN - CLR, ('ESP32 does not fit over the servo', ESP_Z1, H_IN)

# ---------------- print 2 ----------------
def cap_of(raw, trim_at_joint):
    c = raw.intersect(ABOVE()).cut(trim_at_joint)
    return c

def tub_cap():
    c = cap_of(tub_raw(), prism_y(Polygon([(tub_face_x(Z_SPLIT - 30) - 0.6, Z_SPLIT - 30), (200, Z_SPLIT - 30), (200, H + 50),
                                           (tub_face_x(H + 50) - 0.6, H + 50)]), -W - 5, W + 5))
    for (hx, hy) in TUB_BOSSES:
        c = c.cut(cyl_z(hx, hy, 1.7, H_IN - 1, H + 1))
    for (px, py) in LIDAR_PADS:                                       # LiDAR plate screws: pads under the cap (prints top-down)
        c = c.union(cyl_z(px, py, 3.5, H_IN - 5.0, H_IN + 0.1)).cut(cyl_z(px, py, 1.25, H_IN - 6, H + 1))
    c = c.cut(prism_z(rrect(*LIDAR_LEAD, (2, 2, 2, 2)), H_IN - 1, H + 1))
    return c

def shell_cap():
    c = cap_of(shell_raw()[0], bx(J_SH - 10, J_SH + 0.6, -W - 5, W + 5, Z_SPLIT - 1, H + 5))
    c = c.cut(hatch_box(0.0))                                         # tank lifts straight out here
    for (hx, hy) in SHELL_BOSSES:
        c = c.cut(cyl_z(hx, hy, 1.7, H_IN - 1, H + 1))
    c = c.cut(prism_z(rrect(S2_X0 + 3, S2_X0 + ESP[0] - 3, ESP_SLOT[0] + 4.0, ESP_SLOT[1] - 2.0, (2, 2, 2, 2)), H_IN - 1, H + 1))  # ESP32 USB-C
    sx = SERVO_SHAFT_X
    return c

def hatch_box(shrink):
    tx0 = S2_X0 - DIV / 2 + shrink
    tx1 = S2_X0 + TANK[0] + 2 * CLR + DIV / 2 - shrink
    ty0 = TANK_SLOT[0] - DIV / 2 + shrink
    return bx(tx0, tx1, ty0, W + 10, Z_SPLIT - 1, H + 5)

def tank_hatch():
    """a piece of the shell cap over the tank: rests on the two full-height walls and the left wall top"""
    c = shell_raw()[0].intersect(ABOVE()).intersect(hatch_box(0.3))
    tx = (S2_X0 + S2_X0 + TANK[0]) / 2
    return c.cut(cyl_z(tx, (TANK_SLOT[0] + WI) / 2 - 6, 5.5, H_IN - 1, H + 1))     # Ø11 tube grommet

def tub_deck():
    d = prism_z(rrect(XRI + 0.3, TUB_DECK_X1, -WI + 0.3, TUB_DECK_Y1, (max(0.5, R_V - T - 0.3), 1, 2, 2)), DECK_Z0, DECK_Z1)
    fr = bx(OPI_X0 - CLR - 1.6, OPI_X1 + CLR + 1.6, OPI_Y0 - CLR - 1.6, OPI_Y1 - 2.0, DECK_Z1 - 0.1, DECK_Z1 + 6).cut(
        bx(OPI_X0 - CLR, OPI_X1 + CLR, OPI_Y0 - CLR, OPI_Y1 + 5, DECK_Z1 - 1, DECK_Z1 + 7))
    d = d.union(fr)
    cx, cy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    for dy in (-32.5, 32.5):
        d = d.cut(cyl_z(cx, cy + dy, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))
    for (px, py) in ((XRI + 8.0, TUB_DECK_Y1 - 4.0), (TUB_DECK_X1 - 5.0, TUB_DECK_Y1 - 4.0)):
        d = d.cut(cyl_z(px, py, 1.7, DECK_Z0 - 1, DECK_Z1 + 1))
    return d.cut(prism_z(rrect(cx - 22, cx + 22, cy - 30, cy + 30, (6, 6, 6, 6)), DECK_Z0 - 1, DECK_Z1 + 1))

def tub_shelf():
    x1 = tub_face_in(SHELF_Z1) - 0.3
    y1 = BOARD_Y1 + 1.0
    s = prism_z(rrect(XRI + 0.3, x1, -WI + 0.3, y1, (max(0.5, R_V - T - 0.3), 1, 1, 1)), SHELF_Z0, SHELF_Z1)
    s = s.union(bx(XRI + 4, x1 - 4, y1 - 2.0, y1, SHELF_Z0 - 7, SHELF_Z0))         # stiffening rib under the free edge
    for (hx, hy) in BOARD_HOLES:
        s = s.cut(cyl_z(hx, hy, 1.35, SHELF_Z0 - 1, BOARD_Z0 + 1))              # M2.5 through: board on 2 mm nylon spacers
    fx, fy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2                           # open over the Pi's fan
    return s.cut(prism_z(rrect(fx - 18, fx + 18, fy - 18, fy + 18, (4, 4, 4, 4)), SHELF_Z0 - 1, SHELF_Z1 + 1))

def pump_deck():
    x0, x1 = J_SH + T + 0.3, S2_X0 - 0.3
    d = prism_z(rrect(x0, x1, -WI + 0.3, WI - 0.3, (1, 1, 1, 1)), DECK_Z0, DECK_Z1)
    # nose-bay rear wall stands on the deck: full height where the tank hatch rests on it, lower elsewhere (cables pass over)
    w0, x1 = P_X1 + CLR, S2_X0                                   # 0.45 from the pumps and from the nose-bay parts
    d = d.union(bx(w0, x1, TANK_SLOT[0] - DIV, WI - 0.3, DECK_Z1 - 0.1, H_IN - 0.05))
    d = d.union(bx(w0, x1, -WI + 0.3, TANK_SLOT[0] - DIV + 0.01, DECK_Z1 - 0.1, 125.0))
    # gussets on the pump side, only where neither pump nor a bend zone is: left end full height, right end above the lower pump
    py1 = WI - CLR - PUMP_UP_SHIFT
    for (ya, yb, za, zb) in ((py1 + CLR, WI - 0.3, DECK_Z1, H_IN - 0.05), (-WI + 0.3, -WI + CLR + PUMP[1] - PUMP[1] + 7.5, P_Z[1] + CLR, 125.0)):
        d = d.union(prism_y(Polygon([(w0 + 0.01, za), (w0 - 8, za + 8), (w0 + 0.01, za + 8)]), ya, yb).union(bx(w0 - 2.5, w0 + 0.01, ya, yb, za, zb)))
    for yy in (-45.0, 45.0):                                                     # two straps round the pump stack
        for xx in (x0 + 0.8, w0 - 9.8):
            d = d.cut(bx(xx, xx + 3.0, yy - 3.5, yy + 3.5, DECK_Z0 - 1, DECK_Z1 + 1))
    d = d.cut(prism_z(rrect(XF - 10, XF + 6, -6, 6, (2, 2, 2, 2)), DECK_Z0 - 1, DECK_Z1 + 1))   # plug access over the motor gap
    return d

def lidar_mount():
    """flat plate for the D500 on the tub cap: 4 x M3 into the cap pads, 3 x M2.5 from the LiDAR into the plate"""
    cx, z0 = LIDAR['cx'], H
    m = prism_z(rrect(cx - LIDAR['fp'][0] / 2 - 1.5, cx + LIDAR['fp'][0] / 2 + 5.5, -LIDAR['fp'][1] / 2 - 1.5, LIDAR['fp'][1] / 2 + 1.5,
                      (5, 5, 5, 5)), z0, z0 + LIDAR['plate'])
    m = try_fillet(m, '>Z', 1.0)
    for (hx, hy) in LIDAR_HOLES:
        m = m.cut(cyl_z(hx, hy, 1.1, z0 + 0.8, z0 + 6))
    for (px, py) in LIDAR_PADS:
        m = m.cut(cyl_z(px, py, 1.7, z0 - 1, z0 + 6)).cut(cyl_z(px, py, 3.0, z0 + LIDAR['plate'] - 1.8, z0 + 6))
    return m.cut(prism_z(rrect(*LIDAR_LEAD, (2, 2, 2, 2)), z0 - 1, z0 + 6))

# ---------------- components (ghosts) ----------------
def components():
    """name -> (cq solid, body, bay, mass g, mass source, seat). body = 'tub' | 'shell' | 'wheel'.
    seat = how it is held: the faces that are SUPPOSED to touch (excluded from the clearance check)."""
    C = {}
    def motor(xa, s):
        y0 = s * M_FACE; yg = s * (M_FACE - M_GB_L); yb = s * (M_FACE - M_LEN)
        g = cyl_y(xa, ZGB, M_GB_R, *sorted((y0, yg))).union(cyl_y(xa, ZGB, M_CAN_R, *sorted((yg, yb))))
        return g.union(cyl_y(xa, ZH, 6.0, *sorted((s * M_FACE, s * W)))).union(cyl_y(xa, ZH, 3.0, *sorted((s * W, s * (W + 15)))))
    for (xa, body, nm) in ((XR, 'tub', 'rear'), (XF, 'shell', 'front')):
        bay = 'tub_lower' if body == 'tub' else 'shell_lower'
        for s, side in ((1, 'L'), (-1, 'R')):
            C[f'motor_{nm}_{side}'] = (motor(xa, s), body, bay, 150, 'team', dict(face_y=s * M_FACE))
            C[f'wheel_{nm}_{side}'] = (cyl_y(xa, ZH, WHEEL_R_WORST, *sorted((s * 76.0, s * 114.0))), 'wheel', None, 60, 'estimate', None)
    C['orange_pi_n515'] = (bx(OPI_X0, OPI_X1, OPI_Y0, OPI_Y1, OPI_Z0, OPI_Z1), 'tub', 'tub_upper', 200, 'estimate (Pi 5 Max + metal case)', 'bottom')
    brd = bx(BOARD_X0, BOARD_X1, BOARD_Y0, BOARD_Y1 + 3.0, BOARD_Z0, BOARD_Z1)
    C['yahboom_board'] = (brd, 'tub', 'tub_upper', 50, 'estimate', 'bottom')
    C['estop_body'] = (bx(XRI, XRI + ESTOP['depth'], *ESTOP['y'], *ESTOP['z']), 'tub', 'tub_upper', 60, 'estimate', dict(face_x=XRI))
    ey, ez = sum(ESTOP['y']) / 2, sum(ESTOP['z']) / 2
    C['estop_head'] = (cyl_x(ey, ez, ESTOP['head_r'], XRO - ESTOP['head_h'], XRO), 'tub', None, 0, '', None)
    C['regulator_5v'] = (bx(*REG['x'], *REG['y'], *REG['z']), 'tub', 'tub_lower', 5, 'Pololu', 'bottom')
    cx = LIDAR['cx']; lz = H + LIDAR['plate']
    C['lidar_d500'] = (bx(cx - LIDAR['fp'][0] / 2, cx + LIDAR['fp'][0] / 2, -LIDAR['fp'][1] / 2, LIDAR['fp'][1] / 2, lz, lz + 34.8 - 12.6).union(
        cyl_z(cx, 0, 35.29 / 2, lz + 34.8 - 12.6, lz + 34.8)), 'tub', None, 42, 'LD19 datasheet', 'bottom')
    py0, py1 = -WI + CLR, WI - CLR
    C['pump_lower'] = (bx(P_X0, P_X1, py0, py0 + PUMP[1], P_Z[0], P_Z[1]), 'shell', 'shell_upper', 75, 'estimate', 'bottom')
    C['pump_upper'] = (bx(P_X0, P_X1, py1 - PUMP_UP_SHIFT - PUMP[1], py1 - PUMP_UP_SHIFT, P_Z[1], P_Z[2]), 'shell', 'shell_upper', 75, 'estimate', 'bottom')
    x0 = S2_X0 + CLR
    C['tank_water'] = (bx(x0, x0 + TANK[0], TANK_SLOT[0] + CLR, TANK_SLOT[0] + CLR + TANK[1], TANK_Z0, TANK_Z0 + TANK[2]),
                       'shell', 'nose', 125, 'estimate (tank ~30 g + ~95 ml water)', 'bottom')
    C['battery_3s2p'] = (bx(x0, x0 + BATT[0], BAT_SLOT[0] + CLR, BAT_SLOT[0] + CLR + BATT[1], DECK_Z0, DECK_Z0 + BATT[2]),
                         'shell', 'nose', 310, 'team', 'bottom')
    for i in range(2):
        z0 = DECK_Z0 + i * NINE[2]
        C[f'nine_volt_{i + 1}'] = (bx(x0, x0 + NINE[0], NINE_SLOT[0] + CLR, NINE_SLOT[0] + CLR + NINE[1], z0, z0 + NINE[2]),
                                   'shell', 'nose', 30, 'estimate', 'bottom')
    C['esp32_s3'] = (bx(x0, x0 + ESP[0], ESP_SLOT[0] + CLR + ESP_OFF, ESP_SLOT[0] + CLR + ESP_OFF + ESP[1], ESP_Z0, ESP_Z1), 'shell', 'nose', 10, 'estimate', 'bottom')
    for i, yc in enumerate(XYMOS_Y):
        C[f'xy_mos_{i + 1}'] = (bx(*XYMOS_X, yc - 17, yc + 17, FLOOR, FLOOR + 17), 'shell', 'shell_lower', 8, 'estimate', 'bottom')
    vz = NOSE_SLOPE_IN(VL53_X)
    C['vl53l0x'] = (bx(-12.5, 12.5, -6, 6, 0, 3.2).rotate((0, 0, 0), (0, 1, 0), -45).translate((VL53_X, VL53_Y, vz + CLR * 1.42)),
                    'shell', 'shell_lower', 1, 'datasheet', dict(plane=[-1 / math.sqrt(2), 0, 1 / math.sqrt(2), (vz - VL53_X) / math.sqrt(2)]))
    sb = SERVO['body']; sy0, sy1 = ESP_SLOT[0] + CLR, ESP_SLOT[0] + CLR + sb[1]
    C['servo_mg90s'] = (bx(SERVO_X1 - sb[0], SERVO_X1, sy0, sy1, SERVO_Z0, SERVO_TOP).union(
        bx(SERVO_EARS[0], SERVO_EARS[1], sy0, sy1, SERVO_Z0 + SERVO['ear_z'], SERVO_Z0 + SERVO['ear_z'] + 2.5)).union(
        cyl_z(SERVO_SHAFT_X, (sy0 + sy1) / 2, 2.4, SERVO_TOP - 0.01, SERVO_TOP + 4.0)), 'shell', 'nose', 13, 'datasheet',
        dict(ear_z=SERVO_Z0 + SERVO['ear_z']))
    return C

def keepouts():
    """space a component needs but does not fill: plugs, fan air, tube bends"""
    K = {}
    K['pi_plug_zone'] = (bx(OPI_X0, OPI_X1, OPI_Y1, OPI_Y1 + OPI_PLUG, OPI_Z0, OPI_Z1), 'tub', 'tub_upper')
    fx, fy = (OPI_X0 + OPI_X1) / 2, (OPI_Y0 + OPI_Y1) / 2
    K['pi_fan_air'] = (bx(fx - 20, fx + 20, fy - 20, fy + 20, OPI_Z1, OPI_Z1 + OPI_AIR), 'tub', 'tub_upper')
    py0, py1 = -WI + CLR, WI - CLR
    K['pump_lower_bends'] = (bx(P_X0, P_X1, py0 + PUMP[1], py0 + PUMP[1] + BEND, P_Z[0], P_Z[1]), 'shell', 'shell_upper')
    K['pump_upper_bends'] = (bx(P_X0, P_X1, py1 - PUMP_UP_SHIFT - PUMP[1] - BEND, py1 - PUMP_UP_SHIFT - PUMP[1], P_Z[1], P_Z[2]), 'shell', 'shell_upper')
    return K

def bays():
    """interior regions for fill % (intersected with each body's cavity by the checker)"""
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
PRINT2 = ('tub_cap', 'shell_cap', 'tank_hatch', 'tub_deck', 'tub_shelf', 'pump_deck', 'lidar_mount')
BUILDERS = dict(main_tub=main_tub, front_shell=front_shell, tub_cap=tub_cap, shell_cap=shell_cap, tank_hatch=tank_hatch,
                tub_deck=tub_deck, tub_shelf=tub_shelf, pump_deck=pump_deck, lidar_mount=lidar_mount)
BODY_OF = dict(main_tub='tub', tub_cap='tub', tub_deck='tub', tub_shelf='tub', lidar_mount='tub',
               front_shell='shell', shell_cap='shell', tank_hatch='shell', pump_deck='shell')
BED_FLIP = dict(tub_cap=180, shell_cap=180, tank_hatch=180, tub_deck=0, tub_shelf=180, pump_deck=0, lidar_mount=0)

def summary():
    return dict(version='v2', XRO=XRO, NOSE=NOSE, H=H, TOP_T=H, TOP_S=H, Z_SPLIT=Z_SPLIT, length=round(NOSE - XRO, 1),
                nose_overhang=round(NOSE - WHEEL_FRONT, 1), wheel_front=WHEEL_FRONT, ramp_x0=RAMP_X0,
                OPI=(OPI_X0, OPI_X1, OPI_Y0, OPI_Y1), BOARD=(BOARD_X0, BOARD_X1, BOARD_Z0, BOARD_Z1), S2_X0=S2_X0,
                slots=dict(esp=ESP_SLOT, bat=BAT_SLOT, nine=NINE_SLOT, tank=TANK_SLOT), lidar_cx=LIDAR['cx'],
                scan_plane=H + LIDAR['plate'] + LIDAR['scan'], servo=dict(x1=SERVO_X1, z0=SERVO_Z0, shaft_x=SERVO_SHAFT_X, arm_window=ARM_WIN, wheel_x_at_arm=round(ARM_WHEEL_X, 1)),
                esp_z=(ESP_Z0, ESP_Z1))

def export_all(outdir='out'):
    import trimesh
    os.makedirs(outdir, exist_ok=True)
    for f in os.listdir(outdir):
        if f.endswith('.stl') or f.endswith('.step'):
            os.remove(os.path.join(outdir, f))                       # no stale parts from older versions
    parts = {}
    for name in PRINT1 + PRINT2:
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
    meta = dict(summary=summary(), body_of=BODY_OF, print1=PRINT1, print2=PRINT2, bed_flip=BED_FLIP,
                components={k: dict(body=v[1], bay=v[2], mass=v[3], src=v[4], seat=v[5]) for k, v in comps.items()},
                keepouts={k: dict(body=v[1], bay=v[2]) for k, v in keepouts().items()},
                hinge=dict(x=XH, z=ZH, gap=GAP, rake=RAKE, sweep=SWEEP))
    with open(f'{outdir}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return parts

BED, BED_GAP = 256.0, 3.0
P1S_EXCL = (18.0, 28.0)             # P1S bed_exclude_area: front-left corner X 0-18, Y 0-28 must stay empty
def print_layouts(meshes):
    """meshes: name -> trimesh in the model frame. Returns {1: [(name, mesh on bed)], 2: [...]}.
    Print 1: tub + shell exactly as assembled at 0 deg, belly down, centred.
    Print 2: hand-placed, column A = shell cap, tub cap, pump deck (caps top-down, turned 90 deg);
             column B = Pi deck, board shelf, LiDAR plate + tank hatch."""
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
    top = lay('shell_cap', 180, 90, x0, P1S_EXCL[1] + 1.5)        # starts above the P1S keep-out corner
    top = lay('tub_cap', 180, 90, x0, top[1] + g)
    lay('pump_deck', 0, 90, x0, top[1] + g)
    xb = max(m.bounds[1][0] for n, m in out[2]) + g
    top = lay('tub_deck', 0, 0, xb, 1.0)
    top = lay('tub_shelf', 180, 0, xb, top[1] + g)
    t2 = lay('lidar_mount', 0, 90, xb, top[1] + g)
    lay('tank_hatch', 180, 0, t2[0] + g, top[1] + g)
    return out

def export_prints(outdir='out', dest='.'):
    """the two deliverables, as single STL files"""
    import trimesh
    meshes = {n: trimesh.load(f'{outdir}/{n}.stl') for n in PRINT1 + PRINT2}
    L = print_layouts(meshes)
    names = {1: 'FPMS_Rover1_Print1_Chassis_Assembled.stl', 2: 'FPMS_Rover1_Print2_Caps_Decks_LiDAR.stl'}
    for k in (1, 2):
        m = trimesh.util.concatenate([mm for _, mm in L[k]])
        m.export(f'{dest}/{names[k]}'); print(names[k], m.extents.round(1), 'bed max', m.bounds[1].round(1))

if __name__ == '__main__':
    import sys
    print(json.dumps(summary(), indent=1))
    export_all()
    if '--prints' in sys.argv:
        export_prints()
