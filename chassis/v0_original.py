# FPMS Rover 1 chassis - CadQuery source (concept C, print-in-place hinge). Single self-contained file.
# Frame: X forward (rear axle at X=0), Y left, Z up, mm. Ground at Z=0.
# Print 1 = main tub + front shell printed ASSEMBLED with the hinge in place (no springs, no pins).
# Print 2 = lids + decks + LiDAR mount + dowels.
# Run:  pip install cadquery trimesh manifold3d   then   python fpms_rover1_cad.py
import cadquery as cq, math, os, trimesh
def bx(x0,x1,y0,y1,z0,z1):
    return cq.Workplane('XY').box(x1-x0,y1-y0,z1-z0).translate(((x0+x1)/2,(y0+y1)/2,(z0+z1)/2))
def rbox(x0,x1,y0,y1,z0,z1,r):
    w = bx(x0,x1,y0,y1,z0,z1)
    return w.edges('|Z').fillet(r) if r>0 else w
def cyl_y(x,z,r,y0,y1):
    return cq.Workplane('XZ').circle(r).extrude(-(y1-y0)).translate((x,y0,z)) if False else \
           cq.Workplane('XZ', origin=(0,0,0)).center(x,z).circle(r).extrude(-(y1-y0)).translate((0,y0,0))
def cyl_x(y,z,r,x0,x1):
    return cq.Workplane('YZ').center(y,z).circle(r).extrude(x1-x0).translate((x0,0,0))
def try_fillet(w, sel, r):
    try: return w.edges(sel).fillet(r)
    except Exception as e: print('  fillet skipped', sel, r, e.__class__.__name__); return w
def teardrop_y(x,z,r,y0,y1):
    pts=[(x - r*math.cos(math.radians(45)), z + r*math.sin(math.radians(45))),(x, z + r*math.sqrt(2)),(x + r*math.cos(math.radians(45)), z + r*math.sin(math.radians(45)))]
    c = cyl_y(x,z,r,y0,y1)
    tri = cq.Workplane('XZ').polyline([pts[0],pts[1],pts[2],(x,z)]).close().extrude(-(y1-y0)).translate((0,y0,0))
    return c.union(tri)
def dshape_y(x,z,r,flat,y0,y1):   # circle radius r, flat on top at z+flat
    return cyl_y(x,z,r,y0,y1).cut(bx(x-r-1,x+r+1,y0-1,y1+1,z+flat,z+r+1))


def dowel():
    return cq.Workplane('XY').circle(1.95).extrude(5.2).edges('>Z').chamfer(0.4)


XR, XH, ZH = 0.0, 48.5, 40.0; XF = XH + 45.0          # e = 45
ZGB = 47.0; BELLY, T = 26.0, 2.4; FLOOR = BELLY + T
W = 74.0; WI = W - T
TOP = 134.9; LID_TOP = TOP + T                           # shells' top rim, lids
DECK_U = 66.0; DECK_T = DECK_U + 2.4                     # removable decks over the motors
MAIN_X0, MAIN_FW = -40.0, XH - 16.0                      # main tub rear face, front wall outer face (32.5)
FR_RW, NOSE = XH + 16.0, 166.0
CH = 4.0                                                 # top-edge chamfers at the joint                           # front shell rear wall outer face (64.5), nose
DZ = 2.6                                                 # deck parts raised vs. concept layout

def motor_mount(w, xa, sides=(1, -1)):
    face = WI - 4.1                                      # motor face at y = 67.5, motor reaches y = 5.5
    for s in sides:
        w = w.union(bx(xa-21.5, xa+21.5, *sorted((s*face, s*WI)), FLOOR-0.5, DECK_U-0.1))
        w = w.union(bx(xa-16, xa+16, *sorted((s*48, s*63)), FLOOR-0.5, ZGB).cut(cyl_y(xa, ZGB, 18.7, -100, 100)))
        w = w.union(bx(xa-14, xa+14, *sorted((s*14, s*26)), FLOOR-0.5, ZGB).cut(cyl_y(xa, ZGB, 16.6, -100, 100)))
        for sx in (1, -1):
            fy = sorted((s*50, s*60))
            w = w.union(bx(*sorted((xa+sx*18.95, xa+sx*20.95)), *fy, FLOOR-0.5, 57)).union(bx(*sorted((xa+sx*17.2, xa+sx*20.95)), *fy, 55, 57))
        w = w.union(bx(xa-8, xa+8, *sorted((s*3.0, s*5.0)), FLOOR-0.5, 34))
        w = w.cut(cyl_y(xa, ZH, 6.2, *sorted((s*(face-2), s*(W+10)))))
    return w

def ledges(w, runs):
    """deck support ledges, each run sits against a real wall; top at DECK_U"""
    for (a, b, c, d) in runs:
        w = w.union(bx(a, b, c, d, DECK_U-3, DECK_U))
        w = w.union(cq.Workplane('XY').box(1, 1, 1).translate(((a+b)/2, (c+d)/2, DECK_U-3.5)).intersect(bx(a, b, c, d, DECK_U-4, DECK_U-3)) if False else w)
    return w

def frame(x0, x1, y0, y1, h, t=1.6, z0=FLOOR):
    return bx(x0-t, x1+t, y0-t, y1+t, z0-0.5, z0+h).cut(bx(x0, x1, y0, y1, z0-1, z0+h+1))

# ---------------- MAIN TUB ----------------
def main_tub():
    o = bx(MAIN_X0, MAIN_FW, -W, W, BELLY, TOP).edges('|Z').edges('>X').fillet(4).edges('|Z').edges('<X').fillet(12)
    o = try_fillet(o, '<Z', 3)
    tub = o.faces('>Z').shell(-T)
    tub = tub.cut(cq.Workplane('XZ').polyline([(MAIN_FW-CH, TOP+1), (MAIN_FW+1, TOP+1), (MAIN_FW+1, TOP-CH-1)]).close().extrude(-200).translate((0, -100, 0)))
    tub = tub.union(cq.Workplane('XZ').polyline([(MAIN_FW-CH, TOP), (MAIN_FW, TOP-CH), (MAIN_FW-T*1.414, TOP-CH), (MAIN_FW-CH-T*1.414+0.01, TOP)]).close().extrude(-2*WI).translate((0, -WI, 0)))
    tub = motor_mount(tub, XR)
    # hinge cups (round about the pin), 0-deg catch bars, 20-deg stop pads
    for s in (1, -1):
        cy = sorted((s*60, s*W))
        cup = cyl_y(XH, ZH, 12, *cy).union(bx(MAIN_FW-2, XH, *cy, BELLY, ZH+12)).union(bx(XH, XH+8.5, *cy, BELLY, ZH))
        tub = tub.union(cup.cut(dshape_y(XH, ZH, 5.25, 4.45, -100, 100)))
        ky = sorted((s*46, s*58))
        tub = tub.union(bx(MAIN_FW-2, XH-7.5, *ky, 50.3, 56))
        tub = tub.union(cq.Workplane('XZ').polyline([(MAIN_FW-2, 45.3), (MAIN_FW+3, 50.3), (MAIN_FW-2, 50.3)]).close().extrude(-(ky[1]-ky[0])).translate((0, ky[0], 0)))
        tub = tub.union(bx(MAIN_FW-0.5, MAIN_FW+0.25, *sorted((s*30, s*50)), 118, TOP-CH-0.2))
    # Pi fan vents in the front wall
    for i in range(11):
        vy = -66 + i*8
        tub = tub.cut(cq.Workplane('YZ').center(vy, 100).slot2D(54, 3.2, 90).extrude(10).translate((MAIN_FW-6, 0, 0)))
    # wiring holes to the front shell (below the deck)
    for yy in (-40, 40):
        tub = tub.cut(cyl_x(yy, 56, 7, MAIN_FW-6, MAIN_FW+2))
    # E-stop: open notch in the rear-left upper corner, panel 22 mm in, closed to the interior
    nx0, nx1, ny0, nz0, nz1 = MAIN_X0-1, -18.0, 36.5, 86.0, 129.0
    tub = tub.cut(bx(nx0, nx1, ny0, W+5, nz0, nz1))
    dz = nx1 - MAIN_X0                                                       # 22 mm: 45-deg wedge depth
    tub = tub.union(bx(nx1-T, nx1, ny0-T, W, nz0-T, nz1+T))                 # panel
    tub = tub.union(cq.Workplane('XZ').polyline([(MAIN_X0, nz0-dz), (nx1, nz0), (nx1, nz1+T), (MAIN_X0, nz1+T)]).close().extrude(-T).translate((0, ny0-T+T, 0)).translate((0, -T, 0)))
    tub = tub.union(cq.Workplane('XZ').polyline([(MAIN_X0, nz0-dz), (nx1, nz0), (MAIN_X0, nz0)]).close().extrude(-(W-(ny0-T))).translate((0, ny0-T, 0)))   # 45-deg floor wedge
    tub = tub.union(bx(MAIN_X0, nx1, ny0-T, W, nz1, nz1+T))                 # roof (bridges 22 mm)
    td = cq.Workplane('YZ').center(56.5, 107.5).circle(11.15).extrude(10).translate((nx1-6, 0, 0))
    tdt = cq.Workplane('YZ').polyline([(56.5-7.9, 107.5+7.9), (56.5, 107.5+15.8), (56.5+7.9, 107.5+7.9), (56.5, 107.5)]).close().extrude(10).translate((nx1-6, 0, 0))
    tub = tub.cut(td.union(tdt))
    # deck ledges (deck spans the interior over the rear motors)
    tub = ledges(tub, [(MAIN_X0+T, MAIN_X0+T+3, -WI, ny0-T), (MAIN_FW-T-3, MAIN_FW-T, -WI, ny0-T), (MAIN_X0+T, MAIN_FW-T, -WI, -WI+3)])
    # 5 V supply slot on the floor between the motors (on edge)
    tub = tub.union(frame(18.6, 28.5, -12.9, 12.9, 8))
    # drains
    for (dx, dy) in ((-30, 0), (25, 40), (25, -40)):
        tub = tub.cut(cq.Workplane('XY').center(dx, dy).circle(2).extrude(10).translate((0, 0, BELLY-1)))
    return tub

# ---------------- FRONT SHELL ----------------
def front_shell():
    o = bx(FR_RW, NOSE, -W, W, BELLY, TOP).edges('|Z').edges('>X').fillet(4).edges('|Z').edges('<X').fillet(4)
    o = try_fillet(o, '<Z', 3)
    fs = o.faces('>Z').shell(-T)
    # sloped top-rear edge: 45 deg, 8 mm, so the tall shell swings back clear of the main tub
    wedge = cq.Workplane('XZ').polyline([(FR_RW-1, TOP-CH-1), (FR_RW+CH+1, TOP+1), (FR_RW-1, TOP+1)]).close().extrude(-200).translate((0, -100, 0))
    fs = fs.cut(wedge)
    roof = cq.Workplane('XZ').polyline([(FR_RW, TOP-CH), (FR_RW+CH, TOP), (FR_RW+CH+T*1.414, TOP), (FR_RW+T*1.414, TOP-CH)]).close().extrude(-2*WI).translate((0, -WI, 0))
    fs = fs.union(roof)
    fs = motor_mount(fs, XF)
    for s in (1, -1):
        ky = sorted((s*46, s*58))
        k = cyl_y(XH, ZH, 10, *ky).union(bx(XH-10, XH, *ky, BELLY, ZH+10)).union(bx(XH, FR_RW+1, *ky, BELLY, ZH+10))
        fs = fs.union(k.cut(teardrop_y(XH, ZH, 5.3, -100, 100)))
        fs = fs.union(bx(XH+13, FR_RW+0.5, *sorted((s*3.7, s*25.7)), 51, 56.3).union(bx(XH+13, FR_RW+0.5, *sorted((s*25.7, s*27.3)), 51, 60)))     # tab shelf + side lip
    # deck ledges over the front motors (deck X FR_RW+2.4 .. 116.9)
    fs = ledges(fs, [(FR_RW+T, FR_RW+T+3, -WI, WI), (FR_RW+T, 117.0, -WI, -WI+3), (FR_RW+T, 117.0, WI-3, WI)])
    # deck supports from the floor: transverse rib ahead of the front motors + centre rib between them
    fs = fs.union(bx(115.0, 117.0, -WI, WI, FLOOR-0.5, DECK_U))
    fs = fs.union(bx(70.0, 115.5, -1.5, 1.5, FLOOR-0.5, DECK_U))
    for yy in (-40, 40):
        fs = fs.cut(cyl_x(yy, 50, 6, 114, 118))          # wire pass through the rib
    # water slab: two pumps + tank standing, frames + top fingers
    fs = fs.union(frame(123.2, NOSE-T, -WI+0.3, WI-0.3, 12))

    # VL53L0X: window in the nose above the tank + two 45-deg slotted rails inside
    fs = fs.cut(bx(NOSE-T-1, NOSE+1, 36, 61, 112, 124))
    for ry in (34.4, 61.6):
        rail = bx(NOSE-T-9, NOSE-T, ry-0.8, ry+0.8, 108.5, 128)
        slot = bx(-14, 14, -5, 5, -1, 1).rotate((0, 0, 0), (0, 1, 0), 45).translate((NOSE-T-4.5, ry, 118))
        fs = fs.union(rail.cut(slot))
    # servo S2 pod outside the right nose wall (ahead of the front wheel), servo standing, shaft up
    pod = bx(NOSE-24.4, NOSE+1.7, -W-14.0, -W+0.5, BELLY, 78.5).cut(bx(NOSE-23.2, NOSE+0.5, -W-12.8, -W+0.01, 50, 90))
    fs = fs.union(pod).cut(cyl_y(NOSE-10, 64, 3, -W-1, -WI+1))
    # wiring holes
    for yy in (-40, 40):
        fs = fs.cut(cyl_x(yy, 56, 7, FR_RW-2, FR_RW+6))
    for (dx, dy) in ((140, -50), (140, 0), (140, 50)):
        fs = fs.cut(cq.Workplane('XY').center(dx, dy).circle(2).extrude(10).translate((0, 0, BELLY-1)))
    return fs

# ---------------- DECKS (print 2) ----------------
def main_deck():
    x0, x1, y0, y1 = MAIN_X0+T+0.3, MAIN_FW-T-0.3, -WI+0.3, 36.5-T-0.3
    d = bx(x0, x1, y0, y1, DECK_U, DECK_T).edges('|Z').fillet(10)
    # Orange Pi in N515 standing on edge (35.8 thick in X, 92.8 along Y), USB end toward +Y
    pxa, pxb, pya, pyb = -14.7, 21.1, -70.8, 22.0
    for xx in (pxa-1.4, pxb):
        d = d.union(bx(xx, xx+1.4, pya, pyb, DECK_T, DECK_T+10))
    # Yahboom PCB standing in a slot near the rear wall (components face +X)
    d = d.union(bx(-37.2, -32.2, -67.5, 17.5, DECK_T, DECK_T+8).cut(bx(-35.95, -34.05, -69, 19, DECK_T+1, DECK_T+9)))
    return d

def front_deck():
    x0, x1, y0, y1 = FR_RW+T+0.3, 122.7, -WI+0.3, WI-0.3
    d = bx(x0, x1, y0, y1, DECK_U, DECK_T).edges('|Z').fillet(3)
    z0, h = DECK_T, 10
    w = lambda a, b, c, e, hh=h: bx(a, b, c, e, z0-0.1, z0+hh)
    # battery 50 (X) x 75 (Y) x 65 standing at X 72.4..122.4: rear wall + two side walls (front open to the deck edge)
    d = d.union(w(70.8, 72.4, -39.1, 39.1)).union(w(70.8, x1, 37.5, 39.1)).union(w(70.8, x1, -39.1, -37.5))
    # 9 V cells (stacked) X 69..117.5, Y 39.4..56.9; XY-MOS (stacked, on edge) X 69..103, Y 58.5..70.5 (side wall is the outer stop)
    d = d.union(w(67.4, 69.0, 39.1, 70.9, 8)).union(w(67.4, 117.5, 56.9, 58.5, 8)).union(w(117.5, 119.1, 39.1, 58.5, 8))
    # ESP32 standing X 74..87, Y -64.8..-39.4
    d = d.union(w(72.4, 74.0, -66.4, -39.1)).union(w(87.0, 88.6, -66.4, -39.1)).union(w(72.4, 88.6, -66.4, -64.8))
    # wire slots
    d = d.cut(bx(90, 118, -62, -42, DECK_U-1, DECK_T+1)).cut(bx(119.6, x1-1.5, 42, 66, DECK_U-1, DECK_T+1))
    return d

# ---------------- LIDS (print 2) ----------------
def lid_main():
    l = rbox(MAIN_X0, MAIN_FW, -W, W, TOP, LID_TOP, 0)
    l = bx(MAIN_X0, MAIN_FW-CH, -W, W, TOP, LID_TOP).edges('|Z').edges('>X').fillet(2).edges('|Z').edges('<X').fillet(12)
    l = try_fillet(l, '>Z', 1.5)
    lo = bx(MAIN_X0+T+0.25, MAIN_FW-CH-T*1.414-0.25, -WI+0.25, WI-0.25, TOP-5, TOP+0.1).edges('|Z').fillet(9.5)
    li = bx(MAIN_X0+T+1.85, MAIN_FW-CH-T*1.414-1.85, -WI+1.85, WI-1.85, TOP-6, TOP+1).edges('|Z').fillet(7.9)
    lip = lo.cut(li).cut(bx(MAIN_X0, -15, 33, W, 80, 140)).cut(bx(-17, 22, -W, -55, 80, 140))   # clear the E-stop notch and the Pi
    l = l.union(lip)
    cx = 3.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            l = l.cut(cq.Workplane('XY').center(cx+sx*19, sy*19).circle(2.1).extrude(10).translate((0, 0, TOP-2)))
    l = l.cut(cq.Workplane('XY').center(cx, 31).rect(22, 10).extrude(10).translate((0, 0, TOP-2)))
    return l

def lid_front():
    x0 = FR_RW + CH + T*1.414
    l = bx(x0, NOSE, -W, W, TOP, LID_TOP).edges('|Z').edges('>X').fillet(12)
    l = try_fillet(l, '>Z', 1.5)
    lo = bx(x0+0.25, NOSE-T-0.25, -WI+0.25, WI-0.25, TOP-5, TOP+0.1)
    li = bx(x0+1.85, NOSE-T-1.85, -WI+1.85, WI-1.85, TOP-6, TOP+1)
    return l.union(lo.cut(li).cut(bx(x0-1, 125, -40, 40, TOP-7, TOP+1)))   # lip notched over the battery

def lidar_mount():
    cx, cy = 3.0, 0
    m = rbox(cx-30, cx+30, cy-26.15, cy+26.15, LID_TOP, LID_TOP+3, 4)
    for sx in (-1, 1):
        for sy in (-1, 1):
            tx = cx + sx*(27.3+0.9); ty = cy + sy*15.45
            m = m.union(bx(tx-0.9, tx+0.9, ty-5, ty+5, LID_TOP, LID_TOP+9)).union(bx(tx-0.9, tx+0.9, ty-5, ty+5, LID_TOP+7.8, LID_TOP+9).translate((-sx*0.9, 0, 0)))
            m = m.cut(cq.Workplane('XY').center(cx+sx*19, cy+sy*19).circle(2.1).extrude(10).translate((0, 0, LID_TOP-2)))
    return m

def pin():   # D-profile pin along +Y: head outside the cup (y 74..77), inner end y 1
    p = cyl_y(0, 0, 5.0, 1, W).union(cyl_y(0, 0, 7.0, W, W+3))
    p = p.cut(bx(-10, 10, -1, 100, 4.2, 10))
    return p.cut(cyl_y(0, 0, 10, 2.0, 3.6).cut(cyl_y(0, 0, 4.0, 0, 10)))

def spring_c(turns=2.34, t=1.4, b=22.0, pitch=2.6, r_hub=7.0, theta_end_deg=8.0, hub_len=41.8):
    """printed spiral; strip overlaps the hub by 1 mm; hub tube runs on to the knuckle as a spacer.
    Print frame: spiral flat on the bed (XY), hub tube rises in +Z. Installed (right side): X->X, Y->Z, Z->-y."""
    r0 = r_hub + t/2 - 1.0
    th_total = turns*2*math.pi; th_start = math.radians(theta_end_deg) - th_total
    N = 400; inner, outer = [], []
    for i in range(N+1):
        th = th_start + th_total*i/N; r = r0 + pitch*(th - th_start)/(2*math.pi)
        inner.append(((r - t/2)*math.cos(th), (r - t/2)*math.sin(th))); outer.append(((r + t/2)*math.cos(th), (r + t/2)*math.sin(th)))
    strip = cq.Workplane('XY').polyline(outer + inner[::-1]).close().extrude(b)
    hub = cq.Workplane('XY').circle(r_hub).extrude(hub_len)
    hole = cq.Workplane('XY').circle(5.3).extrude(hub_len).cut(bx(-10, 10, 4.45, 10, -1, hub_len+1))
    r1 = r0 + pitch*turns
    tab = bx(r1-0.2, 26.0, -t/2, t/2, 0, b).rotate((0, 0, 0), (0, 0, 1), theta_end_deg)
    return strip.union(hub).union(tab).cut(hole)
def spring_R(): return spring_c()
def spring_L(): return spring_c().mirror('XZ')
def coupon_hinge():
    out = None
    for i, (dc, dk) in enumerate(((10.4, 10.4), (10.5, 10.6), (10.6, 10.8))):
        blk = bx(0, 24, 0, 12, 0, 24).translate((i*30, 0, 0))
        cupb = dshape_y(i*30+12, 12, dc/2, dc/2-0.8, -1, 13)
        kn = bx(0, 24, 14, 26, 0, 24).translate((i*30, 0, 0)).cut(teardrop_y(i*30+12, 12, dk/2, 13, 27))
        part = blk.cut(cupb).union(kn)
        out = part if out is None else out.union(part)
    return out
def coupon_motor():
    w = bx(-24, 24, 44, W, BELLY, FLOOR).union(bx(-24, 24, WI, W, BELLY, 72))
    w = motor_mount(w, 0, sides=(1,))
    return w.cut(cyl_y(0, ZH, 6.2, 60, W+10)).intersect(bx(-30, 30, 40, 80, 0, 100))


# ---------------- PRINT-IN-PLACE HINGE ----------------
GAP = 0.4
def main_pip():
    t = main_tub()
    for s in (1, -1):
        ky = sorted((s*46, s*58))
        t = t.cut(bx(MAIN_FW-2.5, XH-7.4, *ky, 44.5, 56.2))                  # remove the old over-the-top catch + gusset
        t = t.union(cyl_y(XH, ZH, 5.0, *sorted((s*32, s*74))))              # pin, fused, runs cup-to-cup through the knuckle
        icy = sorted((s*32, s*44))                                           # inner cup so the pin is held at both ends
        t = t.union(cyl_y(XH, ZH, 12, *icy).union(bx(MAIN_FW-2, XH, *icy, BELLY, ZH+12)).union(bx(XH, XH+8.5, *icy, BELLY, ZH)))
        t = t.union(bx(MAIN_FW-2, XH-10-GAP, *ky, 28, 34))                  # 0-deg catch: vertical face 0.4 mm behind the knuckle
    return t
def front_pip():
    f = front_shell()
    for s in (1, -1):
        f = f.cut(bx(XH+12.9, FR_RW-0.01, *sorted((s*3.5, s*27.5)), 50.9, 60.2))   # spring shelves not needed
        f = f.cut(teardrop_y(XH, ZH, 5.0+GAP, -100, 100))                          # bore: 0.4 mm all round the pin
    return f
# ---------------- EXPORT ----------------
def to_mesh(s, name):
    os.makedirs('out', exist_ok=True)
    cq.exporters.export(s, f'out/{name}.stl', tolerance=0.05, angularTolerance=0.15)
    cq.exporters.export(s, f'out/{name}.step')
    return trimesh.load(f'out/{name}.stl')

if __name__ == '__main__':
    BED = 256.0
    m = to_mesh(main_pip(), 'main_tub_pip'); f = to_mesh(front_pip(), 'front_shell_pip')
    p1 = trimesh.util.concatenate([m, f])                       # printed upright, as assembled, at 0 deg
    lo, hi = p1.bounds; p1.apply_translation([BED/2-(lo[0]+hi[0])/2, BED/2-(lo[1]+hi[1])/2, -lo[2]])
    p1.export('FPMS_Rover1_Print1_Chassis_Assembled.stl'); print('print 1', p1.extents.round(1))
    R = trimesh.transformations.rotation_matrix
    parts = [('lid_main', lid_main(), True), ('lid_front', lid_front(), True), ('front_deck', front_deck(), False),
             ('main_deck', main_deck(), 'rot'), ('lidar_mount', lidar_mount(), False)] + [(f'dowel{i}', dowel(), False) for i in range(4)]
    x, y, rowh, items = 5.0, 5.0, 0.0, []
    for name, s, flip in parts:
        q = to_mesh(s, name)
        if flip == 'rot': q.apply_transform(R(math.pi/2, [0, 0, 1]))
        elif flip: q.apply_transform(R(math.pi, [1, 0, 0]))          # lids print top-down
        q.apply_translation(-q.bounds[0])
        if x + q.extents[0] > BED - 5: x, y, rowh = 5.0, y + rowh + 5, 0.0
        q.apply_translation([x, y, 0]); items.append(q); x += q.extents[0] + 5; rowh = max(rowh, q.extents[1])
    p2 = trimesh.util.concatenate(items); p2.export('FPMS_Rover1_Print2_Lids_Decks.stl')
    print('print 2 footprint', p2.bounds.round(1).tolist(), '(must fit 256 x 256)')
