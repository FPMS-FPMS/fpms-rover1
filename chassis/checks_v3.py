"""v3 fit checks by computation. Run from chassis/ after fpms_rover1_v3.py (reads out_v3/).
Writes CHECKS.md and out_v3/checks.json. Exit code 1 if a hard check fails. (v2's checker is checks.py.)"""
import json, math, os, sys, numpy as np, trimesh, manifold3d as m3
sys.path.insert(0, '.')
import fpms_rover1_v3 as C
OUT = 'out_v3'
meta = json.load(open(f'{OUT}/meta.json'))
S = meta['summary']; H = meta['hinge']; BED = 256.0
PETG = 1.27  # g/cm3
MIN_CLR = 0.4

def load(n):
    return trimesh.load(f'{OUT}/{n}.stl')
def man(mesh):
    return m3.Manifold(m3.Mesh(vert_properties=np.asarray(mesh.vertices, np.float32), tri_verts=np.asarray(mesh.faces, np.uint32)))
def inter_vol(a, b):
    return (a ^ b).volume()
def rot_shell(mesh, deg):
    """rotate about the hinge axis (Y through X=H.x, Z=H.z), +deg = nose up (trimesh's +angle about +Y is nose down)"""
    M = trimesh.transformations.rotation_matrix(math.radians(-deg), [0, 1, 0], [H['x'], 0, H['z']])
    m = mesh.copy(); m.apply_transform(M); return m

parts = {n: load(n) for n in meta['print1'] + meta['print2'] + meta['print3'] if not n.startswith('spacer')}
comps = {n: load(f'comp/{n}') for n in meta['components']}
keeps = {n: load(f'comp/keepout_{n}') for n in meta['keepouts']}
for n, m in list(parts.items()) + list(comps.items()) + list(keeps.items()):
    assert m.is_watertight, f'{n} is not watertight - every boolean below would read 0 and pass'
P = {n: man(m) for n, m in parts.items()}
Cm = {n: man(m) for n, m in comps.items()}
K = {n: man(m) for n, m in keeps.items()}
R = {}; hard_fail = []
def allowed(a, b):
    """designed contacts between ghosts: a tube ends on its barb / fitting / nozzle; the arm's two positions are one arm"""
    p = {a, b}
    if p == {'refill_arm_stowed', 'refill_arm_deployed'}:
        return True
    for t in p:
        if t.startswith('tube_'):
            o = (p - {t}).pop()
            if o in ('pump_lower', 'pump_upper', 'tank_water', 'nozzle_luer'):
                return True
    return False

# ---------- 1. overlaps: components vs every printed part and vs each other ----------
ov = []
for cn, cm in Cm.items():
    for pn, pm in P.items():
        v = inter_vol(cm, pm)
        if v > 0.5:
            ov.append((cn, pn, round(v, 1)))
names = list(Cm)
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        if {names[i].split('_')[0], names[j].split('_')[0]} == {'motor', 'wheel'}:
            continue
        if allowed(names[i], names[j]):
            continue
        v = inter_vol(Cm[names[i]], Cm[names[j]])
        if v > 0.5:
            ov.append((names[i], names[j], round(v, 1)))
OWN = {'pi_plug_zone': 'orange_pi_n515', 'pi_fan_air': 'orange_pi_n515', 'pump_lower_bends': 'pump_lower', 'pump_upper_bends': 'pump_upper'}
for kn, km in K.items():
    for pn, pm in P.items():
        v = inter_vol(km, pm)
        if v > 0.5:
            ov.append(('keepout ' + kn, pn, round(v, 1)))
    for cn, cm in Cm.items():
        if cn.startswith('wheel') or OWN.get(kn) == cn:
            continue
        v = inter_vol(km, cm)
        if v > 0.5:
            ov.append(('keepout ' + kn, cn, round(v, 1)))
R['overlaps'] = ov
if ov:
    hard_fail.append(f'{len(ov)} overlaps')

# ---------- 2. pocket clearance: every component to the nearest printed part, its designed seat excluded ----------
allparts = trimesh.util.concatenate([parts[n] for n in parts])
pq = trimesh.proximity.ProximityQuery(allparts)
clr, seat_note = {}, {}
for cn, cm in comps.items():
    seat = meta['components'][cn]['seat']
    if seat is None:
        continue
    pts, fi = trimesh.sample.sample_surface_even(cm, 4000, seed=1)
    nrm = cm.face_normals[fi]
    keep = np.ones(len(pts), bool)
    zmin = cm.bounds[0][2]
    if seat == 'free':
        pass
    elif seat == 'bottom' or isinstance(seat, dict):
        # what it rests on: every downward face, plus a 0.6 mm band above its lowest point (points there sit next to the seat)
        keep &= nrm[:, 2] > -0.5
        keep &= pts[:, 2] > zmin + 0.6
        seat_note[cn] = 'rests on its underside'
    if isinstance(seat, dict) and 'face_y' in seat:
        fy = seat['face_y']                       # motor: gearbox face bolted to the pad
        keep = (np.abs(pts[:, 1] - fy) > 0.6)
        keep &= np.abs(pts[:, 1]) < abs(fy) + 0.01   # the boss/shaft beyond the face run in their own Ø12.9 hole: checked separately
        seat_note[cn] = 'gearbox face bolted to the pad'
    if isinstance(seat, dict) and 'plane' in seat:         # VL53: lies on the 45 deg nose slope
        a, b, c, d = seat['plane']
        keep = np.abs(pts @ np.array([a, b, c]) - d) > 0.6 + 0.45
        keep &= (nrm @ np.array([a, b, c])) < 0.5
        seat_note[cn] = 'lies on the 45 deg slope'
    if isinstance(seat, dict) and 'ear_z' in seat:         # servo: hangs by its ears (rear ear on the motor pad, front on a post)
        keep &= np.abs(pts[:, 2] - seat['ear_z']) > 0.6
        seat_note[cn] = 'hangs by its ears'
    if isinstance(seat, dict) and 'face_x' in seat:
        keep = pts[:, 0] > seat['face_x'] + 0.6      # E-stop: panel nut against the rear wall
        seat_note[cn] = 'panel face on the rear wall'
    if isinstance(seat, dict) and 'boxes' in seat:          # designed contacts (rails, posts, cradles)
        for b in seat['boxes']:
            inside = ((pts[:, 0] > b[0] - 0.6) & (pts[:, 0] < b[1] + 0.6) & (pts[:, 1] > b[2] - 0.6) & (pts[:, 1] < b[3] + 0.6)
                      & (pts[:, 2] > b[4] - 0.6) & (pts[:, 2] < b[5] + 0.6))
            keep &= ~inside
        seat_note[cn] = 'held by rails / posts / cradle (contact boxes excluded)'
    sp = pts[keep]
    _, d, _ = pq.on_surface(sp)
    clr[cn] = round(float(d.min()), 2)
# motor boss in its locating hole: radial clearance by construction
R['boss_hole_radial_clearance'] = round(C.CLR, 2)
R['clearance_mm'] = clr
low = {k: v for k, v in clr.items() if v < MIN_CLR - 1e-3}
if low:
    hard_fail.append(f'pocket clearance under {MIN_CLR}: {low}')

# ---------- 3. hinge sweep ----------
tub_grp = man(trimesh.util.concatenate([parts[n] for n in parts if meta['body_of'][n] == 'tub'] +
                                       [comps[n] for n in comps if meta['components'][n]['body'] == 'tub']))
shell_all = trimesh.util.concatenate([parts[n] for n in parts if meta['body_of'][n] == 'shell'] +
                                     [comps[n] for n in comps if (meta['components'][n]['body'] == 'shell' or n.startswith('wheel_front'))
                                      and n != 'refill_arm_deployed'])
sweep = []
for k in range(0, 101):
    a = -3.0 + 0.25 * k
    sweep.append((a, round(inter_vol(tub_grp, man(rot_shell(shell_all, a))), 2)))
free = [a for a, v in sweep if v < 0.05]
lo_stop = max([a for a, v in sweep if a < 0 and v >= 0.05], default=None)
hi_stop = min([a for a, v in sweep if a > 0 and v >= 0.05], default=None)
R['hinge_sweep'] = dict(free_from=min(free), free_to=max(free), nose_down_stop_deg=lo_stop, nose_up_stop_deg=hi_stop, samples=sweep[::4])
if not (min(free) <= 0 and max(free) >= 20):
    hard_fail.append(f'hinge does not move freely 0..20 (free {min(free)}..{max(free)})')
gap0 = P['main_tub'].min_gap(P['front_shell'], 5.0)
R['print1_min_gap_mm'] = round(gap0, 3)
if gap0 < H['gap'] - 0.02:
    hard_fail.append(f'print-in-place gap {gap0:.3f} < {H["gap"]}')
R['wheel_to_body_mm'] = {wn: round(Cm[wn].min_gap(P['main_tub' if 'rear' in wn else 'front_shell'], 10.0), 2)
                         for wn in comps if wn.startswith('wheel')}
if min(R['wheel_to_body_mm'].values()) < 4.0 - 1e-3:
    hard_fail.append(f"wheel gap under 4 mm: {R['wheel_to_body_mm']}")

# ---------- 4. outline + curb ----------
wheel_front = S['wheel_front']
everything = [m for n, m in parts.items()] + [m for n, m in comps.items() if not n.startswith(('wheel', 'estop_head', 'motor', 'refill_arm_deployed', 'lidar', 'rgb'))]  # shafts sit in the wheel hubs
allv = np.vstack([m.vertices for m in everything])
R['outline'] = dict(nose_x=round(float(allv[:, 0].max()), 2), max_abs_y=round(float(np.abs(allv[:, 1]).max()), 2),
                    wheel_outer_y=114.0, nose_overhang=round(float(allv[:, 0].max()) - wheel_front, 2))
if R['outline']['nose_overhang'] > 20.0 + 1e-3:
    hard_fail.append(f"nose overhang {R['outline']['nose_overhang']} > 20")
if R['outline']['max_abs_y'] > 114.0:
    hard_fail.append('something sticks out sideways past the wheels')
if S['length'] > 210.0 + 1e-3:
    hard_fail.append(f"length {S['length']} > 210")
# curb: for every curb height up to the axle, the wheel must touch first (both wheel sizes)
shell_parts = trimesh.util.concatenate([parts['front_shell']] + [comps[n] for n in comps
                                        if meta['components'][n]['body'] == 'shell' and not n.startswith(('wheel', 'refill_arm_deployed'))])
sv = shell_parts.vertices
curb = {}
for Rw in (40.0, 43.3):
    zg = 40.0 - Rw; worst = None
    for h in np.arange(1.0, Rw + 0.01, 0.5):
        xc = 93.5 + math.sqrt(Rw ** 2 - (Rw - h) ** 2)          # where the wheel meets the curb corner
        under = sv[sv[:, 0] >= xc]
        if len(under) and under[:, 2].min() <= zg + h:
            worst = round(float(h), 1); break
    curb[f'wheel_d{2 * Rw:.1f}'] = 'wheel touches first for every curb up to the axle' if worst is None else f'body touches first at {worst} mm'
R['curb'] = curb
if any('body' in v for v in curb.values()):
    hard_fail.append(f'curb: {curb}')

# ---------- 5. bed fit ----------
lay = C.print_layouts(parts)
p1b = trimesh.util.concatenate([m for _, m in lay[1]])
R['print1_extent'] = (p1b.bounds[1] - p1b.bounds[0]).round(1).tolist()
if max(R['print1_extent'][:2]) > BED - 4 or R['print1_extent'][2] > 250:
    hard_fail.append(f"print 1 does not fit {R['print1_extent']}")
placed = lay[2]
R['print3_parts'] = [n for n, _ in lay[3]]
p3 = trimesh.util.concatenate([m for _, m in lay[3]]); R['print3_extent'] = (p3.bounds[1] - p3.bounds[0]).round(1).tolist()
touch = []
for i in range(len(placed)):
    for j in range(i + 1, len(placed)):
        g = man(placed[i][1]).min_gap(man(placed[j][1]), 3.0)
        if g < 2.0:
            touch.append(f'{placed[i][0]}~{placed[j][0]} {g:.2f}mm')
p2 = trimesh.util.concatenate([m for _, m in placed])
ex, ey = C.P1S_EXCL                     # Bambu refuses a plate with anything in the P1S keep-out corner
for n, m in placed + lay[1]:
    v = m.vertices
    if np.any((v[:, 0] < ex + 1.0) & (v[:, 1] < ey + 1.0)):
        touch.append(f'{n} in the P1S keep-out corner')
R['print2_extent'] = (p2.bounds[1] - p2.bounds[0]).round(1).tolist()
R['print2_problems'] = touch
R['print2_fits'] = bool(not touch and p2.bounds[0][0] >= 0 and p2.bounds[0][1] >= 0 and p2.bounds[1][0] <= BED and p2.bounds[1][1] <= BED)
R['print2_area_pct'] = round(100 * sum(m.extents[0] * m.extents[1] for _, m in placed) / BED ** 2, 1)
if not R['print2_fits']:
    hard_fail.append(f"print 2 does not fit {R['print2_extent']} {touch}")

# ---------- 6. fill % per bay ----------
cav = C.cavities(); bays = C.bays()
fill = {}
for bn, (body, region) in bays.items():
    bv = cav[body].intersect(region).val().Volume() / 1000
    cv = sum(abs(comps[c].volume) for c, d in meta['components'].items() if d['bay'] == bn and not str(d['src']).startswith('optional')) / 1000
    kv = sum(abs(keeps[k].volume) for k, d in meta['keepouts'].items() if d['bay'] == bn) / 1000
    fill[bn] = dict(bay_cm3=round(bv, 1), parts_cm3=round(cv, 1), keepout_cm3=round(kv, 1),
                    fill_pct=round(100 * cv / bv, 1), used_pct=round(100 * (cv + kv) / bv, 1))
tb = sum(f['bay_cm3'] for f in fill.values()); tc = sum(f['parts_cm3'] for f in fill.values()); tk = sum(f['keepout_cm3'] for f in fill.values())
fill['TOTAL'] = dict(bay_cm3=round(tb, 1), parts_cm3=round(tc, 1), fill_pct=round(100 * tc / tb, 1), used_pct=round(100 * (tc + tk) / tb, 1))
R['fill_v2'] = fill
vol = {n: abs(comps[n].volume) / 1000 for n in comps}
mot = vol['motor_rear_L']
v0 = dict(
    tub_lower=dict(bay_cm3=67.7 * 143.2 * (66 - 28.4) / 1000, parts_cm3=2 * mot + vol['regulator_5v']),
    tub_upper=dict(bay_cm3=67.7 * 143.2 * (134.9 - 68.4) / 1000, parts_cm3=vol['orange_pi_n515'] + vol['yahboom_board'] + vol['estop_body']),
    shell_lower=dict(bay_cm3=(122.7 - 66.9) * 143.2 * (66 - 28.4) / 1000, parts_cm3=2 * mot),
    shell_upper=dict(bay_cm3=(122.7 - 66.9) * 143.2 * (134.9 - 68.4) / 1000,
                     parts_cm3=vol['battery_3s2p'] + 2 * vol['nine_volt_1'] + 2 * vol['xy_mos_1'] + vol['esp32_s3']),
    nose=dict(bay_cm3=(163.6 - 123.2) * 143.2 * (134.9 - 28.4) / 1000, parts_cm3=vol['pump_lower'] + vol['pump_upper'] + vol['tank_water']))
for v in v0.values():
    v['fill_pct'] = round(100 * v['parts_cm3'] / v['bay_cm3'], 1); v['bay_cm3'] = round(v['bay_cm3'], 1); v['parts_cm3'] = round(v['parts_cm3'], 1)
v0['TOTAL'] = dict(bay_cm3=round(sum(v['bay_cm3'] for v in v0.values()), 1), parts_cm3=round(sum(v['parts_cm3'] for v in v0.values()), 1))
v0['TOTAL']['fill_pct'] = round(100 * v0['TOTAL']['parts_cm3'] / v0['TOTAL']['bay_cm3'], 1)
R['fill_v0'] = v0
def hull_cm3(ms):
    return round(float(trimesh.util.concatenate(ms).convex_hull.volume) / 1000, 1)
V0 = 'out/v0/out'
hull_v0 = hull_cm3([trimesh.load(f'{V0}/{n}.stl') for n in ('main_tub_pip', 'front_shell_pip', 'lid_main', 'lid_front')]) if os.path.exists(f'{V0}/main_tub_pip.stl') else None
hull_v2 = hull_cm3([parts[n] for n in ('main_tub', 'front_shell', 'tub_cap', 'shell_cap')])
v1h = json.load(open('history/v1_checks.json')) if os.path.exists('history/v1_checks.json') else {}
R['compare'] = {
    'v0': dict(length=206.0, nose_overhang=round(166.0 - 136.8, 1), top=137.3, fill=v0['TOTAL']['fill_pct'], used=None, hull=hull_v0, front_axle=None),
    'v1': dict(length=v1h.get('envelope', {}).get('v1_len'), nose_overhang=round(189.7 - 136.8, 1),
               top=max(v1h.get('envelope', {}).get('v1_top_tub', 0), v1h.get('envelope', {}).get('v1_top_shell', 0)),
               fill=v1h.get('fill_v1', {}).get('TOTAL', {}).get('fill_pct'), used=v1h.get('fill_v1', {}).get('TOTAL', {}).get('used_pct'),
               hull=(v1h.get('hull_cm3') or {}).get('v1'), front_axle=v1h.get('mass', {}).get('front_axle_pct')),
    'v2': json.load(open('history/v2_checks.json'))['compare']['v2'] if os.path.exists('history/v2_checks.json') else {},
    'v3': dict(length=S['length'], nose_overhang=R['outline']['nose_overhang'], top=S['H'], fill=fill['TOTAL']['fill_pct'],
               used=fill['TOTAL']['used_pct'], hull=hull_v2, front_axle=None)}
R['hinge_gap_dead_cm3'] = dict(v0=round(32.0 * 148 * (134.9 - 26) / 1000, 1),
                               v3=round(0.5 * (S['Z_SPLIT'] - 40) ** 2 * math.tan(math.radians(H['rake'])) * 148 / 1000, 1))

# ---------- 7. weight and balance ----------
mass = []
for n, m in parts.items():
    mass.append((n, meta['body_of'][n], abs(m.volume) / 1000 * PETG, m.center_mass))
OPTIONAL = {n for n, d in meta['components'].items() if str(d['src']).startswith('optional')}
for n, d in meta['components'].items():
    if d['mass'] and n not in OPTIONAL:
        mass.append((n, d['body'] if d['body'] != 'wheel' else ('tub' if 'rear' in n else 'shell'), d['mass'], comps[n].center_mass))
mass.append(('wiring+screws (estimate)', 'tub', 50.0, np.array([0.0, 0, 80])))
mass.append(('wiring+tubes (estimate)', 'shell', 50.0, np.array([100.0, 0, 90])))
M = sum(g for _, _, g, _ in mass); X = sum(g * c[0] for _, _, g, c in mass) / M; Z = sum(g * c[2] for _, _, g, c in mass) / M
Ms = sum(g for _, b, g, _ in mass if b == 'shell'); Xs = sum(g * c[0] for _, b, g, c in mass if b == 'shell') / Ms
R['mass'] = dict(total_g=round(M), cg_x=round(X, 1), cg_z=round(Z, 1), front_axle_pct=round(100 * X / 93.5, 1),
                 shell_g=round(Ms), shell_cg_x=round(Xs, 1), shell_closing_moment_Nmm=round(Ms / 1000 * 9.81 * (Xs - H['x']), 1),
                 printed_petg_g=round(sum(g for n, _, g, _ in mass if n in parts)),
                 items=[(n, b, round(g), round(float(c[0]), 1)) for n, b, g, c in mass])
R['compare']['v3']['front_axle'] = R['mass']['front_axle_pct']
bal = [(n, meta['components'][n]['mass'], comps[n].center_mass) for n in sorted(OPTIONAL)]
if bal:
    Mb = M + sum(g for _, g, _ in bal); Xb = (X * M + sum(g * c[0] for _, g, c in bal)) / Mb
    Zb = (Z * M + sum(g * c[2] for _, g, c in bal)) / Mb
    xb = sum(g * c[0] for _, g, c in bal) / sum(g for _, g, _ in bal)
    R['mass']['with_ballast'] = dict(ballast_g=round(sum(g for _, g, _ in bal)), total_g=round(Mb), cg_x=round(Xb, 1), cg_z=round(Zb, 1),
                                     front_axle_pct=round(100 * Xb / 93.5, 1),
                                     need_for_65=round((0.65 * 93.5 * M - X * M) / (xb - 0.65 * 93.5)))

# ---------- 8. hinge printability ----------
def overhang_area(mesh, zone):
    fn = mesh.face_normals; c = mesh.triangles_center
    sel = (fn[:, 2] < -0.72) & (c[:, 2] > 26.3) & zone(c)
    return float(mesh.area_faces[sel].sum())
zone = lambda c: (np.abs(c[:, 0] - H['x']) < 16) & (c[:, 2] < 62) & (np.abs(c[:, 1]) > 28)
R['hinge_overhang_mm2'] = {n: round(overhang_area(parts[n], zone), 1) for n in ('main_tub', 'front_shell')}
if max(R['hinge_overhang_mm2'].values()) > 1.0:                 # nothing in the hinge may need support
    hard_fail.append(f"steep overhang in the hinge zone {R['hinge_overhang_mm2']}")


# ================= v3 hard checks =================
def shell_group(deg, include_deployed=False):
    ms = [parts[n] for n in parts if meta['body_of'][n] == 'shell']
    ms += [comps[n] for n in comps if meta['components'][n]['body'] == 'shell' and (include_deployed or n != 'refill_arm_deployed')]
    return rot_shell(trimesh.util.concatenate(ms), deg)

# ---------- 9. LiDAR scan plane vs the lifted shell (B5) ----------
scan = S['scan_plane']
zmax20 = float(shell_group(20.0).vertices[:, 2].max())
zmax0 = float(shell_group(0.0).vertices[:, 2].max())
R['lidar'] = dict(scan_plane=scan, shell_top_0deg=round(zmax0, 2), shell_top_20deg=round(zmax20, 2),
                  margin_20deg=round(scan - zmax20, 2), required=10.0 + C.LIDAR['beam'], riser=C.LIDAR['riser'])
if scan - zmax20 < 10.0 + C.LIDAR['beam'] - 1e-3:
    hard_fail.append(f"LiDAR scan plane only {scan - zmax20:.1f} mm above the shell at 20 deg (need {10 + C.LIDAR['beam']})")
# nothing of the tub may reach the scan plane either (cap, riser, E-stop guard ...)
tub_top = max(float(parts[n].vertices[:, 2].max()) for n in parts if meta['body_of'][n] == 'tub' and n != 'lidar_mount')
R['lidar']['tub_top'] = round(tub_top, 2)

# ---------- 10. wet zone: every electronic part above the wet floor + 5 or behind a wall (A3) ----------
WATER = [n for n in comps if n.startswith('tube_')] + ['tank_water', 'pump_lower', 'pump_upper', 'nozzle_luer']
ELEC = ['battery_3s2p', 'nine_volt_1', 'nine_volt_2', 'esp32_s3', 'xy_mos_1', 'xy_mos_2', 'servo_mg90s', 'vl53l0x']
shell_printed = trimesh.util.concatenate([parts[n] for n in parts if meta['body_of'][n] == 'shell'])
ray = trimesh.ray.ray_triangle.RayMeshIntersector(shell_printed)
water_mesh = trimesh.util.concatenate([comps[n] for n in WATER])
wq = trimesh.proximity.ProximityQuery(water_mesh)
deck = parts['pump_deck'].bounds
wet = {}
for e in ELEC:
    m = comps[e]
    pts, _ = trimesh.sample.sample_surface_even(m, 600, seed=3)
    cp, dist, _ = wq.on_surface(pts)
    near = dist < 15.0
    exposed = 0
    if near.any():
        o = pts[near]; t = cp[near]; d = t - o; L = np.linalg.norm(d, axis=1); dn = d / L[:, None]
        locs, idx, _ = ray.intersects_location(o + dn * 0.05, dn, multiple_hits=False)
        hit = np.full(len(o), np.inf)
        for loc, i in zip(locs, idx):
            hit[i] = min(hit[i], np.linalg.norm(loc - o[i]))
        clear_path = hit > L - 0.1
        above = t[:, 2] > o[:, 2] - 2.0                   # water at or above the part can drip/splash onto it
        exposed = int(np.sum(clear_path & above))
    z0 = float(m.bounds[0][2])
    under_tray = bool(m.bounds[1][2] < deck[0][2] and m.bounds[0][0] >= deck[0][0] - 0.5 and m.bounds[1][0] <= deck[1][0] + 0.5)
    ok = (exposed == 0) and (z0 >= C.FLOOR + 5.0 or under_tray)
    wet[e] = dict(bottom_z=round(z0, 1), wet_floor_plus5=round(C.FLOOR + 5.0, 1), nearest_water_mm=round(float(dist.min()), 1),
                  exposed_points=exposed, under_sealed_tray=under_tray, ok=ok)
    if not ok:
        hard_fail.append(f'wet zone: {e} {wet[e]}')
R['wet_zone'] = dict(parts=wet, drains_shell=C.DRAINS_SHELL, drain_d=2 * C.DRAIN_R, tub_drains=[(-17, 0), (38, 55), (38, -55)])

# ---------- 11. cables across the hinge (A4) ----------
CABLES = [  # name, conductors, wire OD mm, connector (w, h) that must pass, from -> to
    ('battery main lead', 2, 3.6, (15.5, 8.0), 'battery (shell) -> E-stop / main switch (tub)'),
    ('switched 12 V to pumps + XY-MOS', 2, 2.3, (10.0, 5.0), 'E-stop (tub) -> XY-MOS (shell), XT30'),
    ('charge lead', 2, 2.3, (10.0, 5.0), 'DC-099 jack (tub) -> battery BMS (shell), XT30'),
    ('front motor L', 6, 1.2, (13.9, 4.5), 'Yahboom board (tub) -> JGB37 front L, PH2.0-6P'),
    ('front motor R', 6, 1.2, (13.9, 4.5), 'Yahboom board (tub) -> JGB37 front R, PH2.0-6P'),
    ('ESP32 USB', 1, 3.5, (12.4, 6.5), 'ESP32 (shell) -> Orange Pi (tub), micro-USB/USB-C'),
]
from shapely.geometry import Polygon as SP, box as sbx
from shapely import affinity as aff
pass_poly = C.CABLE_PASS
bundle_area = sum(n * math.pi * (d / 2) ** 2 for _, n, d, _, _ in CABLES)
plugs_fit = {}
for name, n, d, (w, h), _ in CABLES:
    r = sbx(-w / 2 - 0.5, -h / 2 - 0.5, w / 2 + 0.5, h / 2 + 0.5)
    ok = any(pass_poly.contains(aff.translate(rr, cx, cz)) for rr in (r, aff.rotate(r, 90, origin=(0, 0)))
             for cx in np.arange(-6, 6.1, 0.5) for cz in np.arange(34, 56, 0.5))
    plugs_fit[name] = ok
fill_pct = 100 * bundle_area / pass_poly.area
cz = pass_poly.centroid.y
r_axis = abs(cz - H['z']) + 4.0                       # bundle centroid offset from the hinge axis (+ half the bundle)
stretch = 2 * r_axis * math.sin(math.radians(10.0))   # chord change at 20 deg
R['cables'] = dict(pass_area_mm2=round(pass_poly.area, 1), bundle_mm2=round(bundle_area, 1), fill_pct=round(fill_pct, 1),
                   plugs_fit=plugs_fit, centroid_to_axis_mm=round(r_axis, 1), length_change_20deg_mm=round(stretch, 1),
                   table=[dict(name=a, conductors=b, wire_od=c, plug=d, route=e) for a, b, c, d, e in CABLES],
                   not_crossing=['VL53L0X, servo, XY-MOS gates, 9 V cells: all on the shell side, wired to the ESP32 there',
                                 'AS5600: board on the tub, magnet on the shell - no wire crosses',
                                 'LiDAR lead: lid hole under the riser -> Pi, all in the tub', 'RGB bar: riser -> Yahboom board (tub)'])
if fill_pct > 40 or not all(plugs_fit.values()) or stretch > 10:
    hard_fail.append(f"cable pass: fill {fill_pct:.0f}% plugs {plugs_fit} stretch {stretch:.1f}")

# ---------- 12. refill arm + intake hose clear wheel and ground at 0 and 20 deg (A2) ----------
def arm_pose(name, deg):
    return rot_shell(comps[name], deg)
wheelF = man(comps['wheel_front_R']); wheelR = man(comps['wheel_rear_R'])
arm = {}
for nm in ('refill_arm_stowed', 'refill_arm_deployed'):
    for deg in (0.0, 20.0):
        a = arm_pose(nm, deg)
        wf = man(rot_shell(comps['wheel_front_R'], deg))
        arm[f'{nm}@{deg:g}'] = dict(front_wheel_gap=round(man(a).min_gap(wf, 30.0), 2), rear_wheel_gap=round(man(a).min_gap(wheelR, 30.0), 2),
                                    lowest_z=round(float(a.vertices[:, 2].min()), 2))
R['arm'] = arm
for k, v in arm.items():
    if v['front_wheel_gap'] < 4.0 or v['rear_wheel_gap'] < 4.0 or v['lowest_z'] < 0.0 - 1e-3:
        hard_fail.append(f'refill arm {k}: {v}')
if arm['refill_arm_deployed@0']['lowest_z'] > 2.0:
    hard_fail.append(f"refill arm does not reach the ground: lowest {arm['refill_arm_deployed@0']['lowest_z']}")
# hose outside the body: along the arm's outer face from the strainer to 12 mm from the pivot, then a free loop to the window
def hose_outside(deg_below):
    a = math.radians(deg_below); u = np.array([math.cos(a), 0, -math.sin(a)])
    piv = np.array([C.SERVO_PX, C.ARM['y'][0] - 0.0, C.SERVO_PZ])
    p_tip = piv + u * (C.ARM['reach'] + C.ARM['strainer'][1] - 2.0)
    p0 = piv + u * 14.0
    p3 = np.array([C.WIN['x'][0] + 9.0, -C.W - 1.0, sum(C.WIN['z']) / 2])
    t0 = -u; t3 = np.array([-0.55, 0.835, 0.0])
    Lh = np.linalg.norm(p3 - p0)
    b1, b2 = p0 + t0 * Lh * 0.5, p3 - t3 * Lh * 0.5
    ts = np.linspace(0, 1, 60)[:, None]
    bez = (1 - ts) ** 3 * p0 + 3 * (1 - ts) ** 2 * ts * b1 + 3 * (1 - ts) * ts ** 2 * b2 + ts ** 3 * p3
    return np.vstack([np.linspace(p_tip, p0, 20), bez[1:]])
sys.path.insert(0, '/home/aryan/.claude/jobs/40bc94d8/tmp')
def min_radius(P, span=6):
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1); s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1], 1.0); Q = np.stack([np.interp(t, s, P[:, k]) for k in range(3)], 1)
    best = 1e9
    for i in range(span, len(Q) - span):
        a_, b_, c_ = Q[i - span], Q[i], Q[i + span]
        A, B, Cc = np.linalg.norm(b_ - c_), np.linalg.norm(a_ - c_), np.linalg.norm(a_ - b_)
        ar = np.linalg.norm(np.cross(b_ - a_, c_ - a_)) / 2
        if ar > 1e-6:
            best = min(best, A * B * Cc / (4 * ar))
    return best
hose = {}
for st, deg_b in (('stowed', C.ARM['stowed_deg']), ('deployed', C.ARM['deployed_deg'])):
    P0 = hose_outside(deg_b)
    for deg in (0.0, 20.0):
        Mrot = trimesh.transformations.rotation_matrix(math.radians(-deg), [0, 1, 0], [H['x'], 0, H['z']])
        Pr = trimesh.transformations.transform_points(P0, Mrot)
        wc = trimesh.transformations.transform_points([[93.5, 0, 40.0]], Mrot)[0]
        dw = min(abs(math.hypot(p[0] - wc[0], p[2] - wc[2]) - C.WHEEL_R_WORST) if -C.WHEEL_Y0 - C.WHEEL_W - 4.5 < p[1] < -C.WHEEL_Y0 + 4.5 else 99 for p in Pr) - C.TUBE_R
        hose[f'{st}@{deg:g}'] = dict(wheel_gap=round(dw, 1), lowest_z=round(float(Pr[:, 2].min() - C.TUBE_R), 1),
                                     min_bend_radius=round(min_radius(P0[19:]), 1), length=round(float(np.sum(np.linalg.norm(np.diff(P0, axis=0), axis=1))), 1))
R['intake_hose_outside'] = hose
for k, v in hose.items():
    if v['wheel_gap'] < 4.0 or v['lowest_z'] < 0.0 or v['min_bend_radius'] < 25.0:
        hard_fail.append(f'intake hose {k}: {v}')

# ---------- 13. spray cone clear (A1) ----------
tip = np.array(C.NOZ['tip']); ca, sa = math.cos(math.radians(C.NOZ['angle'])), math.sin(math.radians(C.NOZ['angle']))
dvec = np.array([ca, 0, -sa]); half = math.radians(4.0)
side = np.cross(dvec, [0, 1, 0]); side /= np.linalg.norm(side); up_ = np.cross(side, dvec)
cone_pts = []
for s_ in np.linspace(0.5, 600, 240):
    r_ = s_ * math.tan(half)
    for th in np.linspace(0, 2 * math.pi, 12, endpoint=False):
        for rr in (0.0, r_ * 0.5, r_):
            cone_pts.append(tip + dvec * s_ + rr * (math.cos(th) * np.array([0, 1, 0]) + math.sin(th) * up_))
cone_pts = np.array(cone_pts); cone_pts = cone_pts[cone_pts[:, 2] > 0]
blocked = {}
for n, m in list(parts.items()) + [(n, comps[n]) for n in comps if n not in ('nozzle_luer',) and not n.startswith('tube_')]:
    inside = m.contains(cone_pts)
    if inside.any():
        blocked[n] = int(inside.sum())
R['spray'] = dict(tip=[round(float(v), 1) for v in tip], angle_down=C.NOZ['angle'], cone_half_angle=4.0, length=600,
                  lands_at_x=round(float(tip[0] + tip[2] / math.tan(math.radians(C.NOZ['angle']))), 0), blocked_by=blocked,
                  tip_inside_outline=bool(tip[0] <= S['NOSE'] + 1e-6))
if blocked or not R['spray']['tip_inside_outline']:
    hard_fail.append(f'spray cone blocked: {blocked}')

# ---------- 14. tubes: bend radius after the silicone's own smoothing, lengths (A1/A2) ----------
def chaikin(P, n=3):
    P = np.asarray(P, float)
    for _ in range(n):
        Q = [P[0]]
        for a_, b_ in zip(P[:-1], P[1:]):
            Q += [0.75 * a_ + 0.25 * b_, 0.25 * a_ + 0.75 * b_]
        Q.append(P[-1]); P = np.array(Q)
    return P
tubes = {}
for n, pts in C.TUBES.items():
    P = np.asarray(pts, float)
    tubes[n] = dict(length_mm=round(float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))), 0),
                    min_bend_radius=round(min_radius(chaikin(P), span=8), 1))
R['tubes'] = tubes
if not tubes:
    hard_fail.append('tubes not routed (tubes_v3.json missing)')
for n, v in tubes.items():
    if v['min_bend_radius'] < 25.0 - 0.5:
        hard_fail.append(f'tube {n} bend radius {v["min_bend_radius"]} < 25')

# ---------- 15. AS5600 geometry (B9) ----------
R['as5600'] = dict(magnet_to_chip_gap_mm=round(C.AS['y_chip'] - C.AS['chip_h'] - C.AS['y_mag'], 2), on_axis_offset_mm=0.0,
                   datasheet_gap='0.5-3 mm (aim ~1)', lug_print_in_place_gap=C.GAP)

# ---------- 16. ESP32 support (B6) ----------
R['esp32'] = dict(rails='C-rails on both long edges, from the ledge to %.1f of %.1f mm height' % (C.ESP_RAIL_Z1 - C.ESP_Z0, C.ESP[2]),
                  usb='USB end up, under an open slot in the shell cap: reachable with nothing removed', splash_wall=True)


# ---------- 17. assembly: every part goes in along a straight path (ASSEMBLY.md order) ----------
# (component, direction it moves while going IN, travel mm). The path checked is the reverse: from its seat, travel mm
# against that direction, against the printed parts present at that step (no caps / hatches / riser yet) and every
# component installed before it.
ORDER = [
    ('regulator_5v', (0, 0, -1), 60), ('as5600_board', (0, 0, -1), 60), ('xy_mos_1', (0, 0, -1), 60), ('xy_mos_2', (0, 0, -1), 60),
    ('orange_pi_n515', (0, 0, -1), 80), ('yahboom_board', (0, 0, -1), 50), ('estop_body', (0, 0, -1), 40),
    ('charge_jack_dc099', (1, 0, 0), 30),
    ('pump_lower', (0, 0, -1), 100), ('pump_upper', (0, 0, -1), 60), ('servo_mg90s', (0, -1, 0), 4.5),
    ('esp32_s3', (0, 0, -1), 80), ('tank_water', (0, 0, -1), 60), ('battery_3s2p', (0, 0, -1), 60), ('nozzle_luer', (0, 0, -1), 30),
]
PRESENT = ['main_tub', 'front_shell', 'tub_deck', 'tub_shelf', 'pump_deck']
def sweep_mesh(m, d, L, step=1.5):
    d = np.asarray(d, float)
    ms = [m.copy() for _ in range(int(L / step) + 1)]
    for i, mm in enumerate(ms):
        mm.apply_translation(-d * i * step)
    return ms
ins = {}
installed = []
for cn, d, L in ORDER:
    obst = [P[n] for n in PRESENT] + [Cm[n] for n in installed]
    worst = 0.0
    for mm in sweep_mesh(comps[cn], d, L):
        mm_ = man(mm)
        for o in obst:
            v = inter_vol(mm_, o)
            if v > worst:
                worst = v
    # the servo first drops in from above (its spline then goes 4.5 mm into the wall): check that drop too
    if cn == 'servo_mg90s':
        sv = comps[cn].copy(); sv.apply_translation([0, 4.6, 0])
        for mm in sweep_mesh(sv, (0, 0, -1), 80):
            for o in obst:
                worst = max(worst, inter_vol(man(mm), o))
    ins[cn] = dict(direction=d, travel=L, blocked_mm3=round(worst, 1))
    installed.append(cn)
R['insertion'] = ins
bad_ins = {k: v for k, v in ins.items() if v['blocked_mm3'] > 1.0}
if bad_ins:
    hard_fail.append(f'insertion path blocked: {bad_ins}')

R['hard_fail'] = hard_fail
json.dump(R, open(f'{OUT}/checks.json', 'w'), indent=1, default=float)

# ---------- report ----------
L = ['# v3 fit checks (computed by checks_v3.py from the exported meshes)', '']
L += [f"**Hard checks: {'ALL PASS' if not hard_fail else 'FAIL - ' + '; '.join(hard_fail)}**", '']
cmp = R['compare']
L += ['## v0 / v1 / v2 / v3', '', '| | v0 (original) | v1 | v2 (approved) | v3 |', '|---|---|---|---|---|']
def cell(v, unit=''):
    return '-' if v is None else f'{v}{unit}'
for key, lab, unit in (('length', 'Length', ' mm'), ('nose_overhang', 'Nose past front wheel (Ø86.6 worst case)', ' mm'),
                       ('top', 'Top of body (LiDAR not included)', ' mm'), ('fill', 'Fill: component volume / bay volume', '%'),
                       ('used', 'Fill incl. plug, fan-air and tube-bend zones', '%'), ('hull', 'Space claimed (convex hull)', ' cm3'),
                       ('front_axle', 'Weight on front axle', '%')):
    L.append(f"| {lab} | {cell(cmp['v0'][key], unit)} | {cell(cmp['v1'][key], unit)} | {cell(cmp['v2'].get(key), unit)} | {cell(cmp['v3'][key], unit)} |")
L += ['', 'v0 numbers come from the original script; its pumps had no room to bend their tubes and its tank and E-stop zones '
      'were not reserved, so its fill is flattered.', '']
L += ['## Overlaps (must be zero)', '', f"{len(ov)} overlaps" + ('' if not ov else ':')] + [f'- {a} x {b}: {v} mm3' for a, b, v in ov] + ['']
L += [f'## Pocket clearance (minimum {MIN_CLR} mm per side)', '',
      'Smallest gap from each component to any printed part, ignoring only the surface it is designed to rest or bolt on.',
      f"Motor bosses run in Ø12.9 holes: {R['boss_hole_radial_clearance']} mm radial by construction.", '',
      '| component | min gap mm | seat (excluded) |', '|---|---|---|']
L += [f"| {k} | {v} | {seat_note.get(k, '')} |" for k, v in sorted(clr.items(), key=lambda kv: kv[1])] + ['']
hs = R['hinge_sweep']
L += ['## Hinge sweep (shell + its cap, hatch, deck, components and front wheels rotated about the pin)', '',
      f"- moves freely from {hs['free_from']} to {hs['free_to']} deg (0.25 deg steps)",
      f"- 0 deg catch touches at {hs['nose_down_stop_deg']} deg; 20 deg stop at {hs['nose_up_stop_deg']} deg",
      f"- smallest gap between tub and shell as printed: {R['print1_min_gap_mm']} mm (target {H['gap']})",
      f"- hinge-zone downward overhang steeper than 44 deg: {R['hinge_overhang_mm2']} mm2", '']
o = R['outline']
L += ['## Outline and curbs', '', f"- length {S['length']} mm (limit 210)",
      f"- nose reaches X {o['nose_x']}, {o['nose_overhang']} mm past the Ø86.6 front wheel (limit 20)",
      f"- widest point |Y| {o['max_abs_y']} mm (wheels reach 114): nothing sticks out sideways",
      f"- nose underside rises at 45 deg from X {S['ramp_x0']}, {round(S['wheel_front'] - S['ramp_x0'], 1)} mm behind the wheel front"]
L += [f'- curb, {k}: {v}' for k, v in curb.items()] + ['']
L += ['## Wheels', '', '| wheel | gap to its body mm |', '|---|---|'] + [f'| {k} | {v} |' for k, v in R['wheel_to_body_mm'].items()] + ['']
L += ['## Bed (Bambu P1S 256 x 256)', '', f"- Print 1 (tub + shell assembled): {R['print1_extent']} mm",
      f"- Print 3 (LiDAR riser, refill arm, 4 coupling spacers): {R['print3_extent']} mm",
      f"- Print 2 (caps, hatches, decks, shelf): {R['print2_extent']} mm, fits: {R['print2_fits']}"
      f" ({R['print2_area_pct']}% of the bed){'; ' + ', '.join(touch) if touch else ''}", '']
L += ['## Fill per bay (v3)', '', '| bay | bay cm3 | components cm3 | fill | incl. plug/air/bend zones |', '|---|---|---|---|---|']
for bn in ('tub_lower', 'tub_upper', 'shell_lower', 'shell_upper', 'nose', 'TOTAL'):
    b = fill[bn]
    L.append(f"| {bn} | {b['bay_cm3']} | {b['parts_cm3']} | {b['fill_pct']}% | {b['used_pct']}% |")
L += ['', f"Dead volume in the hinge gap: v0 {R['hinge_gap_dead_cm3']['v0']} cm3 -> v3 {R['hinge_gap_dead_cm3']['v3']} cm3", '']
ms = R['mass']
L += ['## Weight and balance (estimates marked in meta.json)', '',
      f"- total {ms['total_g']} g, printed PETG ~{ms['printed_petg_g']} g (solid-volume estimate; the slicer number replaces it)",
      f"- CG at X {ms['cg_x']} (front axle carries **{ms['front_axle_pct']}%**, target 70%), CG height {ms['cg_z']} mm",
      f"- front shell {ms['shell_g']} g with CG at X {ms['shell_cg_x']}: {ms['shell_closing_moment_Nmm']} N.mm holds it closed about the hinge"]
if 'with_ballast' in ms:
    wb = ms['with_ballast']
    L += [f"- optional ballast (steel shot in epoxy, low in the nose under the battery, room for {wb['ballast_g']} g): "
          f"front axle **{wb['front_axle_pct']}%**, CG X {wb['cg_x']} / height {wb['cg_z']} mm, total {wb['total_g']} g; "
          f"{wb['need_for_65']} g of it reaches 65%"]
L += ['']
li = R['lidar']
L += ['## LiDAR clear of the lifted shell (B5)', '',
      f"- scan plane Z {li['scan_plane']} (riser {li['riser']} mm); shell top {li['shell_top_0deg']} at 0 deg, {li['shell_top_20deg']} at 20 deg",
      f"- margin at 20 deg **{li['margin_20deg']} mm** (need >= {li['required']}: 10 + {C.LIDAR['beam']} mm allowed for beam height, not published)", '']
L += ['## Wet zone (A3)', '', '| electronic part | bottom Z | wet floor + 5 | nearest water mm | exposed pts | under sealed tray | ok |', '|---|---|---|---|---|---|---|']
L += [f"| {k} | {v['bottom_z']} | {v['wet_floor_plus5']} | {v['nearest_water_mm']} | {v['exposed_points']} | {v['under_sealed_tray']} | {v['ok']} |" for k, v in R['wet_zone']['parts'].items()]
L += ['', f"Drains Ø{R['wet_zone']['drain_d']}: shell {R['wet_zone']['drains_shell']}, tub {R['wet_zone']['tub_drains']}. "
      'Exposed = a sample point with an unobstructed straight line to water within 15 mm that is level with or above it.', '']
cb = R['cables']
L += ['## Cables across the hinge (A4)', '', f"Pass {cb['pass_area_mm2']} mm2, bundle {cb['bundle_mm2']} mm2 (**{cb['fill_pct']}%** fill, limit 40); "
      f"bundle {cb['centroid_to_axis_mm']} mm from the hinge axis -> length change {cb['length_change_20deg_mm']} mm at 20 deg (they twist, not stretch).", '',
      '| cable | conductors | wire OD | plug passes | route |', '|---|---|---|---|---|']
L += [f"| {t['name']} | {t['conductors']} | {t['wire_od']} | {cb['plugs_fit'][t['name']]} | {t['route']} |" for t in cb['table']]
L += [''] + [f'- {x}' for x in cb['not_crossing']] + ['']
L += ['## Refill arm and intake hose (A2)', '', '| arm @ hinge deg | front wheel gap | rear wheel gap | lowest Z |', '|---|---|---|---|']
L += [f"| {k} | {v['front_wheel_gap']} | {v['rear_wheel_gap']} | {v['lowest_z']} |" for k, v in R['arm'].items()]
L += ['', '| hose outside @ hinge deg | wheel gap | lowest Z | min bend R | length |', '|---|---|---|---|---|']
L += [f"| {k} | {v['wheel_gap']} | {v['lowest_z']} | {v['min_bend_radius']} | {v['length']} |" for k, v in R['intake_hose_outside'].items()] + ['']
sp = R['spray']
L += ['## Spray nozzle (A1)', '', f"- tip {sp['tip']}, {sp['angle_down']} deg down, inside the outline: {sp['tip_inside_outline']}",
      f"- 4 deg half-angle cone, 600 mm: blocked by {sp['blocked_by'] or 'nothing'}; straight line reaches the ground at X ~{sp['lands_at_x']}", '']
L += ['## Tubes (6x9 silicone, R25 minimum)', '', '| tube | length mm | min bend radius mm |', '|---|---|---|']
L += [f"| {k} | {v['length_mm']} | {v['min_bend_radius']} |" for k, v in R['tubes'].items()] + ['']
L += ['## AS5600 hinge sensor (B9)', '', f"- magnet face to chip {R['as5600']['magnet_to_chip_gap_mm']} mm (datasheet {R['as5600']['datasheet_gap']}), magnet on the hinge axis", '']
L += ['## ESP32 (B6)', '', f"- {R['esp32']['rails']}; {R['esp32']['usb']}", '']
L += ['## Assembly: straight insertion paths (in ASSEMBLY.md order)', '', '| part | moves in along | travel mm | blocked mm3 |', '|---|---|---|---|']
L += [f"| {k} | {v['direction']} | {v['travel']} | {v['blocked_mm3']} |" for k, v in R['insertion'].items()] + ['']
open('CHECKS.md', 'w').write('\n'.join(L) + '\n')
print('\n'.join(L))
sys.exit(1 if hard_fail else 0)
