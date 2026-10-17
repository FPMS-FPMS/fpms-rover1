"""Fit checks by computation. Run from chassis/ after fpms_rover1_cad.py (reads out/).
Writes CHECKS.md and out/checks.json. Exit code 1 if a hard check fails."""
import json, math, os, sys, numpy as np, trimesh, manifold3d as m3
sys.path.insert(0, '.')
import fpms_rover1_cad as C
OUT = 'out'
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

parts = {n: load(n) for n in meta['print1'] + meta['print2']}
comps = {n: load(f'comp/{n}') for n in meta['components']}
keeps = {n: load(f'comp/keepout_{n}') for n in meta['keepouts']}
for n, m in list(parts.items()) + list(comps.items()) + list(keeps.items()):
    assert m.is_watertight, f'{n} is not watertight - every boolean below would read 0 and pass'
P = {n: man(m) for n, m in parts.items()}
Cm = {n: man(m) for n, m in comps.items()}
K = {n: man(m) for n, m in keeps.items()}
R = {}; hard_fail = []

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
    if seat == 'bottom' or isinstance(seat, dict):
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
                                     [comps[n] for n in comps if meta['components'][n]['body'] == 'shell' or n.startswith('wheel_front')])
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

# ---------- 4. outline + curb ----------
wheel_front = S['wheel_front']
everything = [m for n, m in parts.items()] + [m for n, m in comps.items() if not n.startswith(('wheel', 'estop_head', 'motor'))]  # shafts sit in the wheel hubs
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
                                        if meta['components'][n]['body'] == 'shell' and not n.startswith('wheel')])
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
    'v2': dict(length=S['length'], nose_overhang=R['outline']['nose_overhang'], top=S['H'], fill=fill['TOTAL']['fill_pct'],
               used=fill['TOTAL']['used_pct'], hull=hull_v2, front_axle=None)}
R['hinge_gap_dead_cm3'] = dict(v0=round(32.0 * 148 * (134.9 - 26) / 1000, 1),
                               v2=round(0.5 * (S['Z_SPLIT'] - 40) ** 2 * math.tan(math.radians(H['rake'])) * 148 / 1000, 1))

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
R['compare']['v2']['front_axle'] = R['mass']['front_axle_pct']
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

json.dump(R, open(f'{OUT}/checks.json', 'w'), indent=1, default=float)

# ---------- report ----------
L = ['# Fit checks (computed by checks.py from the exported meshes)', '']
L += [f"**Hard checks: {'ALL PASS' if not hard_fail else 'FAIL - ' + '; '.join(hard_fail)}**", '']
cmp = R['compare']
L += ['## v0 / v1 / v2', '', '| | v0 (original) | v1 | v2 |', '|---|---|---|---|']
def cell(v, unit=''):
    return '-' if v is None else f'{v}{unit}'
for key, lab, unit in (('length', 'Length', ' mm'), ('nose_overhang', 'Nose past front wheel (Ø86.6 worst case)', ' mm'),
                       ('top', 'Top of body (LiDAR not included)', ' mm'), ('fill', 'Fill: component volume / bay volume', '%'),
                       ('used', 'Fill incl. plug, fan-air and tube-bend zones', '%'), ('hull', 'Space claimed (convex hull)', ' cm3'),
                       ('front_axle', 'Weight on front axle', '%')):
    L.append(f"| {lab} | {cell(cmp['v0'][key], unit)} | {cell(cmp['v1'][key], unit)} | {cell(cmp['v2'][key], unit)} |")
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
      f"- Print 2 (caps, hatch, decks, shelf, LiDAR plate): {R['print2_extent']} mm, fits: {R['print2_fits']}"
      f" ({R['print2_area_pct']}% of the bed){'; ' + ', '.join(touch) if touch else ''}", '']
L += ['## Fill per bay (v2)', '', '| bay | bay cm3 | components cm3 | fill | incl. plug/air/bend zones |', '|---|---|---|---|---|']
for bn in ('tub_lower', 'tub_upper', 'shell_lower', 'shell_upper', 'nose', 'TOTAL'):
    b = fill[bn]
    L.append(f"| {bn} | {b['bay_cm3']} | {b['parts_cm3']} | {b['fill_pct']}% | {b['used_pct']}% |")
L += ['', f"Dead volume in the hinge gap: v0 {R['hinge_gap_dead_cm3']['v0']} cm3 -> v2 {R['hinge_gap_dead_cm3']['v2']} cm3", '']
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
open('CHECKS.md', 'w').write('\n'.join(L) + '\n')
print('\n'.join(L))
sys.exit(1 if hard_fail else 0)
