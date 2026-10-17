"""Fit checks by computation. Run from chassis/ after fpms_rover1_cad.py (reads out/).
Writes CHECKS.md and out/checks.json. Exit code 1 if a hard check fails."""
import json, math, sys, numpy as np, trimesh, manifold3d as m3
from trimesh.grouping import group_rows
OUT = 'out'
meta = json.load(open(f'{OUT}/meta.json'))
S = meta['summary']; H = meta['hinge']; BED = 256.0
PETG = 1.27  # g/cm3

def load(n):
    return trimesh.load(f'{OUT}/{n}.stl')
def man(mesh):
    return m3.Manifold(m3.Mesh(vert_properties=np.asarray(mesh.vertices, np.float32), tri_verts=np.asarray(mesh.faces, np.uint32)))
def inter_vol(a, b):
    return (a ^ b).volume()
def rot_shell(mesh, deg):
    """rotate about the hinge axis (Y through X=H.x, Z=H.z), +deg = nose up"""
    M = trimesh.transformations.rotation_matrix(math.radians(-deg), [0, 1, 0], [H['x'], 0, H['z']])
    # trimesh rotates by the right-hand rule about +Y: positive angle turns +X toward -Z (nose down), so negate
    m = mesh.copy(); m.apply_transform(M); return m

parts = {n: load(n) for n in meta['print1'] + meta['print2']}
comps = {n: load(f'comp/{n}') for n in meta['components']}
keeps = {n: load(f'comp/keepout_{n}') for n in meta['keepouts']}
for n, m in list(parts.items()) + list(comps.items()):
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
        v = inter_vol(Cm[names[i]], Cm[names[j]])
        if v > 0.5 and not ({names[i].split('_')[0], names[j].split('_')[0]} == {'motor', 'wheel'}):
            ov.append((names[i], names[j], round(v, 1)))
for kn, km in K.items():
    for pn, pm in P.items():
        v = inter_vol(km, pm)
        if v > 0.5:
            ov.append(('keepout ' + kn, pn, round(v, 1)))
    for cn, cm in Cm.items():
        if cn.startswith('wheel'):
            continue
        v = inter_vol(km, cm)
        own = kn.startswith('pi') and cn.startswith('orange') or kn.startswith('pump_1') and cn == 'pump_1' or kn.startswith('pump_2') and cn == 'pump_2'
        if v > 0.5 and not own:
            ov.append(('keepout ' + kn, cn, round(v, 1)))
R['overlaps'] = ov
if ov:
    hard_fail.append(f'{len(ov)} overlaps')

# ---------- 2. clearance from each component to the nearest printed part (its support face excluded) ----------
allparts = trimesh.util.concatenate([parts[n] for n in parts])
pq = trimesh.proximity.ProximityQuery(allparts)
clr = {}
for cn, cm in comps.items():
    if cn.startswith('wheel') or cn in ('estop_head',):
        continue
    pts, fi = trimesh.sample.sample_surface_even(cm, 2500, seed=1)
    nz = cm.face_normals[fi][:, 2]
    side = pts[nz > -0.5]          # drop the face it sits on
    if len(side) == 0:
        continue
    _, d, _ = pq.on_surface(side)   # unsigned: overlaps are caught by the booleans above
    clr[cn] = round(float(d.min()), 2)
R['clearance_mm'] = clr

# ---------- 3. hinge sweep ----------
tub_grp = man(trimesh.util.concatenate([parts[n] for n in parts if meta['body_of'][n] == 'tub'] +
                                       [comps[n] for n in comps if meta['components'][n]['body'] == 'tub']))
shell_meshes = [parts[n] for n in parts if meta['body_of'][n] == 'shell'] + \
               [comps[n] for n in comps if meta['components'][n]['body'] == 'shell' or n.startswith('wheel_front')]
shell_all = trimesh.util.concatenate(shell_meshes)
sweep = []
for k in range(0, 101):
    a = -3.0 + 0.25 * k
    v = inter_vol(tub_grp, man(rot_shell(shell_all, a)))
    sweep.append((a, round(v, 2)))
free = [a for a, v in sweep if v < 0.05]
lo_stop = max([a for a, v in sweep if a < 0 and v >= 0.05], default=None)
hi_stop = min([a for a, v in sweep if a > 0 and v >= 0.05], default=None)
R['hinge_sweep'] = dict(free_from=min(free), free_to=max(free), nose_down_stop_deg=lo_stop, nose_up_stop_deg=hi_stop,
                        samples=sweep[::4])
if not (min(free) <= 0 and max(free) >= 20):
    hard_fail.append(f'hinge does not move freely 0..20 (free {min(free)}..{max(free)})')
# gap between the two printed bodies as printed (0 deg)
tub_m, sh_m = P['main_tub'], P['front_shell']
gap0 = tub_m.min_gap(sh_m, 5.0)
R['print1_min_gap_mm'] = round(gap0, 3)
if gap0 < H['gap'] - 0.02:
    hard_fail.append(f'print-in-place gap {gap0:.3f} < {H["gap"]}')
# the rear wheels against the tub, front wheels against the shell (static)
wc = {}
for wn in [n for n in comps if n.startswith('wheel')]:
    body = 'main_tub' if 'rear' in wn else 'front_shell'
    wc[wn] = round(Cm[wn].min_gap(P[body], 10.0), 2)
R['wheel_to_body_mm'] = wc

# ---------- 4. bed fit ----------
p1 = trimesh.util.concatenate([parts['main_tub'], parts['front_shell']])
ext1 = (p1.bounds[1] - p1.bounds[0]).round(1).tolist()
R['print1_extent'] = ext1
if max(ext1[0], ext1[1]) > BED - 4 or ext1[2] > BED:
    hard_fail.append(f'print 1 does not fit {ext1}')
# print 2: lay every part as it prints and shelf-pack, allowing 90 deg turns
items = []
for n in meta['print2']:
    m = parts[n].copy()
    if meta['bed_flip'][n]:
        m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(meta['bed_flip'][n]), [1, 0, 0]))
    m.apply_translation(-m.bounds[0]); items.append((n, m))
sys.path.insert(0, '.')
import fpms_rover1_cad as C
lay = C.print_layouts(parts)
placed = lay[2]
missing = [n for n in meta['print2'] if n not in {p for p, _ in placed}]
p2 = trimesh.util.concatenate([m for _, m in placed])
# parts on the bed must not touch each other (2 mm minimum) and must stay on the plate
touch = []
for i in range(len(placed)):
    for j in range(i + 1, len(placed)):
        g = man(placed[i][1]).min_gap(man(placed[j][1]), 3.0)
        if g < 2.0:
            touch.append((placed[i][0], placed[j][0], round(g, 2)))
R['print2_extent'] = (p2.bounds[1] - p2.bounds[0]).round(1).tolist()
R['print2_bed_max'] = p2.bounds[1].round(1).tolist()
R['print2_missing'] = missing + [f'{a}~{b} {g}mm' for a, b, g in touch]
R['print2_fits'] = bool(not missing and not touch and p2.bounds[0][0] >= 0 and p2.bounds[0][1] >= 0
                        and p2.bounds[1][0] <= BED and p2.bounds[1][1] <= BED and p2.bounds[1][2] <= BED)
p1b = trimesh.util.concatenate([m for _, m in lay[1]])
R['print1_on_bed'] = dict(min=p1b.bounds[0].round(1).tolist(), max=p1b.bounds[1].round(1).tolist())
R['print2_area_pct'] = round(100 * sum(m.extents[0] * m.extents[1] for _, m in items) / BED ** 2, 1)
if not R['print2_fits']:
    hard_fail.append(f'print 2 does not fit {R["print2_extent"]}')
p2.export(f'{OUT}/print2_layout_preview.stl')

# ---------- 5. fill % per bay (v1) and the v0 baseline ----------
sys.path.insert(0, '.')
import fpms_rover1_cad as C
import cadquery as cq
cav = C.cavities(); bays = C.bays()
fill = {}
for bn, (body, region) in bays.items():
    bv = cav[body].intersect(region).val().Volume() / 1000
    cv = sum(abs(comps[c].volume) for c, d in meta['components'].items() if d['bay'] == bn) / 1000
    kv = sum(abs(keeps[k].volume) for k, d in meta['keepouts'].items() if d['bay'] == bn) / 1000
    fill[bn] = dict(bay_cm3=round(bv, 1), parts_cm3=round(cv, 1), keepout_cm3=round(kv, 1),
                    fill_pct=round(100 * cv / bv, 1), used_pct=round(100 * (cv + kv) / bv, 1))
tot_b = sum(f['bay_cm3'] for f in fill.values()); tot_c = sum(f['parts_cm3'] for f in fill.values())
fill['TOTAL'] = dict(bay_cm3=round(tot_b, 1), parts_cm3=round(tot_c, 1), fill_pct=round(100 * tot_c / tot_b, 1),
                     used_pct=round(100 * (tot_c + sum(f['keepout_cm3'] for f in fill.values())) / tot_b, 1))
R['fill_v1'] = fill
# v0: interiors straight from the original script's constants (boxes, its r12/r4 corners ignored -> v0 is slightly flattered)
vol = {n: abs(comps[n].volume) / 1000 for n in comps}
motor = vol['motor_rear_L']
v0 = dict(
    tub_lower=dict(bay_cm3=67.7 * 143.2 * (66 - 28.4) / 1000, parts_cm3=2 * motor + vol['regulator_5v']),
    tub_upper=dict(bay_cm3=67.7 * 143.2 * (134.9 - 68.4) / 1000, parts_cm3=vol['orange_pi_n515'] + vol['yahboom_board'] + vol['estop_body']),
    shell_lower=dict(bay_cm3=(122.7 - 66.9) * 143.2 * (66 - 28.4) / 1000, parts_cm3=2 * motor),
    shell_upper=dict(bay_cm3=(122.7 - 66.9) * 143.2 * (134.9 - 68.4) / 1000,
                     parts_cm3=vol['battery_3s2p'] + 2 * vol['nine_volt_1'] + 2 * vol['xy_mos_1'] + vol['esp32_s3']),
    nose=dict(bay_cm3=(163.6 - 123.2) * 143.2 * (134.9 - 28.4) / 1000, parts_cm3=vol['pump_1'] + vol['pump_2'] + vol['tank_water']))
for v in v0.values():
    v['fill_pct'] = round(100 * v['parts_cm3'] / v['bay_cm3'], 1); v['bay_cm3'] = round(v['bay_cm3'], 1); v['parts_cm3'] = round(v['parts_cm3'], 1)
tb = sum(v['bay_cm3'] for v in v0.values()); tc = sum(v['parts_cm3'] for v in v0.values())
v0['TOTAL'] = dict(bay_cm3=round(tb, 1), parts_cm3=round(tc, 1), fill_pct=round(100 * tc / tb, 1))
R['fill_v0'] = v0
R['hinge_gap_dead_cm3'] = dict(v0=round(32.0 * 148 * (134.9 - 26) / 1000, 1),
                               v1=round(0.5 * (S['TOP_S'] - 40) ** 2 * math.tan(math.radians(H['rake'])) * 148 / 1000, 1))
env1 = (S['NOSE'] - S['XRO']) * 148 * (S['TOP_T'] - 26) / 1000
R['envelope'] = dict(v0_len=206.0, v1_len=round(S['NOSE'] - S['XRO'], 1), v0_top=137.3, v1_top_tub=S['TOP_T'] + 2.4, v1_top_shell=S['TOP_S'] + 2.4)
# space the chassis claims: convex hull of tub + shell (+ lids), v0 from its own STLs
def hull_cm3(ms):
    return round(trimesh.util.concatenate(ms).convex_hull.volume / 1000, 1)
import os
V0 = 'out/v0/out'
if os.path.exists(f'{V0}/main_tub_pip.stl'):
    R['hull_cm3'] = dict(v0=hull_cm3([trimesh.load(f'{V0}/{n}.stl') for n in ('main_tub_pip', 'front_shell_pip', 'lid_main', 'lid_front')]),
                         v1=hull_cm3([parts[n] for n in ('main_tub', 'front_shell', 'lid_tub', 'lid_shell')]))

# ---------- 6. weight and balance ----------
mass = []
for n, m in parts.items():
    g = abs(m.volume) / 1000 * PETG
    mass.append((n, meta['body_of'][n], g, m.center_mass))
for n, d in meta['components'].items():
    if d['mass']:
        mass.append((n, d['body'] if d['body'] != 'wheel' else ('tub' if 'rear' in n else 'shell'), d['mass'], comps[n].center_mass))
mass.append(('wiring+screws (estimate)', 'tub', 50.0, np.array([0.0, 0, 70])))
mass.append(('wiring+tubes (estimate)', 'shell', 50.0, np.array([100.0, 0, 70])))
M = sum(g for _, _, g, _ in mass); X = sum(g * c[0] for _, _, g, c in mass) / M; Z = sum(g * c[2] for _, _, g, c in mass) / M
Ms = sum(g for _, b, g, _ in mass if b == 'shell'); Xs = sum(g * c[0] for _, b, g, c in mass if b == 'shell') / Ms
R['mass'] = dict(total_g=round(M), cg_x=round(X, 1), cg_z=round(Z, 1), front_axle_pct=round(100 * X / 93.5, 1),
                 shell_g=round(Ms), shell_cg_x=round(Xs, 1), shell_closing_moment_Nmm=round(Ms / 1000 * 9.81 * (Xs - H['x']), 1),
                 printed_petg_g=round(sum(g for n, _, g, _ in mass if n in parts)),
                 items=[(n, b, round(g), round(float(c[0]), 1)) for n, b, g, c in mass])

# ---------- 7. hinge printability: downward faces in the hinge zone ----------
def overhang_area(mesh, zone):
    fn = mesh.face_normals; c = mesh.triangles_center
    sel = (fn[:, 2] < -0.72) & (c[:, 2] > 26.3) & zone(c)
    return float(mesh.area_faces[sel].sum()), int(sel.sum())
zone = lambda c: (np.abs(c[:, 0] - H['x']) < 16) & (c[:, 2] < 62) & (np.abs(c[:, 1]) > 28)
R['hinge_overhang_mm2'] = {n: round(overhang_area(parts[n], zone)[0], 1) for n in ('main_tub', 'front_shell')}

json.dump(R, open(f'{OUT}/checks.json', 'w'), indent=1, default=float)

# ---------- report ----------
L = ['# Fit checks (computed by checks.py from the exported meshes)', '']
L += [f"**Hard checks: {'ALL PASS' if not hard_fail else 'FAIL - ' + '; '.join(hard_fail)}**", '']
L += ['## Overlaps (must be zero)', '', f"{len(ov)} overlaps" + ('' if not ov else ':')] + [f'- {a} x {b}: {v} mm3' for a, b, v in ov] + ['']
L += ['## Component clearance to the nearest printed part (support face excluded)', '', '| component | min gap mm |', '|---|---|']
L += [f'| {k} | {v} |' for k, v in sorted(clr.items(), key=lambda kv: kv[1])] + ['']
hs = R['hinge_sweep']
L += ['## Hinge sweep (shell + its lid, deck, components and front wheels rotated about the pin)', '',
      f"- moves freely from {hs['free_from']} to {hs['free_to']} deg (0.25 deg steps)",
      f"- nose-down stop (0 deg catch) touches at {hs['nose_down_stop_deg']} deg; nose-up stop at {hs['nose_up_stop_deg']} deg",
      f"- smallest gap between tub and shell as printed: {R['print1_min_gap_mm']} mm (target {H['gap']})",
      f"- hinge-zone downward overhang steeper than 44 deg: {R['hinge_overhang_mm2']} mm2", '']
L += ['## Wheels', '', '| wheel | gap to its body mm |', '|---|---|'] + [f'| {k} | {v} |' for k, v in wc.items()] + ['']
L += ['## Bed (Bambu P1S 256 x 256 x 256)', '', f"- Print 1 (tub + shell assembled): {ext1} mm",
      f"- Print 2 (lids, decks, LiDAR mount, hatch): packed footprint {R['print2_extent']} mm, fits: {R['print2_fits']}"
      f" (parts cover {R['print2_area_pct']}% of the bed{'; did not fit: ' + ', '.join(R['print2_missing']) if R['print2_missing'] else ''})", '']
L += ['## Fill per bay', '', '| bay | v0 bay cm3 | v0 fill | v1 bay cm3 | v1 fill | v1 incl. plug/air/bend zones |', '|---|---|---|---|---|---|']
for bn in ('tub_lower', 'tub_upper', 'shell_lower', 'shell_upper', 'nose', 'TOTAL'):
    a, b = v0[bn], fill[bn]
    L.append(f"| {bn} | {a['bay_cm3']} | {a['fill_pct']}% | {b['bay_cm3']} | {b['fill_pct']}% | {b.get('used_pct', '')}% |")
L += ['', f"Dead volume in the hinge gap: v0 {R['hinge_gap_dead_cm3']['v0']} cm3 -> v1 {R['hinge_gap_dead_cm3']['v1']} cm3",
      f"Space the whole chassis claims (convex hull, body + lids): {R.get('hull_cm3')}", '']
ms = R['mass']
L += ['## Weight and balance (estimates marked in meta.json)', '',
      f"- total {ms['total_g']} g, printed PETG ~{ms['printed_petg_g']} g (solid-volume estimate; the slicer number replaces it)",
      f"- CG at X {ms['cg_x']} (front axle carries **{ms['front_axle_pct']}%**, target 70%), CG height {ms['cg_z']} mm",
      f"- front shell {ms['shell_g']} g with CG at X {ms['shell_cg_x']}: {ms['shell_closing_moment_Nmm']} N.mm holds it closed about the hinge", '']
open('CHECKS.md', 'w').write('\n'.join(L) + '\n')
print('\n'.join(L))
sys.exit(1 if hard_fail else 0)
