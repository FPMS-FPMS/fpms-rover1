"""Render the current build (out/) with every component as a coloured ghost.
Writes renders/v1_*.png: iso, iso_rear, side, top, front, sections through each axle, the hinge,
the centreline and a plan cut. Run from chassis/ after fpms_rover1_cad.py."""
import json, os, glob, sys, trimesh, numpy as np
from render import render_views, section
OUT = 'out'; R = 'renders'; P = sys.argv[1] if len(sys.argv) > 1 else 'v1'
meta = json.load(open(f'{OUT}/meta.json'))
PART_COL = dict(main_tub=(0.45, 0.58, 0.78), front_shell=(0.86, 0.55, 0.38), lid_tub=(0.55, 0.75, 0.55), lid_shell=(0.55, 0.75, 0.55),
                tank_hatch=(0.4, 0.6, 0.4), tub_deck=(0.85, 0.78, 0.35), tub_shelf=(0.85, 0.78, 0.35), shell_deck=(0.85, 0.78, 0.35),
                lidar_mount=(0.3, 0.3, 0.3))
COMP_COL = {'motor': (0.2, 0.2, 0.25), 'wheel': (0.15, 0.15, 0.15), 'orange_pi': (0.9, 0.45, 0.1), 'yahboom': (0.1, 0.5, 0.9),
            'estop': (0.9, 0.1, 0.1), 'regulator': (0.2, 0.7, 0.2), 'lidar': (0.1, 0.1, 0.1), 'pump': (0.2, 0.6, 0.9),
            'tank': (0.3, 0.8, 1.0), 'battery': (0.95, 0.85, 0.1), 'nine': (0.7, 0.2, 0.8), 'esp32': (0.1, 0.4, 0.2),
            'xy_mos': (0.5, 0.3, 0.1), 'vl53': (0.9, 0.2, 0.6), 'servo': (0.3, 0.3, 0.9)}
def ccol(n):
    for k, v in COMP_COL.items():
        if n.startswith(k):
            return v
    return (0.5, 0.5, 0.5)

def scene(lids=True, ghosts=True, keepouts=False):
    sc = []
    for n in meta['print1'] + meta['print2']:
        if not lids and n.startswith(('lid', 'tank_hatch', 'lidar_mount')):
            continue
        op = 0.28 if n.startswith(('lid', 'tank_hatch')) else 1.0
        sc.append(dict(path=f'{OUT}/{n}.stl', color=PART_COL[n], opacity=op, name=n))
    if ghosts:
        for n, c in meta['components'].items():
            sc.append(dict(path=f'{OUT}/comp/{n}.stl', color=ccol(n), opacity=0.5 if not n.startswith('wheel') else 0.18, name=n))
    if keepouts:
        for n in meta['keepouts']:
            sc.append(dict(path=f'{OUT}/comp/keepout_{n}.stl', color=(1, 0.2, 0.2), opacity=0.12, name='keepout ' + n))
    return sc

if __name__ == '__main__':
    os.makedirs(R, exist_ok=True)
    print(render_views(scene(), R, P))
    print(render_views(scene(lids=False, keepouts=True), R, P + '_open', views=('iso', 'top')))
    s = meta['summary']
    secs = [('x', [0.0, 0, 0], 'rear axle X=0'), ('x', [93.5, 0, 0], 'front axle X=93.5'), ('x', [48.5, 0, 0], 'hinge X=48.5'),
            ('x', [s['S2_X0'] + 25, 0, 0], f"nose bay X={s['S2_X0'] + 25:.0f}"),
            ('y', [0, 0.3, 0], 'centre Y=0'), ('y', [0, 50.0, 0], 'hinge knuckle Y=50'), ('y', [0, 36.0, 0], 'hinge cup Y=36'),
            ('y', [0, -40.0, 0], 'side Y=-40'), ('z', [0, 0, 90.0], 'plan Z=90'), ('z', [0, 0, 50.0], 'plan Z=50')]
    for ax, o, lab in secs:
        print(section(scene(keepouts=True), R, P, ax, o, lab))
