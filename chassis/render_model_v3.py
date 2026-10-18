"""Render the current build (out/) with every component as a coloured ghost.
Writes renders/v1_*.png: iso, iso_rear, side, top, front, sections through each axle, the hinge,
the centreline and a plan cut. Run from chassis/ after fpms_rover1_cad.py."""
import json, os, glob, sys, trimesh, numpy as np
from render import render_views, section
OUT = 'out_v3'; R = 'renders'; P = sys.argv[1] if len(sys.argv) > 1 else 'v3'
meta = json.load(open(f'{OUT}/meta.json'))
PART_COL = dict(main_tub=(0.45, 0.58, 0.78), front_shell=(0.86, 0.55, 0.38), tub_cap=(0.55, 0.75, 0.55), shell_cap=(0.55, 0.75, 0.55),
                tank_hatch=(0.4, 0.6, 0.4), tub_deck=(0.85, 0.78, 0.35), tub_shelf=(0.85, 0.78, 0.35), pump_deck=(0.85, 0.78, 0.35),
                lidar_mount=(0.3, 0.3, 0.3), batt_hatch=(0.75, 0.65, 0.35), rear_hatch=(0.85, 0.85, 0.82), esp_carrier=(0.4, 0.55, 0.75))
COMP_COL = {'motor': (0.2, 0.2, 0.25), 'wheel': (0.15, 0.15, 0.15), 'orange_pi': (0.9, 0.45, 0.1), 'yahboom': (0.1, 0.5, 0.9),
            'estop': (0.9, 0.1, 0.1), 'regulator': (0.2, 0.7, 0.2), 'lidar': (0.1, 0.1, 0.1), 'pump': (0.2, 0.6, 0.9),
            'tank': (0.3, 0.8, 1.0), 'battery': (0.95, 0.85, 0.1), 'nine': (0.7, 0.2, 0.8), 'esp32': (0.1, 0.4, 0.2),
            'xy_mos': (0.5, 0.3, 0.1), 'vl53': (0.9, 0.2, 0.6), 'servo': (0.3, 0.3, 0.9), 'tube': (0.0, 0.65, 1.0),
            'nozzle': (0.75, 0.75, 0.75), 'refill_arm': (0.9, 0.6, 0.1), 'as5600': (0.2, 0.7, 0.4), 'charge': (0.6, 0.6, 0.6), 'rgb': (1.0, 0.2, 0.8)}
def ccol(n):
    for k, v in COMP_COL.items():
        if n.startswith(k):
            return v
    return (0.5, 0.5, 0.5)

def scene(lids=True, ghosts=True, keepouts=False):
    sc = []
    for n in meta['print1'] + meta['print2'] + [p for p in meta['print3'] if p in ('lidar_mount', 'rear_hatch', 'esp_carrier')]:
        if not lids and n.endswith('_cap') or not lids and n in ('tank_hatch', 'batt_hatch', 'lidar_mount'):
            continue
        op = 0.3 if n.endswith('_cap') or n.endswith('_hatch') else 1.0
        sc.append(dict(path=f'{OUT}/{n}.stl', color=PART_COL[n], opacity=op, name=n))
    if ghosts:
        for n, c in meta['components'].items():
            if n == 'refill_arm_deployed' and not keepouts:
                continue
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
            ('y', [0, -45.0, 0], 'plumbing bay / ESP32 Y=-45'), ('y', [0, -66.0, 0], 'tube lanes Y=-66'), ('y', [0, -16.0, 0], 'tank Y=-16'),
            ('y', [0, 38.0, 0], 'battery Y=38'), ('y', [0, 21.6, 0], 'AS5600 Y=21.6'), ('z', [0, 0, 90.0], 'plan Z=90'),
            ('z', [0, 0, 70.6], 'servo pivot Z=70.6'), ('z', [0, 0, 130.0], 'plan Z=130'), ('z', [0, 0, 152.0], 'tube layer Z=152')]
    for ax, o, lab in secs:
        print(section(scene(keepouts=True), R, P, ax, o, lab))
