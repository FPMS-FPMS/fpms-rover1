"""Baseline: run the ORIGINAL concept-C script unchanged (v0) and render it with ghost motors and wheels.
Run from chassis/:  python render_baseline.py"""
import os, sys, subprocess, trimesh, numpy as np
from render import render_views, section
HERE = os.path.dirname(os.path.abspath(__file__))
V0 = os.path.join(HERE, 'out', 'v0')
os.makedirs(V0, exist_ok=True)
if not os.path.exists(os.path.join(V0, 'out', 'main_tub_pip.stl')):
    subprocess.run([sys.executable, os.path.join(HERE, 'v0_original.py')], cwd=V0, check=True)
d = os.path.join(V0, 'out')


def cyl_y(x, z, r, y0, y1):
    c = trimesh.creation.cylinder(radius=r, height=abs(y1 - y0), sections=48)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation([x, (y0 + y1) / 2, z]); return c


ghosts = []
for xa in (0.0, 93.5):
    for s in (1, -1):   # v0 puts the gearbox face at |y| 67.5, gearbox centre 7 above the axle
        ghosts.append(cyl_y(xa, 47, 18.5, s * 67.5, s * 45.5)); ghosts.append(cyl_y(xa, 47, 16.4, s * 45.5, s * 5.5))
        ghosts.append(cyl_y(xa, 40, 40, s * 76, s * 114))
G = (0.2, 0.75, 0.3)
sc = [dict(path=f'{d}/main_tub_pip.stl', color=(0.55, 0.65, 0.8), name='main tub'),
      dict(path=f'{d}/front_shell_pip.stl', color=(0.85, 0.6, 0.45), name='front shell'),
      dict(path=f'{d}/lid_main.stl', color=(0.6, 0.8, 0.6), opacity=0.3, name='lid main'),
      dict(path=f'{d}/lid_front.stl', color=(0.6, 0.8, 0.6), opacity=0.3, name='lid front'),
      dict(path=f'{d}/main_deck.stl', color=(0.8, 0.75, 0.3), name='main deck'),
      dict(path=f'{d}/front_deck.stl', color=(0.8, 0.75, 0.3), name='front deck'),
      dict(path=f'{d}/lidar_mount.stl', color=(0.3, 0.3, 0.3), name='lidar mount'),
      dict(mesh=trimesh.util.concatenate(ghosts), color=G, opacity=0.35, name='motors + wheels (ghost)')]
R = os.path.join(HERE, 'renders')
print(render_views(sc, R, 'v0'))
for lab, o in (('rear axle X=0', [0, 0, 0]), ('front axle X=93.5', [93.5, 0, 0]), ('hinge X=48.5', [48.5, 0, 0])):
    print(section(sc, R, 'v0', 'x', o, lab))
print(section(sc, R, 'v0', 'y', [0, 60, 0], 'side Y=60'))
print(section(sc, R, 'v0', 'y', [0, 0.5, 0], 'centre Y=0'))
