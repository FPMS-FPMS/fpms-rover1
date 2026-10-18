"""Build preview.html: one self-contained file (three.js r160 inlined) showing the chassis, every component as a
coloured ghost, a hinge slider, show/hide toggles, a section plane and the two print-bed layouts.
Run from chassis/ after fpms_rover1_cad.py and checks.py.  Needs three.module.min.js + OrbitControls.js (r160) in
the folder given by --three (downloaded once from cdn.jsdelivr.net/npm/three@0.160.0)."""
import base64, json, math, os, sys
import numpy as np, trimesh
sys.path.insert(0, '.')
import fpms_rover1_v3 as C

THREE_DIR = sys.argv[sys.argv.index('--three') + 1] if '--three' in sys.argv else 'three'
OUT = 'out_v3'
meta = json.load(open(f'{OUT}/meta.json'))
S = meta['summary']; H = meta['hinge']
chk = json.load(open(f'{OUT}/checks.json'))

PART_STYLE = {  # group, colour, opacity
    'main_tub': ('tub', '#6f8fc4', 1.0), 'front_shell': ('shell', '#dd8c5e', 1.0),
    'tub_cap': ('lids', '#8db8d8', 0.35), 'shell_cap': ('lids', '#e8ad8a', 0.35), 'tank_hatch': ('lids', '#4fb0c8', 0.6),
    'tub_deck': ('decks', '#e0c34f', 1.0), 'tub_shelf': ('decks', '#e0c34f', 1.0), 'pump_deck': ('decks', '#e0c34f', 1.0),
    'lidar_mount': ('decks', '#555555', 1.0), 'batt_hatch': ('lids', '#f0c060', 0.55),
    'rear_hatch': ('tub', '#c8c8c0', 0.75), 'esp_carrier': ('decks', '#6a8cc0', 1.0),
    'refill_arm': ('bedonly', '#e0a020', 1.0), 'spacer_1': ('bedonly', '#999999', 1.0), 'spacer_2': ('bedonly', '#999999', 1.0),
    'spacer_3': ('bedonly', '#999999', 1.0), 'spacer_4': ('bedonly', '#999999', 1.0)}
COMP_COL = [('motor', '#33333d'), ('wheel', '#222222'), ('orange_pi', '#ff7a1a'), ('yahboom', '#1f7fe0'), ('estop', '#e01b1b'),
            ('regulator', '#2fb34a'), ('lidar', '#111111'), ('pump', '#38a0ff'), ('tank', '#4fd8ff'), ('battery', '#f2d20f'),
            ('nine', '#b03acc'), ('esp32', '#167a3a'), ('xy_mos', '#8a5a22'), ('vl53', '#ff3aa0'), ('servo', '#4a4ae0'),
            ('tube', '#00a8ff'), ('nozzle', '#c8c8c8'), ('refill_arm', '#e0a020'), ('as5600', '#3cb371'), ('charge', '#9a9a9a'),
            ('rgb', '#ff33cc')]
def comp_col(n):
    return next((c for k, c in COMP_COL if n.startswith(k)), '#888888')

def pack(mesh):
    m = mesh.copy(); m.merge_vertices()
    v = np.asarray(m.vertices, np.float32); f = np.asarray(m.faces, np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode(), nv=len(v), nf=len(f))

def fit_matrix(a, b):
    """rigid transform taking mesh a's vertices onto b's (same vertex order) - 4x4, row-major"""
    A = np.c_[a.vertices, np.ones(len(a.vertices))]; X, *_ = np.linalg.lstsq(A, b.vertices, rcond=None)
    M = np.eye(4); M[:3, :4] = X.T
    return M.round(6).tolist()

meshes, items = {}, []
for n in meta['print1'] + meta['print2'] + meta['print3']:
    m = trimesh.load(f'{OUT}/{n}.stl'); meshes[n] = m
    g, col, op = PART_STYLE[n]
    items.append(dict(name=n, group=g, body=meta['body_of'][n] if meta['body_of'][n] != 'none' else 'tub', color=col, opacity=op,
                      kind='part' if g != 'bedonly' else 'bedonly', **pack(m)))
for n, d in meta['components'].items():
    m = trimesh.load(f'{OUT}/comp/{n}.stl')
    body = d['body'] if d['body'] != 'wheel' else ('tub' if 'rear' in n else 'shell')
    grp = ('wheels' if n.startswith('wheel') else 'water' if n.startswith(('tube_', 'nozzle')) else
           'arm_deployed' if n == 'refill_arm_deployed' else 'arm_stowed' if n == 'refill_arm_stowed' else 'components')
    items.append(dict(name=n, group=grp, body=body, color=comp_col(n), opacity=0.5 if grp == 'components' else 0.22, kind='comp',
                      mass=d['mass'], **pack(m)))
# v3 extras: spray cone, LiDAR scan plane, cable bundle through the hinge pass, intake hose outside (both arm positions)
def add_extra(name, group, body, color, op, mesh):
    items.append(dict(name=name, group=group, body=body, color=color, opacity=op, kind='extra', **pack(mesh)))
tip = np.array(C.NOZ['tip']); ang = math.radians(C.NOZ['angle']); dvec = np.array([math.cos(ang), 0, -math.sin(ang)])
Lc = 400.0; cone = trimesh.creation.cone(radius=Lc * math.tan(math.radians(4.0)), height=Lc, sections=24)   # base z=0, apex z=Lc
cone.apply_translation([0, 0, -Lc])                                                                          # apex at the origin
cone.apply_transform(trimesh.geometry.align_vectors([0, 0, -1.0], dvec)); cone.apply_translation(tip)
add_extra('spray cone (4 deg, 400 mm shown)', 'spray', 'shell', '#3ad0ff', 0.25, cone)
sp = trimesh.creation.cylinder(radius=260, height=0.6, sections=72); sp.apply_translation([C.LIDAR['cx'], 0, S['scan_plane']])
add_extra('LiDAR scan plane', 'scan', 'tub', '#ff3a3a', 0.12, sp)
ba = chk['cables']['bundle_mm2']; rb = math.sqrt(ba / 0.4 / math.pi) * 0.75
cb = trimesh.creation.cylinder(radius=rb, height=70, sections=24)
cb.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); cb.apply_translation([H['x'], 0, C.CABLE_PASS.centroid.y])
add_extra('cable bundle through the hinge pass', 'cables', 'tub', '#ffb000', 0.6, cb)
def tube_mesh(P, r=4.5):
    out = []
    for a, b in zip(P[:-1], P[1:]):
        L = np.linalg.norm(b - a)
        if L < 1e-6:
            continue
        c = trimesh.creation.cylinder(radius=r, height=L, sections=12)
        c.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (b - a) / L)); c.apply_translation((a + b) / 2); out.append(c)
    return trimesh.util.concatenate(out)
hose_outside = C.hose_outside
add_extra('intake hose outside (arm stowed)', 'arm_stowed', 'shell', '#00a8ff', 0.6, tube_mesh(hose_outside(C.ARM['stowed_deg'])))
add_extra('intake hose outside (arm deployed)', 'arm_deployed', 'shell', '#00a8ff', 0.6, tube_mesh(hose_outside(C.ARM['deployed_deg'])))
for n, d in meta['keepouts'].items():
    m = trimesh.load(f'{OUT}/comp/keepout_{n}.stl')
    items.append(dict(name='keep-out: ' + n, group='keepouts', body=d['body'], color='#ff2a2a', opacity=0.13, kind='keep', **pack(m)))

lay = C.print_layouts(meshes)
beds = {str(k): {n: fit_matrix(meshes[n], mm) for n, mm in v} for k, v in lay.items()}

S = meta['summary']; H = meta['hinge']
sweep = chk['hinge_sweep']
info = dict(
    hinge=dict(x=H['x'], z=H['z'], free=[sweep['free_from'], sweep['free_to']], stops=[sweep['nose_down_stop_deg'], sweep['nose_up_stop_deg']],
               samples=sweep['samples'], gap=chk['print1_min_gap_mm']),
    mass=chk['mass'], compare=chk['compare'], fill_v2=chk['fill_v2']['TOTAL'], overlaps=len(chk['overlaps']), outline=chk['outline'], curb=chk['curb'],
    lidar=chk['lidar'], cables=chk['cables'], arm=chk['arm'], spray=chk['spray'], tubes=chk['tubes'], wet=chk['wet_zone']['parts'],
    as5600=chk['as5600'], hard=chk.get('hard_fail', []),
    minclr=min(chk['clearance_mm'].values()),
    print1=chk['print1_extent'], print2=chk['print2_extent'], p2fits=chk['print2_fits'], scan=S['scan_plane'],
    length=S['length'], top=S['H'],
    dead=chk['hinge_gap_dead_cm3'], wheel=chk['wheel_to_body_mm'], p3=chk.get('print3_extent'))

three = open(f'{THREE_DIR}/three.module.min.js', 'rb').read()
orbit = open(f'{THREE_DIR}/OrbitControls.js', 'rb').read()
du = lambda b: 'data:text/javascript;base64,' + base64.b64encode(b).decode()
importmap = json.dumps({'imports': {'three': du(three), 'three/addons/OrbitControls.js': du(orbit)}})

HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover 1 Chassis v3</title>
<style>
:root{--bg:#f4f5f7;--panel:#ffffff;--ink:#1d2330;--mute:#5b6475;--line:#d9dde5;--acc:#d9622b;--ok:#1f8f4e;--bad:#c62828}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#15181e;--panel:#1d2129;--ink:#e7eaf0;--mute:#9aa3b5;--line:#2e3440;--acc:#ff8a4c;--ok:#4cc27e;--bad:#ff6b6b}}
:root[data-theme="dark"]{--bg:#15181e;--panel:#1d2129;--ink:#e7eaf0;--mute:#9aa3b5;--line:#2e3440;--acc:#ff8a4c;--ok:#4cc27e;--bad:#ff6b6b}
*{box-sizing:border-box}html,body{margin:0;height:100%;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
#app{display:grid;grid-template-columns:320px 1fr;height:100%}
#panel{background:var(--panel);border-right:1px solid var(--line);overflow:auto;padding:14px 16px}
#view{position:relative;min-height:300px}canvas{display:block}
h1{font-size:17px;margin:0 0 2px}h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute);margin:16px 0 6px}
.sub{color:var(--mute);font-size:12px}
.row{display:flex;align-items:center;gap:8px;margin:4px 0}.row label{flex:1}
input[type=range]{width:100%}
.btns{display:flex;flex-wrap:wrap;gap:6px}
button{border:1px solid var(--line);background:transparent;color:var(--ink);border-radius:6px;padding:5px 9px;cursor:pointer;font:inherit;font-size:13px}
button.on{background:var(--acc);border-color:var(--acc);color:#fff}
.sw{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:6px;vertical-align:-1px}
.big{font-size:22px;font-weight:600}
#hstate{font-weight:600}.ok{color:var(--ok)}.bad{color:var(--bad)}
table{border-collapse:collapse;width:100%;font-size:12.5px}td{padding:2px 0;border-bottom:1px solid var(--line)}td:last-child{text-align:right;font-variant-numeric:tabular-nums}
#tip{position:absolute;left:12px;bottom:10px;background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:4px 8px;font-size:12px;pointer-events:none;display:none}
#legend{max-height:none}
@media (max-width:760px){#app{grid-template-columns:1fr;grid-template-rows:55vh auto}#panel{order:2;border-right:0;border-top:1px solid var(--line)}#view{order:1}}
</style>
<script type="importmap">__IMPORTMAP__</script>
</head><body><div id="app">
<div id="panel">
  <h1>FPMS Rover 1 chassis v3</h1>
  <div class="sub">Preview for approval. Model frame: X forward, Y left, Z up, mm. Every component is a ghost at its real size.</div>

  <h2>View</h2>
  <div class="btns" id="modes"><button data-mode="asm" class="on">Assembled</button><button data-mode="1">Print 1 bed</button><button data-mode="2">Print 2 bed</button><button data-mode="3">Print 3 bed</button></div>
  <div class="btns" style="margin-top:6px"><button data-cam="iso">Iso</button><button data-cam="side">Side</button><button data-cam="top">Top</button><button data-cam="front">Front</button><button data-cam="rear">Rear</button></div>

  <h2>Hinge</h2>
  <div class="row"><label for="hinge">Front shell lift</label><span class="big" id="hval">0&deg;</span></div>
  <input id="hinge" type="range" min="0" max="20" step="0.25" value="0">
  <div class="sub">Status at this angle (computed sweep, 0.25&deg; steps): <span id="hstate"></span></div>
  <div class="sub" id="hnote"></div>

  <h2>Section plane</h2>
  <div class="btns" id="axes"><button data-ax="off" class="on">Off</button><button data-ax="x">X</button><button data-ax="y">Y</button><button data-ax="z">Z</button><button id="flip">Flip side</button></div>
  <input id="cut" type="range" min="-60" max="200" step="0.5" value="0" style="margin-top:6px">
  <div class="sub" id="cutval"></div>

  <h2>Show / hide</h2>
  <div id="toggles"></div>

  <h2>Computed checks</h2>
  <table id="checks"></table>

  <h2>Components</h2>
  <div id="legend" class="sub"></div>
</div>
<div id="view"><div id="tip"></div></div>
</div>
<script>window.MODEL = __MODEL__;</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
const M = window.MODEL, H = M.info.hinge;
const view = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.localClippingEnabled = true;
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const cam = new THREE.PerspectiveCamera(30, 1, 1, 5000); cam.up.set(0,0,1);
const ctl = new OrbitControls(cam, renderer.domElement); ctl.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x606070, 1.6));
const d1 = new THREE.DirectionalLight(0xffffff, 1.4); d1.position.set(300,-400,600); scene.add(d1);
const d2 = new THREE.DirectionalLight(0xffffff, 0.6); d2.position.set(-300,300,200); scene.add(d2);
function bg(){ scene.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue('--bg').trim()||'#f4f5f7'); }
bg(); matchMedia('(prefers-color-scheme: dark)').addEventListener('change', bg);

const clip = new THREE.Plane(new THREE.Vector3(-1,0,0), 0); let clipOn = false;
function dec(b64, T){ const s = atob(b64), u = new Uint8Array(s.length); for(let i=0;i<s.length;i++) u[i]=s.charCodeAt(i); return new T(u.buffer); }

// pivot for the front shell: rotation about the hinge axis (Y through X=H.x, Z=H.z); +deg = nose up
const asm = new THREE.Group(); scene.add(asm);
const pivot = new THREE.Group(); pivot.position.set(H.x, 0, H.z); asm.add(pivot);
const shellG = new THREE.Group(); shellG.position.set(-H.x, 0, -H.z); pivot.add(shellG);
const tubG = new THREE.Group(); asm.add(tubG);
const bedG = new THREE.Group(); scene.add(bedG);
const groups = {}; const objs = [];
for (const it of M.items){
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(dec(it.v, Float32Array), 3));
  g.setIndex(new THREE.BufferAttribute(dec(it.f, Uint32Array), 1)); g.computeVertexNormals();
  const ghost = it.opacity < 0.99;
  const mat = new THREE.MeshStandardMaterial({color: it.color, transparent: ghost, opacity: it.opacity, depthWrite: !ghost,
     roughness: .65, metalness: .05, side: THREE.DoubleSide, flatShading: it.kind==='part', clippingPlanes: [clip], clipShadows:true});
  const mesh = new THREE.Mesh(g, mat); mesh.userData = it; mesh.renderOrder = ghost ? 2 : 1;
  if (it.kind !== 'bedonly') (it.body === 'shell' ? shellG : tubG).add(mesh);
  (groups[it.group] ||= []).push(mesh); objs.push(mesh);
  if (it.kind === 'part' || it.kind === 'bedonly') {   // bed copies for the print layouts
    for (const k of ['1','2','3']) { const T = M.beds[k][it.name]; if (!T) continue;
      const b = new THREE.Mesh(g, new THREE.MeshStandardMaterial({color: it.color, roughness:.6, flatShading:true, side:THREE.DoubleSide}));
      b.matrixAutoUpdate = false; b.matrix.fromArray(T.flat()).transpose(); b.userData = {bed:k, name: it.name}; bedG.add(b); }
  }
}
// print bed plates
const plates = {};
for (const k of ['1','2','3']) {
  const p = new THREE.Mesh(new THREE.BoxGeometry(256,256,1), new THREE.MeshStandardMaterial({color:'#3a3d44', roughness:.9}));
  p.position.set(128,128,-0.55); p.userData={bed:k, plate:true}; bedG.add(p); plates[k]=p;
  const e = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(256,256,256)), new THREE.LineBasicMaterial({color:'#8a8f99'}));
  e.position.set(128,128,128); e.userData={bed:k, plate:true}; bedG.add(e);
}

// toggles
const TG = [['tub','Main tub'],['shell','Front shell'],['lids','Caps + hatches'],['decks','Decks + LiDAR riser'],['components','Components (ghosts)'],
  ['water','Water: tubes + nozzle'],['spray','Spray cone'],['arm_stowed','Refill arm stowed + hose'],['arm_deployed','Refill arm deployed + hose'],
  ['cables','Cable bundle (hinge pass)'],['scan','LiDAR scan plane'],['wheels','Wheels (Ø86.6, 4 mm gap)'],['keepouts','Keep-out zones (plugs, fan air)']];
const OFF = new Set(['keepouts','arm_deployed']);
const tg = document.getElementById('toggles');
for (const [k,lab] of TG){ const r = document.createElement('div'); r.className='row';
  r.innerHTML = `<input type="checkbox" id="t_${k}" ${OFF.has(k)?'':'checked'}><label for="t_${k}">${lab}</label>`; tg.appendChild(r);
  r.querySelector('input').addEventListener('change', e => { for (const m of groups[k]||[]) m.visible = e.target.checked; });
  if (OFF.has(k)) for (const m of groups[k]||[]) m.visible = false;
}
// legend
const lg = document.getElementById('legend'); const seen = new Set();
for (const it of M.items) if (it.kind==='comp' && !it.name.startsWith('wheel')) { const key = it.name.replace(/_(L|R|\d)$/,'');
  if (seen.has(key)) continue; seen.add(key);
  const s = document.createElement('div'); s.innerHTML = `<span class="sw" style="background:${it.color}"></span>${key.replace(/_/g,' ')}${it.mass?` <span style="opacity:.7">(${it.mass} g)</span>`:''}`; lg.appendChild(s); }
// checks table
const I = M.info, tb = document.getElementById('checks');
const rows = [
  ['Overlaps (must be 0)', `<span class="${I.overlaps?'bad':'ok'}">${I.overlaps}</span>`],
  ['Hinge free range', `${H.free[0]}&deg; .. ${H.free[1]}&deg;`],
  ['Stops (0&deg; catch / 20&deg; stop)', `${H.stops[0]}&deg; / ${H.stops[1]}&deg;`],
  ['Print-in-place gap', `${H.gap} mm`],
  ['Front axle load', `${I.mass.front_axle_pct}% (target 70)`],
  ['Mass (estimate)', `${I.mass.total_g} g`],
  ['Print 1 size', I.print1.join(' x ')+' mm'],
  ['Print 2 fits bed', `<span class="${I.p2fits?'ok':'bad'}">${I.p2fits?'yes':'no'}</span>`],
  ['Smallest pocket clearance', `<span class="${I.minclr>=0.4?'ok':'bad'}">${I.minclr} mm</span> (min 0.4)`],
  ['Hard checks', `<span class="${I.hard.length?'bad':'ok'}">${I.hard.length?I.hard.length+' failing':'all pass'}</span>`],
  ['Top of body v2 / v3', `${I.compare.v2.top} / ${I.compare.v3.top} mm`],
  ['Length v2 / v3', `${I.compare.v2.length} / ${I.compare.v3.length} mm`],
  ['Fill v2 / v3', `${I.compare.v2.fill}% / ${I.compare.v3.fill}%`],
  ['LiDAR margin over shell at 20&deg;', `<span class="${I.lidar.margin_20deg>=I.lidar.required?'ok':'bad'}">${I.lidar.margin_20deg} mm</span> (need ${I.lidar.required})`],
  ['Spray cone', `${I.spray.angle_down}&deg; down, ${Object.keys(I.spray.blocked_by).length?'<span class=bad>blocked</span>':'<span class=ok>clear</span>'}`],
  ['Cable pass fill', `${I.cables.fill_pct}% (limit 40), ${I.cables.length_change_20deg_mm} mm at 20&deg;`],
  ['Arm deployed lowest Z', `${I.arm['refill_arm_deployed@0'].lowest_z} mm`],
  ['Arm to wheel (min)', `${Math.min(...Object.values(I.arm).map(v=>Math.min(v.front_wheel_gap,v.rear_wheel_gap)))} mm`],
  ['Wheel gap (min)', `${Math.min(...Object.values(I.wheel))} mm (need 4)`],
  ['AS5600 magnet to chip', `${I.as5600.magnet_to_chip_gap_mm} mm`],
  ['Curb (Ø86.6 wheel)', I.curb['wheel_d86.6'].replace('wheel touches first for every curb up to the axle','wheel first, all heights')],
  ['Hinge-gap dead volume', `${I.dead.v0} &rarr; ${I.dead.v3} cm&sup3;`],
  ['LiDAR scan plane', `Z ${I.scan}`],
  ['Top of body (one line)', `${I.top} mm`],
];
tb.innerHTML = rows.map(r=>`<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');

// hinge
const hs = document.getElementById('hinge'), hv = document.getElementById('hval'), hst = document.getElementById('hstate');
document.getElementById('hnote').textContent = `Stops built in: shell rests on its 0° catch (touches at ${H.stops[0]}° when lifted by the tub) and lands flat on the raked tub face at ${H.stops[1]}°.`;
function setHinge(a){ pivot.rotation.y = -a*Math.PI/180; hv.innerHTML = a.toFixed(2).replace(/\.?0+$/,'')+'&deg;';
  const s = H.samples.reduce((b,x)=> Math.abs(x[0]-a) < Math.abs(b[0]-a) ? x : b);
  const free = a >= H.free[0] && a <= H.free[1];
  hst.className = free ? 'ok' : 'bad'; hst.textContent = free ? 'free, no contact' : `contact (${s[1]} mm³)`; }
hs.addEventListener('input', () => setHinge(+hs.value)); setHinge(0);

// section
let ax = 'off', flip = 1; const cut = document.getElementById('cut'), cv = document.getElementById('cutval');
const AX = {x:[1,0,0], y:[0,1,0], z:[0,0,1]}, RNG = {x:[-60,200,48.5], y:[-80,80,0], z:[20,190,70]};
function setCut(){ if (ax==='off'){ clip.set(new THREE.Vector3(0,0,-1), 1e5); cv.textContent=''; return; }
  const n = AX[ax]; const v = +cut.value; clip.normal.set(-n[0]*flip, -n[1]*flip, -n[2]*flip); clip.constant = v*flip;
  cv.textContent = `${ax.toUpperCase()} = ${v} mm (showing the ${flip>0?'lower':'upper'} side)`; }
document.getElementById('axes').addEventListener('click', e => { const b = e.target.closest('button'); if (!b || !b.dataset.ax) return;
  ax = b.dataset.ax; for (const x of document.querySelectorAll('#axes button[data-ax]')) x.classList.toggle('on', x===b);
  if (ax!=='off'){ const r = RNG[ax]; cut.min=r[0]; cut.max=r[1]; cut.value=r[2]; } setCut(); });
document.getElementById('flip').addEventListener('click', ()=>{ flip*=-1; setCut(); });
cut.addEventListener('input', setCut); setCut();

// modes + cameras
let mode = 'asm';
function setMode(m){ mode = m; asm.visible = m==='asm'; bedG.visible = m!=='asm';
  for (const o of bedG.children) o.visible = o.userData.bed === m;  // print 3: riser, arm, spacers
  for (const b of document.querySelectorAll('#modes button')) b.classList.toggle('on', b.dataset.mode===m); camTo(m==='asm'?'iso':'bed'); }
document.getElementById('modes').addEventListener('click', e => { const b = e.target.closest('button'); if (b) setMode(b.dataset.mode); });
function camTo(k){ const c = mode==='asm' ? new THREE.Vector3(70,0,95) : new THREE.Vector3(128,128,30); let d;
  const R = mode==='asm' ? 720 : 760;
  d = {iso:[1,-1.15,0.85], side:[0.001,-1,0.02], top:[0.001,0.001,1], front:[1,0.001,0.05], rear:[-1,0.001,0.05], bed:[0.25,-0.9,0.9]}[k] || [1,-1.15,.85];
  const v = new THREE.Vector3(...d).normalize().multiplyScalar(R); cam.position.copy(c).add(v); ctl.target.copy(c); ctl.update(); }
document.querySelectorAll('[data-cam]').forEach(b => b.addEventListener('click', ()=> camTo(b.dataset.cam)));

// hover names
const ray = new THREE.Raycaster(), mp = new THREE.Vector2(), tip = document.getElementById('tip');
renderer.domElement.addEventListener('pointermove', e => { const r = renderer.domElement.getBoundingClientRect();
  mp.set(((e.clientX-r.left)/r.width)*2-1, -((e.clientY-r.top)/r.height)*2+1); ray.setFromCamera(mp, cam);
  const pool = mode==='asm' ? objs.filter(o=>o.visible) : bedG.children.filter(o=>o.visible && !o.userData.plate);
  const h = ray.intersectObjects(pool, false).filter(x => !clipOn || true)[0];
  if (h){ tip.style.display='block'; tip.textContent = (h.object.userData.name||'').replace(/_/g,' '); } else tip.style.display='none'; });

function resize(){ const w = view.clientWidth, h = view.clientHeight || 400; renderer.setSize(w,h); cam.aspect = w/h; cam.updateProjectionMatrix(); }
addEventListener('resize', resize); resize(); setMode('asm');
// deep links for sharing a view: #h=20&mode=2&cam=side&cut=y:40
{ const q = new URLSearchParams(location.hash.slice(1));
  if (q.get('mode')) setMode(q.get('mode'));
  if (q.get('h')) { hs.value = q.get('h'); setHinge(+hs.value); }
  if (q.get('cam')) camTo(q.get('cam'));
  if (q.get('keep')) document.getElementById('t_keepouts').click();
  if (q.get('cut')) { const [a, v] = q.get('cut').split(':'); document.querySelector(`#axes [data-ax="${a}"]`).click(); cut.value = v; setCut(); } }
(function loop(){ requestAnimationFrame(loop); ctl.update(); renderer.render(scene, cam); })();
</script></body></html>'''

model = dict(items=items, beds=beds, info=info)
html = HTML.replace('__IMPORTMAP__', importmap).replace('__MODEL__', json.dumps(model, separators=(',', ':')))
open('preview_v3.html', 'w').write(html)
print('preview_v3.html', round(len(html) / 1e6, 2), 'MB,', len(items), 'meshes,', sum(i['nf'] for i in items), 'triangles')
