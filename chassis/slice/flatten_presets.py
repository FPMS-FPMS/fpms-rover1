"""Resolve Bambu Studio system presets (which 'inherit' from base files) into flat JSONs the CLI can load.
usage: python flatten_presets.py <profiles/BBL dir> <out dir>"""
import json, os, sys
BBL, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
index = {}
for sub in ('machine', 'process', 'filament'):
    for f in os.listdir(os.path.join(BBL, sub)):
        if f.endswith('.json'):
            try:
                d = json.load(open(os.path.join(BBL, sub, f)))
            except Exception:
                continue
            if 'name' in d:
                index[(sub, d['name'])] = d

def flat(sub, name):
    d = dict(index[(sub, name)])
    parent = d.get('inherits')
    if parent:
        base = flat(sub, parent)
        base.update(d)
        d = base
    return d

WANT = [('machine', 'Bambu Lab P1S 0.4 nozzle'), ('process', '0.20mm Strength @BBL X1C'), ('filament', 'Bambu PETG Basic @BBL X1C')]
for sub, name in WANT:
    d = flat(sub, name)
    d.pop('inherits', None); d['from'] = 'system'
    if sub == 'process':
        d['compatible_printers'] = ['Bambu Lab P1S 0.4 nozzle']
    json.dump(d, open(os.path.join(OUT, f'{sub}.json'), 'w'), indent=1)
    print(sub, name, len(d), 'keys')
