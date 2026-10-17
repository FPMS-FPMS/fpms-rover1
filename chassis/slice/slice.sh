#!/bin/sh
# Slice one STL for a Bambu Lab P1S (0.4 nozzle), Bambu PETG Basic, 0.20 mm Strength, NO supports.
# usage: slice/slice.sh <model.stl> <outdir>   (run from chassis/)
set -e
M="$1"; O="$2"; mkdir -p "$O"
D=$(cd "$(dirname "$0")" && pwd)
flatpak run --filesystem="$(pwd)" --command=bambu-studio com.bambulab.BambuStudio \
  --debug 2 \
  --load-settings "$D/presets/machine.json;$D/presets/process.json" \
  --load-filaments "$D/presets/filament.json" \
  --slice 0 --export-3mf "$O/$(basename "$M" .stl).gcode.3mf" --outputdir "$O" \
  "$M"
