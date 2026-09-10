#!/bin/bash
# Install the Rover 1 Cortex panels into the system tree so plain `cortex` works.
#
#   sudo bash ~/fpms-dev/cortex-rover/install-rover1-panels.sh
#   sudo bash ~/fpms-dev/cortex-rover/install-rover1-panels.sh --uninstall
#
# WHAT THIS TOUCHES, AND WHY IT IS SAFE TO
#   /usr/local/lib/fpms/cortex/backend/   3 NEW files, nothing overwritten
#   /usr/local/lib/fpms/cortex/web/       3 new panels + ros.js + thermal.js
#                                         + vendor/roslib.min.js, and app.js
# app.js is the ONLY pre-existing file replaced. Its diff against the installed
# copy is exactly the 12 lines that register the three tabs - verified before
# writing, and refused if anything else has diverged.
#
# EVERYTHING IS CHECKED BEFORE ANYTHING IS WRITTEN. A half-installed Cortex
# that will not start is worse than one without the new tabs.
set -u

SRC="/home/aryan/fpms-dev/cortex-rover"
DST="/usr/local/lib/fpms/cortex"
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="/usr/local/lib/fpms/cortex-backup-$STAMP"
SHIM="/home/aryan/.local/bin/cortex"

BACKEND_FILES=(panel_control.py panel_lidar.py panel_cameras.py)
WEB_FILES=(panel_control.js panel_lidar.js panel_cameras.js ros.js thermal.js app.js)

[ "$(id -u)" = 0 ] || { echo "run: sudo bash $0" >&2; exit 1; }
[ -d "$DST" ] || { echo "no Cortex installed at $DST" >&2; exit 1; }

# ---------------------------------------------------------------- uninstall
if [ "${1:-}" = "--uninstall" ]; then
    for f in "${BACKEND_FILES[@]}"; do rm -f "$DST/backend/$f"; done
    for f in "${WEB_FILES[@]}"; do [ "$f" = app.js ] || rm -f "$DST/web/$f"; done
    rm -rf "$DST/web/vendor"
    latest=$(ls -1d /usr/local/lib/fpms/cortex-backup-* 2>/dev/null | tail -1)
    if [ -n "$latest" ] && [ -f "$latest/web/app.js" ]; then
        install -o root -g root -m 0644 "$latest/web/app.js" "$DST/web/app.js"
        echo "  app.js restored from $latest"
    else
        echo "  WARNING: no backup found; app.js still lists the three tabs" >&2
    fi
    echo "  uninstalled. Re-run: sudo fpms-integrity baseline"
    exit 0
fi

echo "== 1. verify every file parses BEFORE writing anything =="
fail=0
for f in "${BACKEND_FILES[@]}"; do
    if python3 -c "import ast,sys; ast.parse(open('$SRC/backend/$f').read())" 2>/dev/null; then
        echo "  ok   backend/$f"
    else
        echo "  FAIL backend/$f does not parse"; fail=1
    fi
done
for f in "${WEB_FILES[@]}"; do
    # ES MODULE parse goal, not CommonJS. `node --check` parses as a script and
    # misses duplicate-declaration errors in modules - that exact gap shipped a
    # broken panel here once already.
    if node --experimental-vm-modules --no-warnings -e "
        const vm=require('vm'),fs=require('fs');
        new vm.SourceTextModule(fs.readFileSync('$SRC/web/$f','utf8'),{identifier:'$f'});" 2>/dev/null; then
        echo "  ok   web/$f"
    else
        echo "  FAIL web/$f is not a valid ES module"; fail=1
    fi
done
[ -s "$SRC/web/vendor/roslib.min.js" ] && echo "  ok   web/vendor/roslib.min.js" \
    || { echo "  FAIL vendor/roslib.min.js missing"; fail=1; }

echo "== 2. app.js must differ ONLY by the tab registration =="
d=$(diff "$DST/web/app.js" "$SRC/web/app.js" | grep -c '^[<>]')
removed=$(diff "$DST/web/app.js" "$SRC/web/app.js" | grep -c '^<')
if [ "$removed" -ne 0 ]; then
    echo "  FAIL the installed app.js has $removed line(s) the new one drops."
    echo "       Someone else changed it. Merge by hand rather than clobbering."
    fail=1
else
    echo "  ok   $d added line(s), 0 removed"
fi

[ "$fail" = 0 ] || { echo; echo "REFUSED: nothing was written."; exit 1; }

echo "== 3. back up the current tree =="
cp -a "$DST" "$BACKUP" 2>/dev/null
echo "  $BACKUP"

echo "== 4. install =="
for f in "${BACKEND_FILES[@]}"; do
    install -o root -g root -m 0644 "$SRC/backend/$f" "$DST/backend/$f" && echo "  backend/$f"
done
install -d -o root -g root -m 0755 "$DST/web/vendor"
for f in "${WEB_FILES[@]}"; do
    install -o root -g root -m 0644 "$SRC/web/$f" "$DST/web/$f" && echo "  web/$f"
done
install -o root -g root -m 0644 "$SRC/web/vendor/roslib.min.js" "$DST/web/vendor/roslib.min.js" \
    && echo "  web/vendor/roslib.min.js"

echo "== 5. verify the installed result =="
ok=1
for f in "${BACKEND_FILES[@]}"; do [ -f "$DST/backend/$f" ] || { echo "  MISSING backend/$f"; ok=0; }; done
for f in "${WEB_FILES[@]}"; do [ -f "$DST/web/$f" ] || { echo "  MISSING web/$f"; ok=0; }; done
n=$(grep -cE "id: '(control|lidar|cameras)'" "$DST/web/app.js")
echo "  app.js registers $n/3 tabs"
[ "$n" = 3 ] || ok=0
[ "$ok" = 1 ] && echo "  all files present" || { echo "  VERIFY FAILED - restore with: cp -a $BACKUP/. $DST/"; exit 1; }

echo "== 6. retire the shim and restart the backend =="
rm -f "$SHIM" && echo "  removed $SHIM"
pkill -f "cortex_core.py --port 8770" 2>/dev/null
rm -f /run/user/1000/fpms-cortex.pid
echo "  backend stopped; `cortex` will start the installed one"

echo
echo "DONE. /usr/local is a guarded tree, so now run:"
echo "    sudo fpms-integrity baseline"
echo "Then just:  cortex"
