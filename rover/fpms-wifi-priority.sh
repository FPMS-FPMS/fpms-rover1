#!/bin/bash
# Enforce FPMS_Net priority on the RUNNING supplicant.
#
# WHY THIS EXISTS ON TOP OF THE ExecStartPre INJECTOR
# The ExecStartPre edits /run/netplan/wpa-wlan0.conf, and that works - but
# netplan REGENERATES that file, so the priority lines disappear from disk
# again. Whether wpa_supplicant read the injected version is then a race:
# observed on 2026-09-10 connected to FPMS_Net with ZERO priority lines on
# disk, i.e. it got the right network by luck, because FPMS_Net happens to be
# network 0.
#
# wpa_cli talks to the live supplicant, so this does not care what the file
# says or when it was rewritten. Idempotent: safe to run at boot and by hand.
set -u
IFACE=wlan0
PREFER=FPMS_Net

for _ in $(seq 1 60); do
    [ -S "/run/wpa_supplicant/$IFACE" ] && break
    sleep 1
done
[ -S "/run/wpa_supplicant/$IFACE" ] || { echo "no ctrl interface for $IFACE"; exit 0; }

nets=$(wpa_cli -i "$IFACE" list_networks 2>/dev/null | tail -n +2)
[ -n "$nets" ] || { echo "no networks configured"; exit 0; }

while IFS=$'\t' read -r id ssid _rest; do
    [ -n "${id:-}" ] || continue
    if [ "$ssid" = "$PREFER" ]; then
        wpa_cli -i "$IFACE" set_network "$id" priority 10 >/dev/null 2>&1
        # PMF optional: the ground station AP advertises SAE alongside WPA-PSK.
        wpa_cli -i "$IFACE" set_network "$id" ieee80211w 1 >/dev/null 2>&1
        echo "priority 10 -> network $id ($ssid)"
    else
        wpa_cli -i "$IFACE" set_network "$id" priority 1 >/dev/null 2>&1
        echo "priority  1 -> network $id ($ssid)"
    fi
done <<< "$nets"

# Do NOT reassociate if we are already on the preferred network - that would
# drop a working link on every boot for no reason.
cur=$(wpa_cli -i "$IFACE" status 2>/dev/null | sed -n 's/^ssid=//p')
if [ "$cur" != "$PREFER" ]; then
    echo "on '$cur', preferring $PREFER - reassociating"
    wpa_cli -i "$IFACE" reassociate >/dev/null 2>&1
else
    echo "already on $PREFER - not touching the link"
fi
exit 0
