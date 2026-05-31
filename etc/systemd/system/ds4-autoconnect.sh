#!/usr/bin/env bash
# ds4-autoconnect.sh
# Reconnect all paired DualShock 4 controllers via BlueZ at boot.
#
# How it works:
#   1. Asks bluetoothctl to list all paired devices.
#   2. Filters for Sony DS4 controllers by vendor name.
#   3. Issues a non-blocking "connect <MAC>" for each one found.
#
# Install to /usr/local/bin/ds4-autoconnect.sh (see install_bluetooth.sh)
# and enable the ds4-autoconnect.service unit.
#
# The DS4 must be pressed (PS button) to enter connectable mode.
# This script handles the case where the controller IS powered on at boot
# or is powered on after boot (the udev rule 99-ds4-bluetooth.rules covers
# the hot-plug case when the BT adapter re-initialises).

set -euo pipefail

log() { echo "[ds4-autoconnect] $*" | systemd-cat -t ds4-autoconnect -p info; echo "[ds4-autoconnect] $*"; }
warn() { echo "[ds4-autoconnect] $*" | systemd-cat -t ds4-autoconnect -p warning; echo "[ds4-autoconnect] WARNING: $*"; }

# ── Ensure the BT adapter is powered on ──────────────────────────────────────
log "Powering on bluetooth adapter..."
bluetoothctl power on || warn "Could not power on adapter (may already be on)"

# ── Collect paired DS4 MACs ──────────────────────────────────────────────────
# bluetoothctl devices outputs lines like:
#   Device AA:BB:CC:DD:EE:FF Wireless Controller
# The DS4 reports itself as "Wireless Controller" (Sony vendor).
# We also match common alternate names in case of custom pairings.

log "Scanning paired devices for DualShock 4 controllers..."

DS4_MACS=()
while IFS= read -r line; do
    mac=$(echo "$line" | awk '{print $2}')
    name=$(echo "$line" | cut -d' ' -f3-)

    # Match DS4 device names (Wireless Controller is the standard DS4 name)
    if echo "$name" | grep -qiE "Wireless Controller|DualShock|DS4"; then
        DS4_MACS+=("$mac")
        log "Found DS4: $name ($mac)"
    fi
done < <(bluetoothctl devices Paired 2>/dev/null || bluetoothctl devices 2>/dev/null)

if [ ${#DS4_MACS[@]} -eq 0 ]; then
    warn "No paired DualShock 4 controllers found. Pair one first with:"
    warn "  bluetoothctl"
    warn "  [bluetooth]# power on"
    warn "  [bluetooth]# agent on"
    warn "  [bluetooth]# scan on"
    warn "  # Press PS button on DS4 for 5 s to enter pairing mode"
    warn "  [bluetooth]# pair <MAC>"
    warn "  [bluetooth]# trust <MAC>"
    warn "  [bluetooth]# connect <MAC>"
    exit 0
fi

# ── Connect each DS4 ─────────────────────────────────────────────────────────
for mac in "${DS4_MACS[@]}"; do
    log "Attempting to connect $mac ..."
    # bluetoothctl connect can take a few seconds; give it up to 10 s.
    if timeout 10 bluetoothctl connect "$mac" 2>&1 | grep -q "Connection successful"; then
        log "Connected $mac successfully."
    else
        warn "Could not connect $mac (controller may be off – will connect when powered on)."
    fi
done

log "Done."
