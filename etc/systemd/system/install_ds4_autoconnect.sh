#!/bin/bash
# install_ds4_autoconnect.sh
# Install the DualShock 4 Bluetooth auto-reconnect service for Ubuntu 24.04
# on Raspberry Pi 5.
#
# What this does:
#   1. Copies ds4-autoconnect.sh to /usr/local/bin/ and makes it executable.
#   2. Installs the ds4-autoconnect.service systemd unit.
#   3. Installs the 99-ds4-bluetooth.rules udev rule.
#   4. Enables the service to run at boot.
#   5. Configures BlueZ to auto-connect trusted devices on power-on.
#
# Prerequisites:
#   - DS4 already paired and trusted via bluetoothctl:
#       bluetoothctl pair <MAC>
#       bluetoothctl trust <MAC>
#
# Run once (requires sudo):
#   sudo bash etc/systemd/system/install_ds4_autoconnect.sh
#
# To manually trigger a reconnect at any time:
#   sudo systemctl start ds4-autoconnect.service
#   # or directly:
#   sudo /usr/local/bin/ds4-autoconnect.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
UDEV_DIR="$(cd "$SCRIPT_DIR/../../udev" && pwd)"

echo "=== DualShock 4 Auto-Connect Installer ==="
echo ""

# ── 1. Install the reconnect script ──────────────────────────────────────────
echo "Installing /usr/local/bin/ds4-autoconnect.sh ..."
sudo cp "$SCRIPT_DIR/ds4-autoconnect.sh" /usr/local/bin/ds4-autoconnect.sh
sudo chmod +x /usr/local/bin/ds4-autoconnect.sh

# ── 2. Install the systemd service ───────────────────────────────────────────
echo "Installing ds4-autoconnect.service ..."
sudo cp "$SCRIPT_DIR/ds4-autoconnect.service" /etc/systemd/system/ds4-autoconnect.service
sudo systemctl daemon-reload
sudo systemctl enable ds4-autoconnect.service
echo "  Service enabled (will run at next boot)."

# ── 3. Install the udev rule ─────────────────────────────────────────────────
echo "Installing 99-ds4-bluetooth.rules ..."
sudo cp "$UDEV_DIR/99-ds4-bluetooth.rules" /etc/udev/rules.d/99-ds4-bluetooth.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  udev rules reloaded."

# ── 4. Configure BlueZ for auto-connect on power-on ──────────────────────────
# /etc/bluetooth/main.conf:  FastConnectable=true ensures the adapter stays
# in connectable mode long enough for the DS4 to find it after boot.
BT_CONF="/etc/bluetooth/main.conf"
if [ -f "$BT_CONF" ]; then
    # Enable FastConnectable if not already set
    if grep -q "^FastConnectable" "$BT_CONF"; then
        sudo sed -i 's/^FastConnectable.*/FastConnectable = true/' "$BT_CONF"
    elif grep -q "^#FastConnectable" "$BT_CONF"; then
        sudo sed -i 's/^#FastConnectable.*/FastConnectable = true/' "$BT_CONF"
    else
        echo "" | sudo tee -a "$BT_CONF" > /dev/null
        echo "FastConnectable = true" | sudo tee -a "$BT_CONF" > /dev/null
    fi
    echo "  BlueZ FastConnectable=true set in $BT_CONF."

    # JustWorksRepairing: ensures pairing is re-used without user prompts
    if grep -q "^JustWorksRepairing" "$BT_CONF"; then
        sudo sed -i 's/^JustWorksRepairing.*/JustWorksRepairing = always/' "$BT_CONF"
    elif grep -q "^#JustWorksRepairing" "$BT_CONF"; then
        sudo sed -i 's/^#JustWorksRepairing.*/JustWorksRepairing = always/' "$BT_CONF"
    else
        echo "JustWorksRepairing = always" | sudo tee -a "$BT_CONF" > /dev/null
    fi
    echo "  BlueZ JustWorksRepairing=always set in $BT_CONF."
else
    echo "  WARNING: $BT_CONF not found – skipping BlueZ config."
fi

# ── 5. Restart bluetooth service to pick up config changes ───────────────────
echo "Restarting bluetooth.service ..."
sudo systemctl restart bluetooth.service
sleep 2

# ── 6. Run once now (don't wait for next boot) ────────────────────────────────
echo ""
echo "Running ds4-autoconnect now (press PS button on DS4 if it's powered on)..."
sudo systemctl start ds4-autoconnect.service || true

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "======================================================="
echo "  DS4 Auto-Connect installed successfully!"
echo ""
echo "  How to use:"
echo "  - On boot: press the PS button on the DS4 within ~15 s"
echo "    of the Pi finishing boot. It will auto-connect."
echo "  - To force reconnect manually:"
echo "    sudo systemctl start ds4-autoconnect.service"
echo "  - Check status / logs:"
echo "    systemctl status ds4-autoconnect.service"
echo "    journalctl -t ds4-autoconnect -f"
echo ""
echo "  If the DS4 has never been paired, pair it first:"
echo "    bluetoothctl power on"
echo "    bluetoothctl agent on"
echo "    bluetoothctl scan on"
echo "    # Hold PS + Share on DS4 for 5 s (fast-blink = pairing mode)"
echo "    bluetoothctl pair <MAC>"
echo "    bluetoothctl trust <MAC>"
echo "    bluetoothctl connect <MAC>"
echo "======================================================="
