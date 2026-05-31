#!/bin/bash
# Install udev rules and enable GPIO UART for the Hiwonder ROS Robot Controller
# (STM32 co-processor) on Raspberry Pi 5.
#
# The expansion board connects via the 40-pin GPIO header UART, which appears
# as /dev/ttyAMA on Pi5.  This script:
#   1. Installs the udev rule that creates the /dev/rrc symlink → ttyAMA
#   2. Adds the dtoverlay=uart0-pi5 line to /boot/firmware/config.txt if needed
#   3. Adds the current user to the dialout group
#
# Run once after cloning / building the workspace (requires sudo):
#   sudo bash etc/udev/install_udev_rules.sh

set -e

SCRIPT_DIR="$(dirname "$0")"

# ── 1. Install udev rules ──────────────────────────────────────────────────────

# STM32 co-processor (RRC board) via GPIO UART → /dev/rrc
RULES_SRC="$SCRIPT_DIR/99-ttyAMA-rrc.rules"
RULES_DEST="/etc/udev/rules.d/99-ttyAMA-rrc.rules"
echo "Installing udev rule: $RULES_SRC -> $RULES_DEST"
sudo cp "$RULES_SRC" "$RULES_DEST"

# RPLidar USB-to-serial (CP2102) → /dev/rplidar
RPLIDAR_SRC="$SCRIPT_DIR/99-rplidar.rules"
RPLIDAR_DEST="/etc/udev/rules.d/99-rplidar.rules"
echo "Installing udev rule: $RPLIDAR_SRC -> $RPLIDAR_DEST"
sudo cp "$RPLIDAR_SRC" "$RPLIDAR_DEST"

# V4L2 camera devices → video group with rw access
VIDEO_SRC="$SCRIPT_DIR/99-video.rules"
VIDEO_DEST="/etc/udev/rules.d/99-video.rules"
echo "Installing udev rule: $VIDEO_SRC -> $VIDEO_DEST"
sudo cp "$VIDEO_SRC" "$VIDEO_DEST"

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# ── 2. Enable UART on GPIO pins in boot config ────────────────────────────────
# On Pi 5 the primary GPIO UART is uart0, enabled with dtoverlay=uart0-pi5.
# This maps it to /dev/ttyAMA10.

CONFIG="/boot/firmware/config.txt"

if [ ! -f "$CONFIG" ]; then
    echo "WARNING: $CONFIG not found – skipping boot config update."
    echo "         Add 'dtoverlay=uart0-pi5' manually to your Pi boot config."
else
    if grep -q "dtoverlay=uart0-pi5" "$CONFIG"; then
        echo "dtoverlay=uart0-pi5 already present in $CONFIG – skipping."
    else
        echo "Adding dtoverlay=uart0-pi5 to $CONFIG ..."
        echo "" | sudo tee -a "$CONFIG" > /dev/null
        echo "# Enable GPIO UART for ROS Robot Controller (STM32) expansion board" | sudo tee -a "$CONFIG" > /dev/null
        echo "dtoverlay=uart0-pi5" | sudo tee -a "$CONFIG" > /dev/null
        echo "Done. A REBOOT is required for the UART overlay to take effect."
        NEED_REBOOT=1
    fi
fi

# ── 3. Add user to required groups ────────────────────────────────────────────

REAL_USER="${SUDO_USER:-$USER}"

# dialout – required for /dev/rrc (STM32) and /dev/rplidar (RPLidar)
if ! groups "$REAL_USER" | grep -q dialout; then
    echo "Adding $REAL_USER to dialout group..."
    sudo usermod -aG dialout "$REAL_USER"
    echo "Group change requires logout/login (or reboot) to take effect."
    NEED_REBOOT=1
else
    echo "$REAL_USER is already in dialout group."
fi

# video – required for /dev/video* (USB camera via v4l2_camera_node)
if ! groups "$REAL_USER" | grep -q video; then
    echo "Adding $REAL_USER to video group..."
    sudo usermod -aG video "$REAL_USER"
    echo "Group change requires logout/login (or reboot) to take effect."
    NEED_REBOOT=1
else
    echo "$REAL_USER is already in video group."
fi

# ── Summary ───────────────────────────────────────────────────────────────────

echo ""
if [ "${NEED_REBOOT:-0}" = "1" ]; then
    echo "======================================================="
    echo "  REBOOT REQUIRED for all changes to take effect."
    echo "  After reboot, verify with:"
    echo "    ls -la /dev/rrc       (STM32 board)"
    echo "    ls -la /dev/rplidar   (RPLidar, when USB plugged in)"
    echo "    ls -la /dev/video0    (USB camera)"
    echo "======================================================="
else
    echo "All done. Verify the devices with:"
    echo "  ls -la /dev/rrc       (STM32 board)"
    echo "  ls -la /dev/rplidar   (RPLidar, when USB plugged in)"
    echo "  ls -la /dev/video0    (USB camera)"
fi
