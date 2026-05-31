#!/usr/bin/env bash

set -e

#
# TurboPi ROS2 Jazzy setup for Ubuntu 24.04 on Raspberry Pi 5
#
# Features:
# - Safe to re-run
# - Uses sudo only where needed
# - Creates workspace in the REAL user home, not /root
# - Installs ROS 2 Jazzy + common tools
# - Clones turbopi_ros repo
# - Builds workspace
# - Adds ROS sourcing to ~/.bashrc only once
#

############################
# Detect real user/home
############################

if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(eval echo "~$SUDO_USER")
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

WS_DIR="$REAL_HOME/ros2_ws"
SRC_DIR="$WS_DIR/src"

echo "Using user: $REAL_USER"
echo "Using home: $REAL_HOME"

############################
# ROS repo setup
############################

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "Adding ROS2 repository..."

    sudo apt-get update
    sudo apt-get install -y curl gnupg2 software-properties-common

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

############################
# Install packages
############################

echo "Installing ROS2 Jazzy and dependencies..."

sudo apt-get update

sudo apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    build-essential \
    cmake \
    g++ \
    python3-pip \
    i2c-tools \
    libi2c-dev \
    libasio-dev \
    libtinyxml2-dev \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2controlcli \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-position-controllers \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-tools \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rplidar-ros \
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-mecanum-drive-controller \
    ros-jazzy-joy-linux \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-v4l2-camera \
    ros-jazzy-camera-info-manager \
    ros-jazzy-image-transport \
    ros-jazzy-controller-manager-msgs

############################
# rosdep init
############################

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

sudo -u "$REAL_USER" rosdep update || true

############################
# Workspace creation
############################

if [ ! -d "$WS_DIR" ]; then
    sudo -u "$REAL_USER" mkdir -p "$WS_DIR"
fi

if [ ! -d "$SRC_DIR" ]; then
    sudo -u "$REAL_USER" mkdir -p "$SRC_DIR"
fi

sudo chown -R "$REAL_USER":"$REAL_USER" "$WS_DIR"

############################
# Clone TurboPi repo
############################

if [ ! -d "$SRC_DIR/turbopi_ros" ]; then
    echo "Cloning turbopi_ros..."
    sudo -u "$REAL_USER" git clone https://github.com/wltjr/turbopi_ros "$SRC_DIR/turbopi_ros"
else
    echo "turbopi_ros already cloned"
fi

############################
# Clone Pplidar_ros repo
############################
if [ ! -d "$SRC_DIR/rplidar_ros" ]; then
    echo "Cloning rplidar_ros..."
    sudo -u "$REAL_USER" git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git "$SRC_DIR/rplidar_ros"
else
    echo "rplidar_ros already cloned"
fi

############################
# Source ROS in bashrc
############################

BASHRC="$REAL_HOME/.bashrc"

if ! grep -q "/opt/ros/jazzy/setup.bash" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS2 Jazzy" >> "$BASHRC"
    echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
fi

if ! grep -q "$WS_DIR/install/setup.bash" "$BASHRC"; then
    echo "if [ -f $WS_DIR/install/setup.bash ]; then" >> "$BASHRC"
    echo "    source $WS_DIR/install/setup.bash" >> "$BASHRC"
    echo "fi" >> "$BASHRC"
fi

############################
# Build workspace
############################

echo "Building workspace..."

cd "$WS_DIR"

sudo -u "$REAL_USER" bash -c "
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
"

############################
# Add current user to i2c and dialout access
############################

sudo usermod -aG i2c "$REAL_USER"
sudo usermod -aG dialout "$REAL_USER"

############################
# GPIO UART for ROS Robot Controller (STM32) expansion board
# The board connects via the 40-pin GPIO header UART (ttyAMA0 on Pi5,
# enabled by dtoverlay=uart0-pi5). Note: ttyAMA10 is a different UART.
# Install the udev symlink rule and enable the UART overlay in boot config.
############################

UDEV_RULES_DIR="$SRC_DIR/turbopi_ros/etc/udev"

if [ -f "$UDEV_RULES_DIR/99-ttyAMA-rrc.rules" ]; then
    echo "Installing udev rule for ROS Robot Controller UART..."
    sudo cp "$UDEV_RULES_DIR/99-ttyAMA-rrc.rules" /etc/udev/rules.d/
fi

if [ -f "$UDEV_RULES_DIR/99-rplidar.rules" ]; then
    echo "Installing udev rule for RPLidar (/dev/rplidar symlink)..."
    sudo cp "$UDEV_RULES_DIR/99-rplidar.rules" /etc/udev/rules.d/
fi

sudo udevadm control --reload-rules
sudo udevadm trigger

CONFIG="/boot/firmware/config.txt"
if [ -f "$CONFIG" ] && ! grep -q "dtoverlay=uart0-pi5" "$CONFIG"; then
    echo "Enabling GPIO UART overlay in $CONFIG ..."
    echo "" | sudo tee -a "$CONFIG" > /dev/null
    echo "# Enable GPIO UART for ROS Robot Controller (STM32) expansion board" | sudo tee -a "$CONFIG" > /dev/null
    echo "dtoverlay=uart0-pi5" | sudo tee -a "$CONFIG" > /dev/null
fi

############################
# Done
############################

echo ""
echo "======================================="
echo "ROS2 Jazzy setup completed"
echo "Workspace: $WS_DIR"
echo ""
echo "REBOOT the Pi5 to activate the GPIO UART overlay,"
echo "then verify with:"
echo "  ls -la /dev/rrc       (STM32 board, after reboot)"
echo "  ls -la /dev/rplidar   (RPLidar, when USB plugged in)"
echo ""
echo "Open a NEW terminal or run:"
echo "source ~/.bashrc"
echo "======================================="
