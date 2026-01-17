#!/bin/bash
set -e  # Exit on error
export DEBIAN_FRONTEND=noninteractive
export APT_opts="-o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

LOG_FILE="install_log.txt"
exec > >(tee -a $LOG_FILE) 2>&1

echo -e "${GREEN}=== Native ROS 2 & ArduPilot Simulation Installer ===${NC}"
echo "Target Configuration:"
echo "  - ROS 2: Humble (Binary)"
echo "  - Gazebo: Fortress"
echo "  - ArduPilot: Copter-4.6.3"
echo "====================================================="

# --- Step 1: Cleanup Legacy Source Build ---
ROS2_SRC_DIR="/media/user/Linux_Extra/workspaces/ros2_humble"

if [ -d "$ROS2_SRC_DIR" ]; then
    echo -e "${YELLOW}[Action Required] Found legacy ROS 2 Source Build at: $ROS2_SRC_DIR${NC}"
    echo "This directory takes up ~11GB and is no longer needed (we are switching to Binary)."
    read -p "Do you want to delete it to free up space? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing $ROS2_SRC_DIR..."
        rm -rf "$ROS2_SRC_DIR"
        echo -e "${GREEN}Cleanup complete.${NC}"
    else
        echo "Skipping cleanup. Moving on..."
    fi
else
    echo -e "${GREEN}[OK] No legacy ROS 2 source build found.${NC}"
fi

# --- Step 2: Install System Dependencies (ROS 2 + Gazebo) ---
echo -e "${YELLOW}[Step 2] Installing ROS 2 Humble & Gazebo Fortress...${NC}"
echo "Requesting sudo permission..."

sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release

# Add ROS 2 key/repo if not exists
if ! [ -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

# Install ROS 2
sudo apt install $APT_opts -y ros-humble-desktop ros-dev-tools

# Install Gazebo Fortress (Ignition)
sudo apt install -y ignition-fortress

# Install Bridge
sudo apt install -y ros-humble-ros-gz

# Fix pesky dependencies (Now that jammy-updates is enabled, this should just work)
echo -e "${YELLOW}Fixing libpulse-dev dependencies...${NC}"
sudo apt install -y libpulse-dev libpulse0 libpulse-mainloop-glib0 --fix-missing

# --- Step 3: ArduPilot Setup ---
echo -e "${YELLOW}[Step 3] Setting up ArduPilot Copter-4.6.3...${NC}"
ARDUPILOT_DIR="/media/user/Linux_Extra/workspaces/ardupilot"

if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo -e "${RED}Error: ArduPilot directory not found at $ARDUPILOT_DIR${NC}"
    exit 1
fi

cd "$ARDUPILOT_DIR"
echo "Cleaning git repository..."
git reset --hard
git clean -fdx

echo "Checking out Copter-4.6.3..."
git fetch --tags
git checkout Copter-4.6.3
git submodule update --init --recursive

echo "Running ArduPilot prereqs script..."
# Note: This script typically asks for sudo inside. 
# We pass -y but it might still need password if sudo timeout expires.
bash Tools/environment_install/install-prereqs-ubuntu.sh -y

# --- Step 3b: QGroundControl Setup ---
echo -e "${YELLOW}[Step 3b] Setting up QGroundControl...${NC}"
QGC_DIR="/media/user/Linux_Extra/workspaces/qgc"
QGC_APPIMAGE="$QGC_DIR/QGroundControl.AppImage"

if [ -f "$QGC_APPIMAGE" ]; then
    echo "Found QGroundControl AppImage. Setting permissions..."
    chmod +x "$QGC_APPIMAGE"
    
    # Install libfuse2 (required for AppImage on Ubuntu 22.04)
    echo "Installing libfuse2 (for AppImage support)..."
    sudo apt install -y libfuse2
else
    echo -e "${YELLOW}Warning: QGroundControl AppImage not found at $QGC_APPIMAGE${NC}"
    echo "Please download it to that location later."
fi

# --- Step 4: Environment Configuration ---
echo -e "${YELLOW}[Step 4] Configuring Environment (.bashrc)...${NC}"

BASHRC="$HOME/.bashrc"
BACKUP_BASHRC="$HOME/.bashrc.bak.$(date +%F_%T)"
cp "$BASHRC" "$BACKUP_BASHRC"
echo "Backed up .bashrc to $BACKUP_BASHRC"

# Helper function to append if not exists
append_if_missing() {
    grep -qF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

append_if_missing "source /opt/ros/humble/setup.bash"
append_if_missing "export PATH=$ARDUPILOT_DIR/Tools/autotest:\$PATH"
append_if_missing "export PATH=$HOME/.local/bin:\$PATH"

# Reload bashrc for this script's session (not parent shell, but helps next steps)
source /opt/ros/humble/setup.bash

# --- Step 5: Verification Build ---
echo -e "${YELLOW}[Step 5] Configuring and Building SITL...${NC}"
./waf configure --board sitl

# Check if configure succeeded
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Configuration successful!${NC}"
    echo "Starting build (this might take a while)..."
    ./waf copter
else
    echo -e "${RED}Waf configure failed! Check install_log.txt${NC}"
    exit 1
fi

echo -e "${GREEN}=== Installation Complete! ===${NC}"
echo "Please restart your terminal or run 'source ~/.bashrc' to apply changes."
echo "To test run: sim_vehicle.py -v Copter -f gazebo-iris --console --map"
