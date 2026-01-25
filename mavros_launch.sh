#!/bin/bash

# MAVROS Launch Script for ArduPilot SITL
# Usage: ./mavros_launch.sh
# Connects to ArduPilot SITL on UDP port 14550 (or similar) and bridges to ROS 2.

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check if MAVROS is installed
if ! ros2 pkg list | grep -q mavros; then
    echo "Error: MAVROS is not installed."
    echo "Please install it using: sudo apt install ros-humble-mavros ros-humble-mavros-extras"
    exit 1
fi

echo "============================================"
echo "🚀 Starting MAVROS Bridge"
echo "============================================"
echo " > Freq: 14550/14551 UDP"
echo " > NS:   /mavros"
echo "============================================"

# Launch MAVROS
# fcu_url: UDP connection to ArduPilot (SITL usually outputs to 14550 or 14551)
# gcs_url: Forward MAVLink to QGC (so you don't lose QGC connection)
# tgt_system: Target System ID (usually 1 for the vehicle)
# tgt_component: Target Component ID (1 for autopilot)

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:=udp://127.0.0.1:14551@14555 \
    -p gcs_url:=udp://@127.0.0.1:14550 \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:=screen
