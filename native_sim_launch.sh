#!/bin/bash

# Native ArduPilot + Gazebo Launch Script (Robust Tmux & QGC Support)
# Usage: ./native_sim_launch.sh

# QGC Location
QGC_DIR="/media/user/Linux_Extra/workspaces/qgc"
QGC_PATH="$QGC_DIR/QGroundControl.AppImage"

# ==========================================
# Phase 1: Parent Launcher (Outside Tmux)
# ==========================================
if [ -z "$TMUX" ]; then
    echo "============================================"
    echo "🚀 Starting ArduCopter SITL + Gazebo Fortress"
    echo "============================================"
    echo " > Vehicle: Copter"
    echo " > World:   gazebo-iris"
    echo " > Output:  Tmux + Map"
    echo " > GCS:     QGroundControl"
    echo "============================================"

    # 1. Kill any stale session
    tmux kill-session -t native_sim 2>/dev/null

    # 2. Start QGC in background
    if [ -f "$QGC_PATH" ]; then
        echo "Starting QGroundControl..."
        "$QGC_PATH" &> /dev/null &
        QGC_PID=$!
        echo " - QGC PID: $QGC_PID"
    else
        echo "Warning: QGroundControl not found at $QGC_PATH"
    fi

    # 3. Define cleanup trap (to kill QGC when we exit)
    cleanup() {
        echo "Shutting down..."
        [ -n "$QGC_PID" ] && kill "$QGC_PID" 2>/dev/null
        echo "Done."
    }
    trap cleanup EXIT

    # 4. Start Tmux Session (Split Windows)
    echo "Starting Tmux Session..."
    tmux new-session -d -s native_sim
    
    # Pane 0: SITL (Simulation)
    tmux rename-window -t native_sim:0 'SITL'
    tmux send-keys -t native_sim:0 "export PATH=$PATH:/media/user/Linux_Extra/workspaces/ardupilot/Tools/autotest" C-m
    tmux send-keys -t native_sim:0 "bash $(realpath "$0") --child-sitl" C-m

    # Pane 1: Gazebo (Physics)
    tmux split-window -h -t native_sim:0
    tmux send-keys -t native_sim:0.1 "source ~/.bashrc" C-m
    # Explicitly set critical rendering and comms variables
    tmux send-keys -t native_sim:0.1 "export IGN_RENDER_ENGINE=ogre2" C-m
    tmux send-keys -t native_sim:0.1 "export IGN_IP=127.0.0.1" C-m
    tmux send-keys -t native_sim:0.1 "export IGN_PARTITION=sim" C-m
    tmux send-keys -t native_sim:0.1 "export QT_QPA_PLATFORM=xcb" C-m
    tmux send-keys -t native_sim:0.1 "ign gazebo -v 4 -r /media/user/Linux_Extra/workspaces/ardupilot_gazebo/worlds/iris_arducopter_runway.world" C-m

    # Pane 2: ROS 2 Bridge (Ready to run)
    tmux split-window -v -t native_sim:0.1
    tmux send-keys -t native_sim:0.2 "source /opt/ros/humble/setup.bash" C-m
    tmux send-keys -t native_sim:0.2 "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/media/user/Linux_Extra/workspaces/ros_gz_bridge.yaml" C-m

    # Attach
    tmux attach-session -t native_sim
    
    # When tmux session ends
    exit 0
fi

# ==========================================
# Phase 2: Simulation Runner (Inside Tmux)
# ==========================================


if [ "$1" == "--child-sitl" ]; then
    echo "Loading environment..."
    source ~/.bashrc
    source /opt/ros/humble/setup.bash
    
    # Export for Fortress (Backwards compatibility)
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH
    export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH
    
    # 2026-01-17 Fixes for Ubuntu 22.04 Wayland + Fortress
    export QT_QPA_PLATFORM=xcb
    export IGN_RENDER_ENGINE=ogre2
    export IGN_IP=127.0.0.1
    export IGN_PARTITION=sim

    # Check ArduPilot Path
    if ! command -v sim_vehicle.py &> /dev/null; then
         export PATH=$PATH:/media/user/Linux_Extra/workspaces/ardupilot/Tools/autotest
         export PATH=$PATH:$HOME/.local/bin
    fi

    # Check requirements again
    if ! command -v sim_vehicle.py &> /dev/null; then
        echo "Error: sim_vehicle.py not found!"
        echo "Please run install_native_ros2_sim.sh first to set up paths."
        read -p "Press Enter to exit..."
        exit 1
    fi

    echo "Launching ArduPilot SITL..."
    # Use --model JSON to ensure compatibility with ardupilot_gazebo plugin
    sim_vehicle.py -v Copter -f gazebo-iris --model JSON --console --map
    exit 0
fi
