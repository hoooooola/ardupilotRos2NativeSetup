#!/bin/bash

# QuadPlane Gazebo Launch Script
# Usage: ./quadplane_gazebo_launch.sh

# QGC Location
QGC_DIR="/media/user/Linux_Extra/workspaces/qgc"
QGC_PATH="$QGC_DIR/QGroundControl.AppImage"

if [ -z "$TMUX" ]; then
    echo "Starting QuadPlane Gazebo Session..."
    tmux kill-session -t quad_gz 2>/dev/null
    
    # Start QGC
    if [ -f "$QGC_PATH" ]; then
        "$QGC_PATH" &> /dev/null &
        QGC_PID=$!
    fi
    cleanup() {
        [ -n "$QGC_PID" ] && kill "$QGC_PID" 2>/dev/null
    }
    trap cleanup EXIT

    tmux new-session -d -s quad_gz

    # Pane 0: SITL
    tmux rename-window -t quad_gz:0 'SITL'
    tmux send-keys -t quad_gz:0 "export PATH=$PATH:/media/user/Linux_Extra/workspaces/ardupilot/Tools/autotest" C-m
    tmux send-keys -t quad_gz:0 "bash $(realpath "$0") --child-sitl" C-m

    # Pane 1: Gazebo
    tmux split-window -h -t quad_gz:0
    tmux send-keys -t quad_gz:0.1 "source ~/.bashrc" C-m
    tmux send-keys -t quad_gz:0.1 "export IGN_RENDER_ENGINE=ogre2" C-m
    tmux send-keys -t quad_gz:0.1 "export IGN_IP=127.0.0.1" C-m
    tmux send-keys -t quad_gz:0.1 "export IGN_PARTITION=sim" C-m
    tmux send-keys -t quad_gz:0.1 "export QT_QPA_PLATFORM=xcb" C-m
    tmux send-keys -t quad_gz:0.1 "ign gazebo -v 4 -r /media/user/Linux_Extra/workspaces/ardupilot_gazebo/worlds/quadplane_runway.world" C-m

    # Pane 2: ROS 2 Bridge (Optional, but enabling for future use)
    tmux split-window -v -t quad_gz:0.1
    tmux send-keys -t quad_gz:0.2 "source /opt/ros/humble/setup.bash" C-m
    tmux send-keys -t quad_gz:0.2 "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/media/user/Linux_Extra/workspaces/ros_gz_bridge.yaml" C-m

    tmux attach-session -t quad_gz
    exit 0
fi

if [ "$1" == "--child-sitl" ]; then
    source ~/.bashrc
    export PATH=$PATH:/media/user/Linux_Extra/workspaces/ardupilot/Tools/autotest
    export IGN_IP=127.0.0.1
    
    # Launch ArduPlane with QuadPlane params
    # Using gazebo-zephyr frame as base because it sets up the external JSON model correctly for a plane.
    # We overlay our QuadPlane params on top.
    sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --add-param-file=/media/user/Linux_Extra/workspaces/quadplane_gazebo.parm --console --map
    exit 0
fi
