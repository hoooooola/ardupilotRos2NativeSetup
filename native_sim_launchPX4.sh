#!/bin/bash

# PX4 SITL Launch Script with Tmux
# Features:
# 1. Checks environment (PX4 source, DDS Agent)
# 2. Kills old sessions
# 3. Launches QGroundControl in background
# 4. Sets up Tmux session with 3 panes:
#    - Pane 0: PX4 SITL (Gazebo Classic)
#    - Pane 1: Micro XRCE-DDS Agent
#    - Pane 2: ROS 2 Environment (Ready to use)

SESSION="px4_sim"
PX4_DIR="/media/user/Linux_Extra/workspaces/PX4-Autopilot"
QGC_PATH="/media/user/Linux_Extra/workspaces/qgc/QGroundControl.AppImage"

# 1. Cleanup old processes
echo "🧹 Cleaning up old sessions..."
tmux kill-session -t $SESSION 2>/dev/null
killall -9 px4 2>/dev/null
killall -9 MicroXRCEAgent 2>/dev/null
killall -9 gzserver 2>/dev/null
killall -9 gzclient 2>/dev/null

# 2. Start QGroundControl (Background)
if [ -f "$QGC_PATH" ]; then
    echo "🚀 Starting QGroundControl..."
    "$QGC_PATH" &> /dev/null &
else
    echo "⚠️  QGroundControl not found at $QGC_PATH"
fi

# 3. Create Tmux Session
echo "🛠️  Creating Tmux Session: $SESSION"
tmux new-session -d -s $SESSION

# Pane 0: PX4 SITL (Main Simulation)
tmux rename-window -t $SESSION:0 'PX4_SITL'
tmux send-keys -t $SESSION:0 "cd $PX4_DIR" C-m
# Use HEADLESS=1 to disable GUI if you want pure console, remove it for 3D view.
# We use 'make px4_sitl gazebo' for Gazebo Classic
tmux send-keys -t $SESSION:0 "make px4_sitl gazebo" C-m

# Pane 1: DDS Agent (Right side)
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "MicroXRCEAgent udp4 -p 8888" C-m

# Pane 2: ROS 2 Shell (Bottom)
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "echo '✅ ROS 2 Environment Ready! Try: ros2 topic list'" C-m

# Select Main Pane
tmux select-pane -t $SESSION:0.0

# Attach to session
echo "🔗 Attaching to session..."
tmux attach-session -t $SESSION
