#!/bin/bash

# Native ArduPilot QuadPlane SITL Launch Script
# Usage: ./quadplane_sitl_launch.sh
# description: 啟動 ArduPilot QuadPlane (4+1) 標準模擬環境。
# 注意：此腳本使用 ArduPilot 內建的高精度物理模擬 (Internal SITL)，而非 Gazebo。
# 這是學習 VTOL 模式與轉換邏輯 (Transition) 最穩定且快速的方式。

# QGC Location
QGC_DIR="/media/user/Linux_Extra/workspaces/qgc"
QGC_PATH="$QGC_DIR/QGroundControl.AppImage"

# ==========================================
# Phase 1: Parent Launcher (Outside Tmux)
# ==========================================
if [ -z "$TMUX" ]; then
    echo "============================================"
    echo "🚀 Starting ArduPlane (QuadPlane) SITL"
    echo "============================================"
    echo " > Vehicle: ArduPlane (QuadPlane 4+1)"
    echo " > Physics: Internal SITL (Fast & Stable)"
    echo " > Output:  Tmux + Map + Console"
    echo " > GCS:     QGroundControl"
    echo "============================================"

    # 1. Kill any stale session
    tmux kill-session -t quadplane_sim 2>/dev/null

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

    # 4. Start Tmux Session
    echo "Starting Tmux Session..."
    tmux new-session -d -s quadplane_sim
    
    # Pane 0: SITL (Simulation)
    tmux rename-window -t quadplane_sim:0 'SITL'
    tmux send-keys -t quadplane_sim:0 "export PATH=$PATH:/media/user/Linux_Extra/workspaces/ardupilot/Tools/autotest" C-m
    # Launch with --child-sitl flag to execute the Phase 2 logic below
    tmux send-keys -t quadplane_sim:0 "bash $(realpath "$0") --child-sitl" C-m

    # Attach
    tmux attach-session -t quadplane_sim
    
    exit 0
fi

# ==========================================
# Phase 2: Simulation Runner (Inside Tmux)
# ==========================================

if [ "$1" == "--child-sitl" ]; then
    echo "Loading environment..."
    source ~/.bashrc
    
    # Check ArduPilot Path
    if ! command -v sim_vehicle.py &> /dev/null; then
         export PATH=$PATH:/media/user/Linux_Extra/workspaces/ardupilot/Tools/autotest
         export PATH=$PATH:$HOME/.local/bin
    fi

    echo "Launching ArduPilot QuadPlane SITL..."
    # -v ArduPlane: 指定載具為固定翼
    # -f quadplane: 指定構型為 4+1 資料 VTOL
    # --console: 顯示 MAVProxy 控制台 (看模式、參數)
    # --map: 顯示 2D 地圖 (看航點、飛行軌跡)
    sim_vehicle.py -v ArduPlane -f quadplane --console --map
    exit 0
fi
