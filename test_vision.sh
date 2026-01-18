#!/bin/bash

# 視覺系統快速測試腳本

echo "=========================================="
echo "🔍 ArduPilot 視覺系統測試"
echo "=========================================="

# 檢查 ROS 2 環境
if [ -z "$ROS_DISTRO" ]; then
    echo "載入 ROS 2 環境..."
    source /opt/ros/humble/setup.bash
fi

echo ""
echo "1️⃣  檢查相機主題..."
timeout 3 ros2 topic hz /camera/image_raw 2>&1 | head -n 5

echo ""
echo "2️⃣  啟動形狀偵測器 (10秒測試)..."
timeout 10 python3 /media/user/Linux_Extra/workspaces/ros2_scripts/shape_detector.py &
DETECTOR_PID=$!

echo ""
echo "3️⃣  監控偵測結果..."
sleep 3
timeout 5 python3 /media/user/Linux_Extra/workspaces/ros2_scripts/detection_monitor.py

echo ""
echo "4️⃣  清理..."
kill $DETECTOR_PID 2>/dev/null

echo ""
echo "=========================================="
echo "✅ 測試完成!"
echo ""
echo "下一步:"
echo "  - 在 Gazebo 中放置物體進行測試"
echo "  - 啟動完整系統: ./native_sim_launch.sh"
echo "  - 查看文檔: cat VISION_SYSTEM_README.md"
echo "=========================================="
