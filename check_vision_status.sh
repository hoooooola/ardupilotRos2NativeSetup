#!/bin/bash

# ArduPilot 視覺系統狀態檢查

echo "=========================================="
echo "🔍 ArduPilot 視覺系統狀態檢查"
echo "=========================================="
echo ""

# 載入 ROS 2 環境
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# 1. 檢查 ROS 2 節點
echo "1️⃣  ROS 2 節點狀態:"
echo "---"
ros2 node list 2>/dev/null | while read node; do
    if [[ "$node" == *"shape_detector"* ]]; then
        echo "  ✅ $node (形狀偵測器)"
    elif [[ "$node" == *"streamer"* ]]; then
        echo "  ✅ $node (視訊串流器)"
    elif [[ "$node" == *"bridge"* ]]; then
        echo "  ✅ $node (ROS 2 橋接)"
    else
        echo "  ✅ $node"
    fi
done
echo ""

# 2. 檢查主題
echo "2️⃣  視覺主題狀態:"
echo "---"
for topic in "/camera/image_raw" "/camera/annotated" "/detections"; do
    if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
        echo "  ✅ $topic"
    else
        echo "  ❌ $topic (未找到)"
    fi
done
echo ""

# 3. 檢查影像頻率
echo "3️⃣  影像串流頻率:"
echo "---"
echo "  📹 原始影像:"
timeout 2 ros2 topic hz /camera/image_raw 2>/dev/null | grep "average rate" | head -1 | sed 's/^/    /'
echo "  🎨 標註影像:"
timeout 2 ros2 topic hz /camera/annotated 2>/dev/null | grep "average rate" | head -1 | sed 's/^/    /'
echo ""

# 4. 檢查進程
echo "4️⃣  系統進程:"
echo "---"
if pgrep -f "arducopter" > /dev/null; then
    echo "  ✅ ArduPilot SITL"
else
    echo "  ❌ ArduPilot SITL"
fi

if pgrep -f "ign gazebo" > /dev/null; then
    echo "  ✅ Gazebo Fortress"
else
    echo "  ❌ Gazebo Fortress"
fi

if pgrep -f "QGroundControl" > /dev/null; then
    echo "  ✅ QGroundControl"
else
    echo "  ⚠️  QGroundControl (未運行)"
fi
echo ""

# 5. 檢查最近的偵測
echo "5️⃣  最近的偵測結果:"
echo "---"
echo "  (監聽 2 秒...)"
timeout 2 ros2 topic echo /detections 2>/dev/null | head -10 || echo "  ℹ️  目前沒有偵測到物體"
echo ""

echo "=========================================="
echo "✅ 狀態檢查完成!"
echo ""
echo "💡 提示:"
echo "  - 在 Gazebo 中添加物體進行偵測測試"
echo "  - 在 QGC 中查看即時視訊 (UDP:5600)"
echo "  - 使用 'ros2 topic echo /detections' 監控偵測"
echo "=========================================="
