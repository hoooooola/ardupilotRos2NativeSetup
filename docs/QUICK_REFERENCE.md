# 🚁 ArduPilot + ROS 2 + AI Vision - 快速參考卡

## 🚀 一鍵啟動

```bash
./native_sim_launch.sh
```

## 📋 系統檢查

```bash
./check_vision_status.sh
```

## 🎯 常用指令

### ROS 2 主題

```bash
# 列出所有主題
ros2 topic list

# 查看影像頻率
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/annotated

# 查看偵測結果
ros2 topic echo /detections

# 視覺化影像
rqt_image_view
```

### ROS 2 節點

```bash
# 列出所有節點
ros2 node list

# 查看節點資訊
ros2 node info /shape_detector
ros2 node info /annotated_streamer_h264
```

### 手動啟動組件

```bash
# ROS 2 Bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./ros_gz_bridge.yaml

# 形狀偵測器
python3 ros2_scripts/shape_detector.py

# H.264 串流器
python3 ros2_scripts/annotated_streamer_h264.py

# 偵測監控
python3 ros2_scripts/detection_monitor.py
```

## 🔧 除錯指令

```bash
# 檢查進程
ps aux | grep -E "arducopter|gazebo|shape_detector|streamer"

# 檢查 UDP 串流
sudo tcpdump -i lo udp port 5600

# 檢查 Gazebo 主題
ign topic -l

# 查看 tmux 窗格
tmux list-panes -t native_sim -a
```

## 📊 QGC 視訊設定

1. **Application Settings** → **General** → **Video**
2. **Video Source**: `UDP h.264 Video Stream`
3. **UDP Port**: `5600`
4. **Low Latency Mode**: ✅

## 🎨 ROS 2 主題架構

```
/camera/image_raw      → 原始影像 (sensor_msgs/Image)
/camera/annotated      → 標註影像 (sensor_msgs/Image)
/detections            → 偵測結果 (std_msgs/String, JSON)
```

## 🐛 常見問題快速修復

### 沒有偵測到物體
```bash
# 重啟偵測器
pkill -f shape_detector
python3 ros2_scripts/shape_detector.py
```

### QGC 沒有視訊
```bash
# 重啟串流器
pkill -f annotated_streamer
python3 ros2_scripts/annotated_streamer_h264.py
```

### 完全重啟系統
```bash
pkill -9 -f "QGroundControl|gazebo|arducopter|shape_detector|streamer"
tmux kill-session -t native_sim
./native_sim_launch.sh
```

## 📁 重要檔案位置

```
/media/user/Linux_Extra/workspaces/
├── native_sim_launch.sh          # 主啟動腳本
├── check_vision_status.sh        # 狀態檢查
├── ros_gz_bridge.yaml            # Bridge 配置
├── ros2_scripts/                 # ROS 2 Python 節點
│   ├── shape_detector.py         # 偵測器
│   └── annotated_streamer_h264.py # 串流器
├── docs/                         # 文檔資料夾
│   ├── QUICK_REFERENCE.md        # 本文件
│   ├── VISION_SYSTEM_README.md   # 視覺系統文檔
│   └── PROJECT_SUMMARY.md        # 專案總結
├── ardupilot_gazebo/README.md    # 完整文檔
└── README.md                     # 專案入口
```

## 🧠 升級到 YOLO

```bash
# 安裝
pip3 install ultralytics

# 測試
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt')"
```

## 📖 查看文檔

```bash
# 專案入口
cat README.md

# 完整系統文檔
cat ardupilot_gazebo/README.md

# 視覺系統詳細說明
cat docs/VISION_SYSTEM_README.md

# 專案學習總結
cat docs/PROJECT_SUMMARY.md

# 本快速參考
cat docs/QUICK_REFERENCE.md
```

## 🎯 效能參數

| 項目 | 數值 |
|------|------|
| 影像頻率 | ~20 Hz |
| 視訊延遲 | < 100ms |
| 解析度 | 640x480 |
| 編碼 | H.264 |
| 位元率 | 2000 kbps |

## 💡 快速提示

- **Tmux 切換窗格**: `Ctrl+b` 然後方向鍵
- **Tmux 分離**: `Ctrl+b` 然後 `d`
- **Tmux 重新連接**: `tmux attach -t native_sim`
- **停止所有**: `Ctrl+C` 在主 tmux 視窗

---

**更新日期**: 2026-01-18
