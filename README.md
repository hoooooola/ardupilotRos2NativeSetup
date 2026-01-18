# 🚁 ArduPilot ROS 2 原生模擬環境

完整的 ArduPilot + Gazebo Fortress + ROS 2 Humble + AI 視覺系統

---

## 🚀 快速開始

### 一鍵啟動完整系統

```bash
./native_sim_launch.sh
```

這會自動啟動:
- ✅ ArduPilot SITL (飛行控制模擬)
- ✅ Gazebo Fortress (3D 物理模擬)
- ✅ ROS 2 Bridge (數據橋接)
- ✅ AI 形狀偵測器 (OpenCV)
- ✅ H.264 視訊串流 (到 QGC)
- ✅ QGroundControl (地面控制站)

### 檢查系統狀態

```bash
./check_vision_status.sh
```

---

## 📚 文檔導航

### 🎯 快速參考
- **[docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)** - 常用指令快速參考卡

### 📖 完整文檔
- **[ardupilot_gazebo/README.md](ardupilot_gazebo/README.md)** - 完整系統文檔
  - ROS 2 整合配置
  - AI 視覺系統架構
  - 深度學習升級指南
  - 效能優化技巧
  - 除錯與診斷

### 🤖 視覺系統
- **[docs/VISION_SYSTEM_README.md](docs/VISION_SYSTEM_README.md)** - 視覺系統詳細說明
  - 形狀偵測原理
  - YOLO/TensorFlow 整合
  - Jetson 硬體加速
  - 視覺導航範例

### 🎓 學習總結
- **[docs/PROJECT_SUMMARY.md](docs/PROJECT_SUMMARY.md)** - 專案學習總結
  - 完整時間軸
  - 系統架構圖
  - 關鍵技術學習
  - 問題解決經驗
  - 效能指標

---

## 🏗️ 系統架構

```
Gazebo 相機感測器
    ↓
ROS 2 Bridge (/camera/image_raw)
    ↓
形狀偵測器 (OpenCV/YOLO)
    ↓
標註影像 (/camera/annotated) + 偵測結果 (/detections)
    ↓
H.264 串流器 → QGroundControl (UDP:5600)
```

---

## 📁 專案結構

```
workspaces/
├── README.md                     # 📍 本文件 (專案入口)
│
├── docs/                         # 📚 文檔資料夾
│   ├── QUICK_REFERENCE.md        # 快速參考卡
│   ├── PROJECT_SUMMARY.md        # 學習總結
│   └── VISION_SYSTEM_README.md   # 視覺系統文檔
│
├── ros2_scripts/                 # 🤖 ROS 2 Python 節點
│   ├── shape_detector.py         # 形狀偵測器
│   ├── annotated_streamer_h264.py # H.264 串流器
│   ├── detection_monitor.py      # 偵測監控工具
│   └── camera_streamer_h264.py   # 原始串流器
│
├── ardupilot/                    # ArduPilot 源碼
├── ardupilot_gazebo/             # Gazebo 插件與模型
│   └── README.md                 # 完整系統文檔
│
├── native_sim_launch.sh          # 🚀 自動化啟動腳本
├── check_vision_status.sh        # 系統狀態檢查
├── test_vision.sh                # 視覺系統測試
└── ros_gz_bridge.yaml            # ROS 2 Bridge 配置
```

---

## 🎯 主要功能

### ✅ 已實現功能

1. **完整模擬環境**
   - ArduPilot SITL 飛行控制
   - Gazebo Fortress 物理模擬
   - QGroundControl 地面站

2. **ROS 2 整合**
   - Gazebo ↔ ROS 2 數據橋接
   - 相機影像主題發布
   - 即時數據流處理

3. **AI 視覺系統**
   - OpenCV 形狀偵測
   - 即時影像標註
   - JSON 格式偵測結果

4. **視訊串流**
   - H.264 編碼
   - UDP 低延遲串流
   - QGC 即時顯示

### 🚧 可擴展功能

1. **深度學習**
   - YOLOv8 物體偵測
   - TensorFlow 模型部署
   - GPU 加速推理

2. **高級視覺**
   - 物體追蹤 (DeepSORT)
   - 語義分割
   - 3D 偵測

3. **自主導航**
   - 視覺 SLAM
   - 路徑規劃
   - 避障算法

---

## 🔧 系統需求

### 軟體環境
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- Python 3.10+

### 硬體建議
- CPU: 4 核心以上
- RAM: 8GB 以上
- GPU: 可選 (用於深度學習加速)

---

## 📊 ROS 2 主題

### 訂閱主題
- `/camera/image_raw` - 原始相機影像

### 發布主題
- `/camera/annotated` - 標註後的影像
- `/detections` - JSON 格式偵測結果

### 查看主題
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /detections
```

---

## 🐛 除錯

### 快速診斷
```bash
./check_vision_status.sh
```

### 完全重啟
```bash
pkill -9 -f "QGroundControl|gazebo|arducopter"
tmux kill-session -t native_sim
./native_sim_launch.sh
```

### 查看日誌
```bash
# Tmux 窗格切換
Ctrl+b, 然後方向鍵

# ROS 2 節點資訊
ros2 node list
ros2 node info /shape_detector
```

---

## 📖 學習路徑

### 1. 初學者
1. 閱讀 [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)
2. 運行 `./native_sim_launch.sh`
3. 在 QGC 中觀察視訊串流

### 2. 進階使用者
1. 閱讀 [ardupilot_gazebo/README.md](ardupilot_gazebo/README.md)
2. 修改 `ros2_scripts/shape_detector.py` 調整偵測參數
3. 整合 YOLOv8 物體偵測

### 3. 開發者
1. 閱讀 [docs/PROJECT_SUMMARY.md](docs/PROJECT_SUMMARY.md)
2. 研究系統架構和數據流
3. 開發自定義視覺算法

---

## 🎓 參考資源

### 官方文檔
- [ArduPilot Dev Guide](https://ardupilot.org/dev/)
- [Gazebo Fortress](https://gazebosim.org/docs/fortress)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

### AI 視覺
- [YOLOv8](https://docs.ultralytics.com/)
- [OpenCV](https://docs.opencv.org/4.x/)
- [TensorFlow](https://www.tensorflow.org/)

---

## 🤝 貢獻

歡迎提交 Issue 和 Pull Request!

---

## 📝 更新日誌

### v1.0 (2026-01-18)
- ✅ 完整的 ArduPilot + Gazebo + ROS 2 整合
- ✅ AI 視覺系統 (OpenCV 形狀偵測)
- ✅ H.264 視訊串流到 QGC
- ✅ 自動化啟動腳本
- ✅ 完整文檔系統

---

**建立日期**: 2026-01-18  
**維護者**: User + AI Assistant  
**授權**: MIT
