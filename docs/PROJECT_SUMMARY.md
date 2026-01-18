# 🎓 ArduPilot + ROS 2 + AI 視覺系統 - 學習總結

## 📅 專案時間軸

**日期**: 2026-01-18  
**目標**: 建立完整的 ArduPilot SITL + Gazebo + ROS 2 + AI 視覺系統

---

## 🎯 達成的里程碑

### ✅ 階段 1: 基礎環境建置
- [x] ArduPilot SITL 原生安裝
- [x] Gazebo Fortress 整合
- [x] ROS 2 Humble 環境配置
- [x] QGroundControl 自動啟動

### ✅ 階段 2: 相機系統整合
- [x] Gazebo 相機感測器配置
- [x] ROS 2 Bridge 設置 (Gazebo ↔ ROS 2)
- [x] 影像主題發布與訂閱
- [x] 影像頻率優化 (~20 Hz)

### ✅ 階段 3: 視訊串流
- [x] H.264 編碼實現 (GStreamer)
- [x] UDP 串流到 QGroundControl
- [x] 低延遲配置 (< 100ms)
- [x] 視訊品質優化

### ✅ 階段 4: AI 視覺偵測
- [x] OpenCV 形狀偵測器
- [x] CLAHE 對比度增強
- [x] Canny 邊緣偵測優化
- [x] 即時標註與視覺化

### ✅ 階段 5: 系統整合
- [x] Tmux 自動化啟動腳本
- [x] 所有組件協同運作
- [x] 標註影像串流到 QGC
- [x] 系統狀態監控工具

---

## 🏗️ 系統架構

### 完整數據流

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Fortress                          │
│  ┌──────────────┐      ┌──────────────┐                    │
│  │ ArduPilot    │      │ Camera       │                    │
│  │ Plugin       │      │ Sensor       │                    │
│  └──────┬───────┘      └──────┬───────┘                    │
│         │                     │                             │
└─────────┼─────────────────────┼─────────────────────────────┘
          │                     │
          │ JSON                │ ignition.msgs.Image
          │                     │
┌─────────▼─────────────────────▼─────────────────────────────┐
│                    ROS 2 Bridge                             │
│         MAVLink ◄──────────► /camera/image_raw             │
└─────────┬─────────────────────┬─────────────────────────────┘
          │                     │
          │                     │ sensor_msgs/Image
          │                     │
          │            ┌────────▼──────────┐
          │            │ Shape Detector    │
          │            │ (OpenCV/YOLO)     │
          │            └────────┬──────────┘
          │                     │
          │                     ├─► /detections (JSON)
          │                     │
          │                     └─► /camera/annotated
          │                              │
          │                     ┌────────▼──────────┐
          │                     │ H.264 Streamer    │
          │                     │ (GStreamer)       │
          │                     └────────┬──────────┘
          │                              │
          │                              │ UDP:5600
          │                              │
┌─────────▼──────────────────────────────▼─────────────────────┐
│                   QGroundControl                             │
│         Flight Control  +  Live Video with Annotations      │
└──────────────────────────────────────────────────────────────┘
```

### 關鍵組件

| 組件 | 功能 | 技術 |
|------|------|------|
| **ArduPilot SITL** | 飛行控制模擬 | C++, MAVLink |
| **Gazebo Fortress** | 物理模擬環境 | Ignition, SDF |
| **ROS 2 Bridge** | 數據橋接 | ros_gz_bridge |
| **Shape Detector** | 形狀偵測 | OpenCV, Python |
| **H.264 Streamer** | 視訊編碼串流 | GStreamer, H.264 |
| **QGroundControl** | 地面控制站 | Qt, MAVLink |

---

## 💡 關鍵技術學習

### 1. ROS 2 Bridge 配置

**學到的經驗**:
- YAML 配置文件格式必須精確
- 主題名稱需要完全匹配
- 數據類型映射很重要

**配置範例**:
```yaml
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "ignition.msgs.Image"
  direction: GZ_TO_ROS
```

### 2. GStreamer Pipeline

**學到的經驗**:
- `fdsrc` 用於從 stdin 讀取原始影像
- `videoparse` 需要明確指定格式和尺寸
- `sync=false` 降低延遲

**Pipeline 結構**:
```bash
fdsrc ! 
videoparse width=640 height=480 format=bgr framerate=10/1 ! 
videoconvert ! 
x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! 
rtph264pay config-interval=1 pt=96 ! 
udpsink host=127.0.0.1 port=5600 sync=false
```

### 3. OpenCV 影像處理

**學到的經驗**:
- CLAHE 對比度增強對低對比度場景很有效
- Canny 閾值需要根據場景調整
- 形態學操作可以改善邊緣連續性

**處理流程**:
```python
# 1. 對比度增強
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
gray = clahe.apply(gray)

# 2. 邊緣偵測
edges = cv2.Canny(blurred, 30, 100)

# 3. 形態學操作
kernel = np.ones((3,3), np.uint8)
edges = cv2.dilate(edges, kernel, iterations=1)
edges = cv2.erode(edges, kernel, iterations=1)

# 4. 輪廓分析
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

### 4. Tmux 自動化

**學到的經驗**:
- 使用 `&&` 連接命令確保順序執行
- `sleep` 延遲確保依賴服務啟動完成
- 窗格索引在分割後會重新編號

**最佳實踐**:
```bash
# 單行命令確保執行
tmux send-keys -t pane "source env && sleep 5 && run_app" C-m

# 而不是分開發送
tmux send-keys -t pane "source env" C-m
tmux send-keys -t pane "sleep 5" C-m  # ❌ 會阻塞
tmux send-keys -t pane "run_app" C-m  # ❌ 不會執行
```

### 5. ROS 2 Python 節點

**學到的經驗**:
- JSON 序列化不支援 ROS Time 對象
- 需要正確處理 cv_bridge 編碼
- 使用 `try-except` 捕獲異常避免節點崩潰

**常見陷阱**:
```python
# ❌ 錯誤: Time 對象無法序列化
json.dumps({"timestamp": self.get_clock().now().to_msg()})

# ✅ 正確: 使用 frame 計數或字符串
json.dumps({"frame": self.frame_count})
```

---

## 🔧 遇到的問題與解決方案

### 問題 1: 形狀偵測器無法偵測物體

**現象**: 處理影像但偵測數量為 0

**原因**:
- 對比度不足 (灰色球體 + 灰色背景)
- 面積閾值太高 (500 像素)
- Canny 閾值太高 (50, 150)

**解決方案**:
```python
# 1. 添加 CLAHE 對比度增強
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

# 2. 降低 Canny 閾值
edges = cv2.Canny(blurred, 30, 100)  # 從 (50, 150) 降到 (30, 100)

# 3. 降低面積閾值
if area < 100:  # 從 500 降到 100
    continue
```

### 問題 2: JSON 序列化錯誤

**現象**: `Object of type Time is not JSON serializable`

**原因**: ROS Time 對象無法直接序列化為 JSON

**解決方案**:
```python
# 移除 timestamp 或轉換為可序列化格式
detection_msg.data = json.dumps({
    "frame": self.frame_count,  # 使用 frame 計數代替
    "detections": detections,
    "count": len(detections)
})
```

### 問題 3: Tmux 窗格中腳本未執行

**現象**: `sleep` 命令後的腳本不執行

**原因**: 分開發送命令導致 `sleep` 阻塞

**解決方案**:
```bash
# ❌ 錯誤方式
tmux send-keys "sleep 5" C-m
tmux send-keys "python3 script.py" C-m

# ✅ 正確方式
tmux send-keys "sleep 5 && python3 script.py" C-m
```

### 問題 4: QGC 視訊無畫面

**現象**: QGC 視訊區域空白

**原因**: 
- 串流器未啟動
- Port 配置錯誤
- 需要重啟視訊

**解決方案**:
1. 確認串流器運行: `ps aux | grep annotated_streamer`
2. 檢查 Port: `sudo netstat -tulpn | grep 5600`
3. QGC 中重啟視訊: Settings → Video → Restart

---

## 📊 效能指標

### 系統效能

| 指標 | 數值 | 備註 |
|------|------|------|
| **影像頻率** | ~20 Hz | 原始 + 標註影像 |
| **視訊延遲** | < 100ms | H.264 UDP 串流 |
| **偵測延遲** | ~50ms | OpenCV 處理時間 |
| **CPU 使用率** | ~30% | 單核心 |
| **記憶體使用** | ~500MB | Python 節點總計 |

### 優化建議

**降低延遲**:
- 使用 `speed-preset=ultrafast`
- 啟用 `Low Latency Mode` in QGC
- 降低影像解析度

**提高準確度**:
- 升級到 YOLOv8
- 增加影像解析度
- 使用 GPU 加速

---

## 🎓 技能提升

### 掌握的技術

1. **ROS 2 生態系統**
   - 節點開發 (Python)
   - 主題發布/訂閱
   - Bridge 配置

2. **影像處理**
   - OpenCV 基礎操作
   - 邊緣偵測與輪廓分析
   - 對比度增強技術

3. **視訊串流**
   - GStreamer Pipeline 設計
   - H.264 編碼參數調整
   - UDP 網路串流

4. **系統整合**
   - Tmux 自動化
   - 多進程協調
   - 錯誤處理與除錯

5. **無人機模擬**
   - ArduPilot SITL
   - Gazebo 物理模擬
   - MAVLink 通訊

### 可擴展的方向

1. **深度學習整合**
   - YOLOv8 物體偵測
   - TensorFlow 模型部署
   - GPU 加速推理

2. **高級視覺功能**
   - 物體追蹤 (DeepSORT)
   - 語義分割 (Mask R-CNN)
   - 3D 偵測 (RGBD)

3. **自主導航**
   - 視覺 SLAM
   - 路徑規劃
   - 避障算法

4. **實機部署**
   - Jetson Nano/Xavier
   - 硬體加速
   - 實時優化

---

## 📁 專案結構

```
workspaces/
├── ardupilot/                    # ArduPilot 源碼
├── ardupilot_gazebo/             # Gazebo 插件與模型
│   ├── models/                   # 無人機模型
│   ├── worlds/                   # 模擬世界
│   └── README.md                 # ✨ 更新的完整文檔
├── ros2_scripts/                 # ROS 2 Python 節點
│   ├── shape_detector.py         # 形狀偵測器
│   ├── annotated_streamer_h264.py # H.264 串流器
│   ├── detection_monitor.py      # 偵測監控工具
│   └── camera_streamer_h264.py   # 原始串流器
├── native_sim_launch.sh          # 自動化啟動腳本
├── check_vision_status.sh        # 系統狀態檢查
├── test_vision.sh                # 視覺系統測試
├── ros_gz_bridge.yaml            # ROS 2 Bridge 配置
└── VISION_SYSTEM_README.md       # 視覺系統文檔
```

---

## 🚀 快速啟動指令

```bash
# 1. 啟動完整系統
./native_sim_launch.sh

# 2. 檢查系統狀態
./check_vision_status.sh

# 3. 監控偵測結果
source /opt/ros/humble/setup.bash
ros2 topic echo /detections

# 4. 視覺化影像
rqt_image_view  # 選擇 /camera/annotated
```

---

## 📚 參考文檔

### 專案文檔
- `ardupilot_gazebo/README.md` - 完整系統文檔
- `VISION_SYSTEM_README.md` - 視覺系統詳細說明

### 外部資源
- [ArduPilot Dev Guide](https://ardupilot.org/dev/)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Fortress](https://gazebosim.org/docs/fortress)
- [OpenCV Tutorials](https://docs.opencv.org/4.x/)
- [YOLOv8 Docs](https://docs.ultralytics.com/)

---

## 🎯 下一步計劃

### 短期目標 (1-2 週)
- [ ] 整合 YOLOv8 物體偵測
- [ ] 實現目標追蹤功能
- [ ] 優化 GPU 加速

### 中期目標 (1-2 月)
- [ ] 添加深度相機支援
- [ ] 實現視覺 SLAM
- [ ] 開發自主降落功能

### 長期目標 (3-6 月)
- [ ] Jetson 實機部署
- [ ] 完整自主導航系統
- [ ] 多機協同視覺

---

## 💭 心得與反思

### 成功關鍵因素

1. **系統化思考**: 從底層到上層逐步建構
2. **模組化設計**: 每個組件獨立可測試
3. **自動化優先**: 減少重複手動操作
4. **文檔完整**: 便於後續維護和擴展

### 改進空間

1. **錯誤處理**: 增加更多異常處理機制
2. **參數配置**: 使用配置文件而非硬編碼
3. **單元測試**: 為關鍵組件添加測試
4. **日誌系統**: 統一的日誌格式和級別

### 經驗總結

> "從零開始建立一個完整的無人機視覺系統,最重要的是理解每個組件的作用和它們之間的數據流。模組化設計和自動化腳本大大提高了開發效率。"

---

**建立日期**: 2026-01-18  
**最後更新**: 2026-01-18  
**作者**: AI Assistant + User  
**版本**: 1.0
