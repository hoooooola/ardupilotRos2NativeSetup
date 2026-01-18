# 🤖 ArduPilot ROS 2 視覺系統

## 📋 系統架構

```
Gazebo 相機
    ↓
ROS 2 Bridge (/camera/image_raw)
    ↓
形狀偵測器 (shape_detector.py)
    ↓
標註影像 (/camera/annotated) + 偵測結果 (/detections)
    ↓
H.264 串流器 → QGroundControl (UDP:5600)
```

## 🚀 快速開始

### 1. 自動啟動 (推薦)
```bash
./native_sim_launch.sh
```

這會自動啟動:
- ✅ ArduPilot SITL
- ✅ Gazebo Fortress
- ✅ ROS 2 Bridge
- ✅ 形狀偵測器
- ✅ H.264 視訊串流到 QGC

### 2. 手動測試個別組件

#### 測試形狀偵測器:
```bash
source /opt/ros/humble/setup.bash
python3 ros2_scripts/shape_detector.py
```

#### 監控偵測結果:
```bash
source /opt/ros/humble/setup.bash
python3 ros2_scripts/detection_monitor.py
```

#### 測試 H.264 串流:
```bash
source /opt/ros/humble/setup.bash
python3 ros2_scripts/annotated_streamer_h264.py
```

## 🎯 目前功能

### 形狀偵測 (OpenCV)
- ✅ 三角形 (Triangle)
- ✅ 正方形 (Square)
- ✅ 矩形 (Rectangle)
- ✅ 五邊形 (Pentagon)
- ✅ 六邊形 (Hexagon)
- ✅ 圓形 (Circle)

### 輸出資訊
- 形狀類型
- 面積大小
- 中心座標
- 頂點數量

## 🧠 升級到深度學習 (YOLO/TensorFlow)

### 方案 1: YOLO (推薦用於即時物體偵測)

#### 安裝 YOLOv8:
```bash
pip3 install ultralytics
```

#### 修改 `shape_detector.py`:
```python
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        # 載入預訓練模型
        self.model = YOLO('yolov8n.pt')  # nano 版本,速度快
        
    def detect_objects(self, image):
        results = self.model(image, conf=0.5)
        detections = []
        
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                
                detections.append({
                    "class": self.model.names[cls],
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })
        
        return detections
```

### 方案 2: TensorFlow Object Detection

#### 安裝:
```bash
pip3 install tensorflow opencv-python
```

#### 使用預訓練模型:
```python
import tensorflow as tf

# 載入 SSD MobileNet
model = tf.saved_model.load('ssd_mobilenet_v2/saved_model')
```

### 方案 3: Jetson Inference (如果在 Jetson 上運行)

您提到的 Isaac/Jetpack inference 腳本可以這樣整合:

```python
import jetson.inference
import jetson.utils

class JetsonDetector(Node):
    def __init__(self):
        super().__init__('jetson_detector')
        # 使用 Jetson 硬體加速
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
        
    def detect_objects(self, cv_image):
        # 轉換為 CUDA 格式
        cuda_img = jetson.utils.cudaFromNumpy(cv_image)
        
        # 執行偵測
        detections = self.net.Detect(cuda_img)
        
        results = []
        for detection in detections:
            results.append({
                "class": self.net.GetClassDesc(detection.ClassID),
                "confidence": detection.Confidence,
                "bbox": [detection.Left, detection.Top, 
                        detection.Right, detection.Bottom]
            })
        
        return results
```

## 📊 ROS 2 主題

### 訂閱:
- `/camera/image_raw` - 原始相機影像

### 發布:
- `/camera/annotated` - 標註後的影像
- `/detections` - JSON 格式的偵測結果

### 查看主題:
```bash
# 列出所有主題
ros2 topic list

# 查看偵測結果
ros2 topic echo /detections

# 查看影像頻率
ros2 topic hz /camera/image_raw
```

## 🎮 整合無人機控制

### 建立決策節點:
```python
class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        
        # 訂閱偵測結果
        self.create_subscription(String, '/detections', self.on_detection, 10)
        
        # 發布控制指令
        self.cmd_pub = self.create_publisher(...)
        
    def on_detection(self, msg):
        data = json.loads(msg.data)
        
        # 根據偵測結果做決策
        if "person" in [d["class"] for d in data["detections"]]:
            self.get_logger().warn("Person detected! Taking action...")
            # 發送控制指令
```

## 🔧 效能調整

### 降低延遲:
- 調整 H.264 bitrate (目前 2000 kbps)
- 使用 `speed-preset=ultrafast`
- 啟用 `Low Latency Mode` in QGC

### 提高準確度:
- 增加影像解析度 (需修改 Gazebo SDF)
- 調整偵測閾值
- 使用更大的 YOLO 模型 (yolov8m, yolov8l)

### GPU 加速:
```bash
# 檢查 CUDA 可用性
python3 -c "import torch; print(torch.cuda.is_available())"

# 使用 GPU 版本的 PyTorch
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

## 📝 下一步開發

1. **物體追蹤**: 加入 DeepSORT 或 ByteTrack
2. **語義分割**: 使用 Mask R-CNN 或 YOLACT
3. **3D 偵測**: 整合深度相機
4. **自主導航**: 根據視覺輸入規劃路徑

## 🐛 除錯

### 查看日誌:
```bash
# 在 tmux 中切換窗格
Ctrl+b, 然後按方向鍵

# 查看 ROS 2 日誌
ros2 node list
ros2 node info /shape_detector
```

### 視覺化:
```bash
# 使用 rqt_image_view 查看影像
rqt_image_view
```

## 📚 參考資源

- [YOLOv8 文檔](https://docs.ultralytics.com/)
- [TensorFlow Object Detection](https://github.com/tensorflow/models/tree/master/research/object_detection)
- [Jetson Inference](https://github.com/dusty-nv/jetson-inference)
- [ROS 2 Vision](https://github.com/ros-perception)
