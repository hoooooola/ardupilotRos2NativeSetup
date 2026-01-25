# 🚀 進階研發：ROS 2 飛行數據分析與系統識別

這是從單純「玩飛機」進階到「研發核心算法」的關鍵路徑。透過將飛行數據導入 ROS 2，你可以利用強大的 Python 生態系進行即時計算與分析。

## 1. 核心目標
*   **性能指標計算**: 用 Python (`rclpy`) 寫腳本，即時計算超調量 (Overshoot)、安定時間 (Settling Time) 等飛行性能指標。
*   **系統識別 (System ID)**: 直接將 `/mavros/imu/data` 餵給你的演算法，鑑別出機體的物理參數矩陣。

## 2. 建立數據橋樑 (Telemetry Bridge)
為了讓 ROS 2 收到 ArduPilot 的數據，需要在 SITL (MAVProxy) 端增加一個數據出口。

在 MAVProxy Console 輸入：
```bash
output add 127.0.0.1:14551
```
*(這會將 MAVLink 封包複製一份到 UDP 14551，供 MAVROS 接收)*

## 3. 啟動 ROS 2 接收端
使用 MAVROS 將 MAVLink 轉譯為 ROS 2 Topic。

*   **連接方式**: UDP 14550 或 14551
*   **推薦工具**: `mavros_launch.sh` (已建立於 workspace)

## 4. 關鍵數據主題 (Topics to Watch)
在 ROS 2 環境中 (`rqt` 或程式碼)，你可以訂閱以下主題：

| 數據類型 | Topic 名稱 | 備註 |
| :--- | :--- | :--- |
| **實際姿態** | `/mavros/local_position/pose` | 包含 Quaternion (orientation) |
| **期望姿態** | `/mavros/setpoint_raw/attitude` | 若由外部 ROS 控制時以此為準 |
| **IMU 原值** | `/mavros/imu/data` | 加速度與角速度，做 System ID 必備 |

## 5. 視覺化分析
使用 RQT Plot 即時畫圖：
```bash
rqt_plot /mavros/local_position/pose/pose/orientation/w
```
*(可自行切換為 roll/pitch/yaw 進行更直觀的分析)*