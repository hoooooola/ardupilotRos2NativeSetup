


# ArduPilot Copter 學習地圖 (機械控制組視角)

這份地圖是為了機械系控制組量身打造，省略了純軟體工程的細節（如驅動寫法），專注於**動力學**、**控制理論**與**系統識別**。

## Phase 1: 物理模型與致動 (The Plant)
> **目標**：理解被控體（多旋翼機）的物理特性。
- **剛體動力學 (Rigid Body Dynamics)**
  - 理解推力 (Thrust)、反扭矩 (Torque) 與轉動慣量 (Inertia Matrix) 的關係。
  - **Motor Mixer**: 理解 `Roll/Pitch/Yaw` 力矩指令如何被「解算」分配給 4 顆（或更多）馬達。
- **感測器特性 (Sensors)**
  - **IMU**: 了解加速度計與陀螺儀的頻寬與雜訊（高頻振動對控制的影響）。
  - **GPS/Barometer**: 了解低頻感測器的延遲與飄移。

## Phase 2: 串級控制架構 (Cascade Control Architecture)
> **核心價值**：這是 ArduPilot 的靈魂，也是控制組最能發揮的地方。
由內而外，頻率由高到低：

### 1. 角速度環 (Rate Loop, Fast Loop) - `400Hz+`
*   **輸入**：期望角速度 (Target Rate)
*   **回授**：陀螺儀原始數據 (Gyro Raw)
*   **控制器**：PID + FeedForward (前饋)
*   **輸出**：PWM 訊號 (給馬達電變)
*   **重點**：這是讓飛機「鎖住」姿態的關鍵。PID 參數調整的起點。

### 2. 姿態環 (Attitude Loop/Stabilize) - `100Hz+`
*   **輸入**：期望角度 (Target Euler Angle, from RC stick or Nav)
*   **回授**：EKF 估算姿態 (AHRS)
*   **控制器**：P Controller (比例控制將角度誤差轉為角速度命令)
*   **輸出**：期望角速度 -> **傳給內層**

### 3. 位置/導航環 (Position/Navigation Loop) - `10-50Hz`
*   **輸入**：目標座標 (Waypoint) 或 期望速度 (Velocity Command)
*   **回授**：EKF 融合之位置與速度 (GPS + IMU)
*   **控制器**：PID (將位置誤差轉為期望傾角 Lean Angle)
*   **輸出**：期望角度 -> **傳給姿態環**

## Phase 3: 狀態估計 (State Estimation - EKF)
> **目標**：在雜訊中還原真實狀態。
- **EKF3 (Extended Kalman Filter)**
  - 理解如何融合 **高頻/低精確度** (IMU) 與 **低頻/高精確度** (GPS) 訊號。
  - **Vibration Control**: 震動是控制的敵人。學習使用 **Notch Filter (陷波濾波器)** 濾除機架共振頻率。

## Phase 4: 飛行模式邏輯 (Flight Modes)
> **目標**：理解不同控制層級的介入時機。
- **Stabilize**: 純手動，僅介入 Rate/Attitude Loop（像開賽車）。
- **Loiter (GPS定點)**: 介入 Position Loop（像開有輔助駕駛的車）。
- **Guided (外部控制)**: 這是接 ROS 2 的關鍵模式，允許外部電腦直接對 **位置環** 或 **姿態環** 下命令。

## Phase 5: 驗證與調校 (Verification & Tuning)
- **SITL (Software In The Loop)**: 在電腦上驗證控制律，不必炸機。
- **Log 分析 (DataFlash Logs)**
  - **追隨性分析**: 把 `ATT.DesRoll` (期望) 和 `ATT.Roll` (實際) 疊在同一張圖看。
  - **輸出飽和**: 檢查 PWM 是否頻繁頂到最大值（動力不足或震盪）。
