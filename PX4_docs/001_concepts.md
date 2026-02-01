# 飛控與伴隨電腦架構 (FC and Companion Computer)

參考說明: [PX4 Systems Architecture](https://docs.px4.io/main/en/concept/px4_systems_architecture#fc-and-companion-computer)

![系統架構示意圖](image-6.png)





## 3. 現代無人機通訊數據流 (Modern Data Flow)

| 傳輸路徑 (Path) | 協議與技術 (Protocol/Tech) | 說明 |
| :--- | :--- | :--- |
| **FC 內部** | **uORB** | 飛控內部的訊息交換核心 (Message Bus) |
| **FC $\leftrightarrow$ 伴隨電腦** | **XRCE-DDS / ROS 2** | `PX4 (uORB)` $\to$ `DDS Bridge` $\to$ `ROS 2 Topics`<br>透過 uXRCE-DDS Client/Agent 橋接 |
| **伴隨電腦 $\leftrightarrow$ GCS (地面站)** | **MAVLink** (控制)<br>**WebRTC / RTSP** (影像) | 地面站主要使用 MAVLink，影像走專用串流協議 |
| **伴隨電腦 $\leftrightarrow$ Cloud (雲端)** | **MAVLink / Zenoh**<br>**WebRTC / RTSP** | Zenoh 適合更具彈性的雲端與邊緣通訊 |

## 4. 行業架構選型對比 (Architecture Strategy)

| 類型 | 代表案例 | 硬體平台 | 通訊核心 | 特點與適用場景 |
| :--- | :--- | :--- | :--- | :--- |
| **AI 邊緣運算型** | **Anduril** | **NVIDIA Jetson** | **Zenoh** | **高智能、自動目標識別**<br>適合高頻寬、低延遲的機載 AI 處理與多機協同。 |
| **標準化長續航型** | **Auterion** | **Qualcomm** | **MAVLink** | **4G/5G 遠程圖傳、標準化**<br>適合依賴現有生態系、長距離巡檢與物流應用。 |

---

# [PX4 Architectural Overview](https://docs.px4.io/main/en/concept/architecture#px4-architectural-overview)

PX4 由兩層組成： 
- flight stack, 是一個估計和飛行控制系統， 
- middleware, 是一個通用機器人層，可以支援任何類型的自主機器人，提供內部/外部通訊和硬體整合。

![alt text](image-7.png)

tips: 
- shell ' top '查看正在執行的模組
- <module_name> start/stop
- [Modules & Commands Reference](https://docs.px4.io/main/en/modules/modules_main)

Modules之間透過名為 [uORB](https://docs.px4.io/main/en/middleware/uorb) 的發布/訂閱訊息匯流排進行通訊
- 該系統是響應式的——它是非同步的，當有新資料可用時會立即更新
- 系統元件可以以線程安全的方式從任何位置使用資料

## Flight Stack 
![alt text](image-8.png)

controller variable三種狀態
- a setpoint and a measurement or estimated state
- flight stack control迴路分開看, ie. setpoint = 10m, measurement = 5m, error = 5m, 
  - controller = position controller
  - input = position & atti estimation + nav + rc
  - output = atti & rate controller

## Middleware
包含
- device drivers for embedded sensors
- communication with the external world (companion computer, GCS, etc.) and the uORB publish-subscribe message bus.
- [sim layer](https://docs.px4.io/main/en/simulation/),包含 SITL, HITL, etc.

## Sample data & Update Rates
- IMU取樣1kHz, 積分後以250Hz發布給navigator
- uorb top 指令即時[檢查](https://docs.px4.io/main/en/middleware/uorb)系統上的消息更新速率

## Runtime Environment 
- 模組間通訊（使用 uORB ）基於共享記憶體。整個 PX4 中間件運行在同一個位址空間中，即所有模組共享記憶體。
- 此系統設計使得只需付出最少的努力即可在單獨的位址空間中運行每個模組（需要更改的部分包括 uORB 、 parameter interface 、 dataman 和 perf

PX4 中模組有兩種執行方式：

| 特性 | Tasks (獨立任務) | Work Queue Tasks (工作佇列任務) |
| :--- | :--- | :--- |
| **ie.** | 要「決策」, SLAM, 重型運算, LOG | 只是「接收」數據, IMU, Navigator, Attitude controller |
| **資源與堆疊** | 擁有獨立的 stack 與優先權 | 共享相同的 stack 與執行緒優先權 |
| **記憶體佔用** | 較高 (每個任務獨立) | 較低 (共享資源) |
| **上下文切換** | 較頻繁 | 較少 (更有效率) |
| **限制** | 可執行 Blocking IO、Sleep | **不可** Sleep、Poll 或 Blocking IO |
| **適用場景** | 重型運算、需長時間運行或等待的任務 | 輕量級、週期性、事件驅動的各類驅動程式 |

### Work Queue 機制說明
- **協作式多工 (Co-operative behavior)**: 所有工作佇列任務必須互相協作，不能互相干擾。
- **排程 (Scheduling)**: 可透過指定未來的固定時間或 uORB topic update callback 進行調度。
- **多重佇列**: 一個系統中可以有多個工作佇列，每個佇列可運行多個任務。

### 監控與指令 (Monitoring)
> **INFO**: 正在工作佇列中執行的任務**不會**顯示在 `top` 指令中。

- `top`: 只能看到工作佇列本身 (例如 `wq:lp_default`)。
- `work_queue status`: 使用此指令來顯示所有活動的工作佇列項目。
---
## Controller Diagrams 
- [專業術語](https://docs.px4.io/main/en/contribute/notation#terminology)

Multicopter Control Architecture
![alt text](image-9.png)

Multicopter Angular Rate Controller
![alt text](image-10.png)

Multicopter Attitude Controller
![alt text](image-11.png)

Multicopter Acceleration to Thrust and Attitude Setpoint Conversion
- PID controller to stabilise velocity. Commands an acceleration.
- The integrator includes an anti-reset windup (ARW) using a clamping method.
- The commanded acceleration is NOT saturated - a saturation will be applied to the converted thrust setpoints in combination with the maximum tilt angle.
- Horizontal gains set via parameter MPC_XY_VEL_P_ACC, MPC_XY_VEL_I_ACC and MPC_XY_VEL_D_ACC.
- Vertical gains set via parameter MPC_Z_VEL_P_ACC, MPC_Z_VEL_I_ACC and MPC_Z_VEL_D_ACC.

Multicopter Position Controller
![alt text](image-12.png)

Combined Position and Velocity Controller Diagram
![alt text](image-13.png)