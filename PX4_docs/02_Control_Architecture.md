# Chapter 2: 控制架構 (Control Architecture)

> **這就是 PX4 的靈魂**：全模組化的串級控制鏈。

## 1. 數據流向 (The Pipeline)
不同於 ArduPilot 的緊密耦合，PX4 是一群獨立運作的 Process。

```mermaid
graph LR
    Nav[導航 Loop] -->|Trajectory Setpoint| Pos[位置 Loop]
    Pos -->|Attitude Setpoint| Att[姿態 Loop]
    Att -->|Rate Setpoint| Rate[角速度 Loop]
    Rate -->|Torque Setpoint| Alloc[控制分配 Allocator]
    Alloc -->|Actuator Output| Motor[馬達]
```

## 2. 關鍵模組詳解

### A. 導航與位置 (Navigator & Position Control)
*   **模組名稱**: `navigator`, `mc_pos_control`
*   **輸入**: 目標經緯度 / 搖桿輸入
*   **輸出**: `trajectory_setpoint` (期望軌跡)
*   **特點**: 這裡處理「平滑化 (S-Curve)」，讓無人機不會急停急煞。

### B. 姿態控制 (Attitude Control)
*   **模組名稱**: `mc_att_control`
*   **輸入**: `vehicle_attitude_setpoint` (期望姿態)
*   **輸出**: `vehicle_rates_setpoint` (期望角速度)
*   **控制律**: P Controller (基於四元數的誤差計算)。

### C. 角速度控制 (Rate Control)
*   **模組名稱**: `mc_rate_control`
*   **輸入**: `vehicle_rates_setpoint`
*   **輸出**: `vehicle_torque_setpoint` (歸一化力矩)
*   **控制律**: PID + FeedForward。這是調參最核心的地方。

### D. 控制分配 (Control Allocator) - **最強大之處**
*   **以前 (ArduPilot)**: 你要手動設 `SERVO1_FUNCTION = 33` (Motor 1)。
*   **現在 (PX4)**: 你只要告訴它**幾何形狀 (Geometry)**。
    *   例如：「我有 3 個旋翼，位置在 [0.2, 0], [-0.1, 0.1], [-0.1, -0.1]」。
    *   PX4 會**自動計算**如何混合這些馬達來產生 Roll/Pitch/Yaw 力矩。
    *   **優勢**: 對於 Tiltrotor (傾轉旋翼) 這種幾何形狀會隨時間改變的機型，它能動態調整矩陣。

## 3. 調參順序 (Tuning Guide)
1.  **Rate Loop (PID)**: 先讓飛機在 Rate 模式下不震盪。
2.  **Attitude Loop (P)**: 讓飛機還是平的。
3.  **Position Loop (PID)**: 最後才調 GPS 定點。
*(順序錯了，神仙難救)*
