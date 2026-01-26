# Chapter 1: uORB 中介軟體 (Middleware)

> **一句話解釋**: uORB 就是飛控內部的「佈告欄系統」。

## 1. 核心概念 (Concept)
在 PX4 中，沒有「全域變數 (Global Variables)」。所有的資料交換都必須透過 **uORB (Micro Object Request Broker)**。

*   **Publisher (發布者)**: 感測器驅動程式 (如 IMU) 讀到數據後，貼一張紙條在佈告欄：「現在加速度是 9.8」。
*   **Subscriber (訂閱者)**: 姿態控制器 (Attitude Controller) 隨時盯著佈告欄，看到新紙條就拿去算。
*   **Topic (主題)**: 紙條的標題 (例如 `vehicle_acceleration`, `vehicle_attitude`)。

## 2. 為什麼要這樣設計？ (Why?)
*   **解耦合 (Decoupling)**: 寫控制器的人根本不需要知道 IMU 是哪一家廠牌的，他只要訂閱 `sensor_combined` 就好。
*   **執行緒安全 (Thread Safe)**: uORB 自動處理了多執行緒搶資源的問題。
*   **日誌 (Logging)**: 因為所有數據都經過 uORB，系統可以自動把所有 Topic 存成 `.ulg` 檔，不需要手寫 Log 程式碼。

## 3. 關鍵 Topic 清單 (Cheat Sheet)
作為控制組，這幾個 Topic 你一定要熟：

| Topic 名稱 | 內容 | 誰發布？ | 誰訂閱？ |
| :--- | :--- | :--- | :--- |
| `sensor_combined` | 原始 IMU 數據 | 驅動程式 | EKF2 (濾波器) |
| `vehicle_attitude` | 目前姿態 (四元數) | EKF2 | 姿態控制器 |
| `vehicle_local_position` | 目前位置 (XYZ) | EKF2 | 導航控制器 |
| `actuator_motors` | 馬達輸出指令 (0~1) | 混控器 (Mixer) | 馬達驅動 |
| `vehicle_command` | 外部命令 (MAVLink/ROS) | MAVLink/DDS | 任務管理器 |

## 4. 實戰指令 (NuttX Shell)
在 PX4 Console (MAVLink Console) 中可以即時監看：

```bash
# 監聽姿態數據 (即時印出數值)
listener vehicle_attitude

# 查看有哪些 Topic 正在跑
uorb top
```
