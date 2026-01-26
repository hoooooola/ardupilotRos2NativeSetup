# PX4 架構全解：從工程師到架構師

## 🎯 學習地圖 (Learning Map)
不同於 ArduPilot 的實用主義，PX4 講究的是「軟體架構」與「模組化」。學習 PX4 就是在學習現代機器人作業系統的設計模式。

### 📚 章節規劃
1.  **[01_Middleware.md](01_Middleware.md)** - uORB 通訊機制：理解微服務架構 (Microservices)
2.  **[02_Control_Architecture.md](02_Control_Architecture.md)** - 串級控制與混合器 (Control Allocator)
3.  **[03_Simulation.md](03_Simulation.md)** - SITL 與 Gazebo 整合 (Hardware in the Loop)
4.  **[04_ROS2_Integration.md](04_ROS2_Integration.md)** - uXRCE-DDS 與 Offboard Control
5.  **[05_Custom_Airframe.md](05_Custom_Airframe.md)** - 實戰：定義自己的 VTOL 機型

## 🚀 快速啟動程序 (Quick Start Procedure)
### 自動化啟動腳本 (Recommended)
專案根目錄已建立自動化腳本，單鍵啟動 PX4 SITL + Gazebo Classic + QGroundControl + DDS Agent。

```bash
cd /media/user/Linux_Extra/workspaces
./native_sim_launchPX4.sh
```

**腳本功能：**
1.  **DDS Agent**: 自動啟動 Micro XRCE-DDS Agent (埠號 8888)，建立 ROS 2 通訊橋樑。
2.  **QGroundControl**: 自動背景執行 QGC。
3.  **PX4 SITL**: 編譯並執行 PX4 核心，連接至 Gazebo Classic 模擬環境。
4.  **Auto Cleanup**: 按 `Ctrl+C` 結束時，自動關閉所有相關後台程序。
