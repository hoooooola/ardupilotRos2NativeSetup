# ArduPilot QuadPlane (4+1) 學習地圖 (機械控制組視角)

這份地圖是為了先掌握 **Standard VTOL (QuadPlane)** 架構而設計。這是目前最穩定、業界應用最廣的構型，適合快速上手與建立基礎。

## Phase 1: 概念與架構 (The Architecture)
> **目標**：理解 4+1 這種「混血」控制邏輯。
*   **雙系統共存**: 4+1 本質上是「一台多旋翼 (Copter)」背著「一台固定翼 (Plane)」。
*   **控制權移交**:
    *   **VTOL 模式**: 多旋翼馬達工作，固定翼馬達停轉 (或輔助)。
    *   **Fixed-Wing 模式**: 固定翼馬達工作，多旋翼馬達停轉 (被視為死重 Dead Weight)。
    *   **混合階段 (Transition)**: 兩者同時工作（這是最危險的階段）。

## Phase 2: 關鍵飛行模式 (Flight Modes)
> **目標**：學會操作這兩種截然不同的物理特性。
*   **Q_ 模式 (Quad 模式)**:
    *   `Q_STABILIZE` / `Q_LOITER`: 像飛大疆一樣操作它。這時候它是純多旋翼物理特性。
    *   `Q_RTL`: 垂直起降的自動返航（非常重要，很多炸機發生在切不回來）。
*   **Plane 模式**:
    *   `FBWA` (Fly-By-Wire A): 飛行員控制滾轉/俯仰角度，飛控保持高度。這用來測試氣動力特性。
    *   `AUTO`: 執行推滿油門的 Waypoint 任務。

## Phase 3: 轉換邏輯 (Transition) - 核心必學
> **目標**：這是 VTOL 唯一需要「控制組」專業的地方。
*   **VTOL to Plane (起飛轉平飛)**:
    *   觸發條件：空速 (Airspeed) > `ARSPD_FBW_MIN`。
    *   物理過程：多旋翼維持高度 -> 推力馬達加速 -> 機翼產生升力 -> 多旋翼減出力 -> 多旋翼停轉。
*   **Plane to VTOL (降落轉懸停)**:
    *   這是「背風降落」最容易失速的地方。
    *   觀察 `Q_ASSIST_SPEED` 參數：當空速低於此值，多旋翼強制介入救機。

## Phase 4: 模擬與參數 (SITL & Tuning)
> **目標**：在電腦上把轉換過程調順。
*   **環境搭建**:
    *   使用 `sim_vehicle.py -v ArduPlane -f quadplane` 啟動模擬。
*   **關鍵參數**:
    *   `Q_ENABLE = 1`: 開啟 VTOL 功能。
    *   `Q_FRAME_CLASS`: 設定為 1 (Quad)。
    *   `Q_M_PWM_TYPE`: 設定馬達協議 (DShot/Normal)。
    *   `Q_TILT_MASK`: (4+1 不用設，但要知道這是給 Tiltrotor 用的)。

## Phase 5: 任務規劃 (Mission Planning)
> **目標**：設計一個包含垂直起降的自動任務。
1.  **VTOL_TAKEOFF**: 垂直起飛到指定高度 (如 20m)。
2.  **WAYPOINT**: 第一個航點設遠一點 (如 200m)，觀察它如何加速並轉換。
3.  **VTOL_LAND**: 飛到頭頂，切回多旋翼模式，垂直降落。

## 下一步挑戰 (Next Capability)
學會 4+1 後，你就會發現它的**物理極限**：
*   多旋翼馬達在平飛時是死重與阻力來源。
*   抗風性受限於機翼面積與多旋翼軸距的妥協。
這時候，你就可以帶著這些經驗，無痛進階到 **Tiltrotor (傾轉旋翼)**，去解決上述的空氣動力學矛盾。


嘗試幫我搭建一個quadplane的模擬環境 原來的可不移除嘛？