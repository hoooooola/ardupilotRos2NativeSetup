# ROS 2 架構觀念地圖 (Concept Map)

![ROS 2 Node Graph Visualization](image-1.png)
*圖示：使用 rqt_graph 視覺化 Gazebo 相機與 Shape Detector 之間的數據流向*


這是一份啟發式筆記，用於快速回憶 ROS 2 核心架構。

---

## 🏗️ 實體層 (Entities) - "Who?"
> 系統中的「員工」，負責執行具體工作。

### 關鍵字 (Key Words)
*   **Node (節點)**: 最小執行單位 (Process)。
    *   *特徵*: 專注做一件事 (e.g., 相機驅動、形狀偵測)。
    *   *啟發*: 就像辦公室裡的單一員工。
*   **Discovery**: 自動發現機制。
    *   *特徵*: 節點啟動後會自動互相看見，不需手動連線。

---

## 📡 溝通層 (Communication) - "How?"
> 員工之間傳遞資訊的方式。

### Pattern A: 廣播模式 (Streaming)
*   **Topic (話題)**: 公佈欄 / 廣播頻道。
    *   **Publisher (發布者)**: 拿大聲公講話的人。 (e.g., Gazebo 相機)
    *   **Subscriber (訂閱者)**: 在台下聽講的人。 (e.g., Shape Detector)
    *   **Message (訊息)**: 傳遞的內容 (Data Type)。
        *   *Strongly Typed (強型別)*: 發送格式必須 = 接收格式 (e.g., `sensor_msgs/msg/Image`)。
    *   **QoS (服務品質)**:
        *   `RELIABLE` (掛號信): 重要指令，不可掉包。
        *   `BEST_EFFORT` (平信): 影像串流，掉了就算了。

### Pattern B: 請求模式 (Call/Response) - *Next Step*
*   **Service (服務)**: 打電話 / 點餐。
    *   *特徵*: 雙向確認 (Request -> Response)。常用於改變狀態 (e.g., 起飛、解鎖)。

---

## 🛠️ 工具層 (Tools) - "Observe"
> 上帝視角，用來監控和除錯。

### 視覺化 (Visualization)
*   **`rqt_graph`**: 關係圖。
    *   *功能*: 畫出 Node 和 Topic 的連線圖。
    *   *盲點*: 預設會隱藏 Dead sinks (沒人接的線) 和 System topics。

### 聽診器 (Inspection)
*   **`ros2 topic list`**: 列出所有頻道。
*   **`ros2 topic echo /<name>`**: 偷聽內容 (即時印出數據)。
*   **`ros2 topic hz /<name>`**: 量測頻率 (心跳聲)。
*   **`ros2 topic info /<name>`**: 查身家 (誰發的？誰收的？Type 是什麼？)。

### Log 系統 (Logging)
*   **`/rosout`**: 系統總廣播。
    *   *功能*: 收集所有節點的 print/error log。
*   **`rqt_console`**: 專門看 Log 的過濾器。
- 想要一個現代化、好看、功能強大的綜合控制台來取代 rqt：選 Foxglove Studio。
- 分析波形、PID 調試、看數據曲線：選 PlotJuggler。
- 3D 點雲、地圖：繼續用 RViz2。
---

## 🚀 進階概念 (Pro Concepts) - "Architect"
> 架構師思維，如何設計大型系統。

1.  **Remapping (接線生)**
    *   *核心*: **Decoupling (解耦)**。程式碼寫 `sub("input")`，執行時才決定 `input` 是 `/camera/front` 還是 `/camera/back`。
2.  **Namespace (資料夾)**
    *   *核心*: **Organization (組織)**。用 `/drone1/...` 把相關話題包起來，避免撞名。
3.  **Interface (.msg)**
    *   *核心*: **Contract (合約)**。定義大家溝通的語言格式。

---

## 實戰流程 (Workflow)
1.  **List**: 先看有哪些話題 (`topic list`)
2.  **Graph**: 看誰連給誰 (`rqt_graph`) -> *發現斷線?*
3.  **Info**: 查格式對不對 (`topic info`)
4.  **Interface**: 查格式裡面長怎樣 (`interface show`)
5.  **Echo**: 真的有數據在跑嗎？ (`topic echo`)
