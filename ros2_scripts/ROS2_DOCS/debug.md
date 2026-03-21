# 專業 ROS 2 開發者 Debug 與 Trace 指南

## 👑 總結：推薦工作流
既然架構已經推進到 **PX4 + ROS 2 + Foxglove**，這已經是專業開發系統的標準配置。建議未來的 Debug 習慣是：

1. **輕微問題**：一律看 Foxglove 上的 `Console` 和 `Plot` 折線圖。
2. **想追究數值錯誤**：在終端機使用指令 `ros2 topic echo /特定的Topic名稱`。
3. **出門實測 (或遇難抓的 Bug)**：教這台機體在起飛時，自動執行 `ros2 bag record` 幫自己錄影（類似行車記錄器），回家後丟進 Foxglove 慢動作重播。

---

## 🛠 第一級：即時監控 (The Heartbeat) 💓
* **工具**：Foxglove Studio 或 `rqt_console`
* **作法**：當系統運行時，在 Foxglove 裡開啟 Console (或 Log) 面板，過濾出 `WARN` (警告) 以上的錯誤。藉此一眼看出問題的大方向（例如是硬體斷線還是演算法沒出結果）。

## 🕵️‍♂️ 第二級：資料流的斷點檢查 (The Pipeline Check) 
* **工具**：`ros2 topic echo` / Foxglove 3D & Plot 面板
* **作法**：當發生「沒有報錯，但行為就是不對」的情境時，沿著管線 (Pipeline) 檢查資訊在哪一段消失。

**情境範例： 相機 (A) ➔ AI 辨識 (B) ➔ 控制節點 (C)**
1. **檢查 A (相機)**：懷疑相機斷線？用指令 `ros2 topic hz /camera/image_raw` 檢查掉幀情況（正常約 30hz，0 就是相機壞了）。
2. **視覺化確認**：在 Foxglove 新增 Image Panel 看 `/camera/image_raw` 的畫面是不是全黑。
3. **檢查 B (AI)**：如果畫面正常，再看 AI 辨識節點有沒有發出 `/detections` 主題的結果。
> **結論**：這稱作「斷點排查」。ROS 2 Publisher/Subscriber 的通訊機制非常方便，可以在水管的任何中間環節「截聽」偷看資料。

## ⏪ 第三級：時光回溯系統 (The Time Machine)
* **工具**：MCAP 錄製 (`ros2 bag` / Foxglove Data Recording)
* **作法**：這是專業車廠/無人機公司最核心、最值錢的除錯方式，專門應對沒有連螢幕的戶外實體飛行測試。

**具體流程：**
1. **Bagging 錄包**：將重要 Topic（如相機畫面、感測器、所有 Error）全部存成 `.mcap` (或 `.db3`) 檔（指令：`ros2 bag record -a` 錄下全部）。
2. **重播抓蟲**：撿回機體後，將這幾 GB 的檔案丟進 Foxglove Studio。透過畫面底下的「時間軸 (Timeline)」，可以像看 YouTube 影片一樣快轉、倒轉或慢動作。1:1 在圖形介面上重現墜機前 5 秒鐘的所有感測器數字、3D 視野與 Error 日誌，直到精準找出哪一行邏輯寫錯！
