# 📚 文檔索引

本資料夾包含 ArduPilot ROS 2 視覺系統的完整文檔。

---

## 📖 文檔列表

### 1. [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
**快速參考卡** - 最常用的指令和除錯步驟

**適合對象**: 所有使用者  
**內容**:
- 一鍵啟動指令
- ROS 2 主題操作
- 常見問題快速修復
- 系統除錯指令

---

### 2. [VISION_SYSTEM_README.md](VISION_SYSTEM_README.md)
**視覺系統完整文檔** - AI 視覺系統的詳細說明

**適合對象**: 進階使用者、開發者  
**內容**:
- 系統架構詳解
- OpenCV 形狀偵測原理
- YOLO/TensorFlow 整合指南
- Jetson 硬體加速
- 視覺導航範例
- 效能優化技巧

---

### 3. [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)
**專案學習總結** - 完整的開發歷程和經驗總結

**適合對象**: 學習者、研究者  
**內容**:
- 專案時間軸
- 系統架構圖
- 關鍵技術學習
- 遇到的問題與解決方案
- 效能指標
- 技能提升總結
- 未來規劃

---

## 🎯 推薦閱讀順序

### 初學者路徑
1. 先閱讀 [../README.md](../README.md) 了解專案概況
2. 使用 [QUICK_REFERENCE.md](QUICK_REFERENCE.md) 快速上手
3. 運行 `./native_sim_launch.sh` 體驗系統

### 進階使用者路徑
1. 閱讀 [VISION_SYSTEM_README.md](VISION_SYSTEM_README.md) 了解視覺系統
2. 修改 `ros2_scripts/shape_detector.py` 調整參數
3. 整合 YOLO 進行物體偵測

### 開發者路徑
1. 詳細研讀 [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)
2. 理解系統架構和數據流
3. 開發自定義視覺算法
4. 參考 [../ardupilot_gazebo/README.md](../ardupilot_gazebo/README.md) 進行深度整合

---

## 🔗 相關文檔

### 專案文檔
- [../README.md](../README.md) - 專案入口
- [../ardupilot_gazebo/README.md](../ardupilot_gazebo/README.md) - ArduPilot Gazebo 插件文檔

### ROS 2 腳本
- [../ros2_scripts/](../ros2_scripts/) - ROS 2 Python 節點
  - `shape_detector.py` - 形狀偵測器
  - `annotated_streamer_h264.py` - H.264 視訊串流器
  - `detection_monitor.py` - 偵測結果監控
  - `camera_streamer_h264.py` - 原始串流器

### 啟動腳本
- [../native_sim_launch.sh](../native_sim_launch.sh) - 自動化啟動腳本
- [../check_vision_status.sh](../check_vision_status.sh) - 系統狀態檢查
- [../test_vision.sh](../test_vision.sh) - 視覺系統測試

---

## 📝 文檔維護

### 更新頻率
- **QUICK_REFERENCE.md**: 隨新功能即時更新
- **VISION_SYSTEM_README.md**: 重大功能變更時更新
- **PROJECT_SUMMARY.md**: 專案里程碑時更新

### 貢獻指南
歡迎提交文檔改進建議!請確保:
- 使用清晰的中文說明
- 提供可執行的程式碼範例
- 包含必要的截圖或圖表
- 遵循現有的文檔格式

---

**最後更新**: 2026-01-18  
**維護者**: User + AI Assistant
