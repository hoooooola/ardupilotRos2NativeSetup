## info
- 自動調優功能適用於所有軸心運動性能和控制力都較為均衡的機架
- autotune有問題時才需要Manual tuning 
    -  MC PID Tuning (Manual/Basic) — Manual tuning basic how to.
    -  **MC PID Tuning Guide (Manual/Detailed)** — Manual tuning with detailed explanation (手動調校詳細指南)。從原理到實作完整的調參流程與物理意義分析。
- **MC Filter/Control Latency Tuning** (濾波與延遲調校) — Trade off control latency and noise filtering.
    - 在「控制響應速度 (Latency)」與「雜訊濾波 (Noise Filtering)」之間做取捨。濾波越強，雜訊越少但延遲越高 (飛起來手感笨重)。
- **MC Setpoint Tuning (Trajectory Generator)** (設定點/軌跡調校)
    - 調整無人機跟隨目標點的「手感」。例如 Stick 的靈敏度、最大加速度、以及煞車的快慢。
    - **MC Jerk-limited Type Trajectory** (加加速度限制軌跡)
      - 這是一種更平滑的軌跡類型。透過限制加速度的變化率 (Jerk)，讓飛行動作更柔順，減少機體震動與過衝。
- **Multicopter Racer Setup** (穿越機設定)
    - 針對高機動性、高響應需求的競速無人機進行的極限調校設定 (通常會犧牲部分穩定性換取反應速度)。

---
## autotune


