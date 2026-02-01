# PX4 環境建置與編譯手冊 (Installation & Compilation Guide)

本手冊記錄如何在 **Ubuntu 22.04** + **ROS 2 Humble** 環境下，從零建置 PX4 開發環境。

## 📌 軟韌體版本規劃 (Target Versions)
- **PX4 Firmware**: v1.14.4 (Stable) - 為了解決與 ArduPilot 共存時的 Gazebo 版本衝突，選擇此版本以支援 Gazebo Classic。
- **Simulator**: Gazebo Classic (v11)
- **ROS 2**: Humble
- **OS**: Ubuntu 22.04 LTS

## 1. 下載源碼 (Download Source)
PX4 依賴大量子模組，建議下載 v1.14.4 穩定版。

```bash
cd /media/user/Linux_Extra/workspaces
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.14.4
git submodule update --init --recursive
# 清理可能存在的舊版編譯殘留
make distclean
```

## 2. 安裝依賴 (Install Dependencies)
針對 ROS 2 環境，需補足 Python 套件與 Gazebo Classic。

**Step A: 安裝基礎工具與 Gazebo Classic**
```bash
sudo apt update
sudo apt install -y python3-pip python3-kconfiglib python3-jinja2 python3-jsonschema \
    python3-future python3-cerberus python3-numpy \
    gcc-arm-none-eabi libncurses5-dev

# 安裝 Gazebo Classic (與 ArduPilot 高度相容)
sudo apt install -y gazebo libgazebo-dev
```

**Step B: 安裝 ROS 2 / DDS 橋接器 (Micro XRCE-DDS Agent)**
這是 PX4 連接 ROS 2 的關鍵組件。
```bash
# 如果尚未安裝
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

**Step C: 修復 Python 依賴**
```bash
pip3 install --user pyros-genmsg empy==3.3.4 packaging toml numpy matplotlib>=3.0
```

## 3. 編譯模擬器 (Compile SITL)
使用 `gazebo` (Gazebo Classic) 進行編譯與模擬。

```bash
cd /media/user/Linux_Extra/workspaces/PX4-Autopilot
# 編譯並啟動 SITL (Headless 模式，無圖形介面)
HEADLESS=1 make px4_sitl gazebo
```

## 4. 常見指令 (Cheat Sheet)

| 指令 | 用途 |
| :--- | :--- |
| `make px4_sitl` | 編譯預設模擬器 |
| `make px4_sitl gazebo-classic_iris` | 指定機型為 Iris (四旋翼) |
| `make px4_sitl jmavsim` | 使用 JMAVSim (輕量級模擬器) |
| `make clean` | 清除編譯暫存 (當出現奇怪錯誤時使用) |
| `make distclean` | 大掃除 (包含子模組重置) |


---

