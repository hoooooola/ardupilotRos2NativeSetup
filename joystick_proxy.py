import evdev
from evdev import UInput, ecodes, AbsInfo
import sys

# 目標：建立一個標準的 8 軸虛擬搖桿，每個軸都是完美的 -32768 ~ 32767
TARGET_MIN = -32768
TARGET_MAX = 32767

# 1. 尋找真實裝置
print("🔍 Searching for source device (EdgeTX/Jumper)...")
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
source_device = None
for dev in devices:
    if "EdgeTX" in dev.name or "Jumper" in dev.name or "Radiomaster" in dev.name:
        source_device = dev
        print(f"✅ Found Source: {dev.name} ({dev.path})")
        break

if not source_device:
    print("❌ No device found. Please plug in your controller via USB.")
    sys.exit(1)

# 2. 準備虛擬裝置定義
# 強制宣告 8 個標準軸
target_axes = [
    ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_Z, 
    ecodes.ABS_RX, ecodes.ABS_RY, ecodes.ABS_RZ,
    ecodes.ABS_THROTTLE, ecodes.ABS_RUDDER
]

caps = {
    ecodes.EV_KEY: source_device.capabilities().get(ecodes.EV_KEY, []),
    ecodes.EV_ABS: []
}

# 為每個目標軸設定標準範圍
for axis in target_axes:
    caps[ecodes.EV_ABS].append(
        (axis, AbsInfo(value=0, min=TARGET_MIN, max=TARGET_MAX, fuzz=0, flat=0, resolution=0))
    )

# 3. 讀取來源裝置的範圍資訊 (用於校正)
axis_maps = {} # {source_code: (src_min, src_max, target_code)}

src_abs = source_device.capabilities().get(ecodes.EV_ABS, [])
src_codes = sorted([x[0] for x in src_abs])

print("\n📊 Mapping Configuration:")
for i, src_code in enumerate(src_codes):
    if i >= len(target_axes): break
    
    tgt_code = target_axes[i]
    abs_info = source_device.absinfo(src_code)
    
    # 保存映射關係
    axis_maps[src_code] = {
        'min': abs_info.min,
        'max': abs_info.max,
        'target': tgt_code
    }
    print(f"  Src Axis {src_code} [{abs_info.min}, {abs_info.max}] -> Tgt Axis {tgt_code}")

# 4. 啟動虛擬裝置
new_name = "Generic Virtual Joystick V2"
try:
    ui = UInput(events=caps, name=new_name, version=0x1, vendor=0x1234, product=0x5678)
    print(f"\n🚀 {new_name} is LIVE!")
    print("👉 Please select this new device in QGC and Calibrate.")
    print("🔄 Proxy running... (Ctrl+C to stop)")
    
    for event in source_device.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code in axis_maps:
                data = axis_maps[event.code]
                val = event.value
                
                # 計算正規化 (0.0 ~ 1.0)
                # 處理可能的除以零
                range_span = data['max'] - data['min']
                if range_span == 0: range_span = 1
                
                norm = (val - data['min']) / range_span
                
                # 放大到目標範圍 (-32768 ~ 32767)
                target_val = int(TARGET_MIN + (norm * (TARGET_MAX - TARGET_MIN)))
                
                # 安全限制
                target_val = max(TARGET_MIN, min(TARGET_MAX, target_val))
                
                ui.write(ecodes.EV_ABS, data['target'], target_val)
                ui.syn()
                
        elif event.type == ecodes.EV_KEY:
            ui.write(ecodes.EV_KEY, event.code, event.value)
            ui.syn()
            
except OSError as e:
    print(f"❌ Error: {e}")
except KeyboardInterrupt:
    print("\nStopping.")
