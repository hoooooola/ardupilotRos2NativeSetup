#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        
        # 1. 訂閱無人機狀態 (State)
        self.current_state = State()
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10)
            
        # 2. 建立服務客戶端 (Service Clients)
        # 用於解鎖 (Arming)
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        # 用於切換模式 (Set Mode)
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        # 用於起飛 (Takeoff)
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        # 用於降落 (Land)
        self.land_client = self.create_client(CommandTOL, 'mavros/cmd/land')

        # 等待服務連線
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Arming service...')
        
        self.get_logger().info('Mission Control Node Initialized! 🚀')

    def state_cb(self, msg):
        self.current_state = msg

    def run_mission(self):
        # Step 0: 等待 MAVROS 連線
        while not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            time.sleep(1)

        # Step 1: 切換到 GUIDED 模式
        # GUIDED 是 ArduPilot 專門接受外部指令的模式 (類似 PX4 的 Offboard)
        self.get_logger().info('Setting mode to GUIDED...')
        req_mode = SetMode.Request()
        req_mode.custom_mode = "GUIDED"
        
        while self.current_state.mode != "GUIDED":
            self.set_mode_client.call_async(req_mode)
            time.sleep(2)
        self.get_logger().info('✅ Mode set to GUIDED!')

        # Step 2: 解鎖 (Arming)
        self.get_logger().info('Arming vehicle...')
        req_arm = CommandBool.Request()
        req_arm.value = True
        
        while not self.current_state.armed:
            self.arming_client.call_async(req_arm)
            time.sleep(2)
        self.get_logger().info('✅ Vehicle Armed!')

        # Step 3: 起飛 (Takeoff) 
        self.get_logger().info('Taking off to 5 meters...')
        req_takeoff = CommandTOL.Request()
        req_takeoff.altitude = 5.0  # 目標高度 5m
        req_takeoff.latitude = 0.0  # 0 代表使用當前位置
        req_takeoff.longitude = 0.0
        
        self.takeoff_client.call_async(req_takeoff)
        # 簡單等待起飛完成 (實際應用應檢查高度)
        time.sleep(10) 
        self.get_logger().info('✅ Takeoff Complete (Assumed)!')

        # Step 4: 懸停 (Hover)
        self.get_logger().info('Hovering for 10 seconds...')
        time.sleep(10)

        # Step 5: 降落 (Land)
        self.get_logger().info('Landing...')
        req_land = CommandTOL.Request()
        self.land_client.call_async(req_land)
        
        # 等待降落並鎖定
        while self.current_state.armed:
            self.get_logger().info('Waiting for disarm...', throttle_duration_sec=2)
            time.sleep(1)
            
        self.get_logger().info('✅ Mission Accomplished! Landed and Disarmed.')

def main(args=None):
    rclpy.init(args=args)
    mission_node = MissionControl()
    
    # 這裡我們不使用 rclpy.spin() 而是直接跑任務流程
    # 這是為了簡化範例，正式專案建議使用 State Machine
    try:
        # 啟動一個背景執行緒來處理 callback (這樣 state_cb 才會更新)
        import threading
        spinner = threading.Thread(target=rclpy.spin, args=(mission_node,))
        spinner.start()
        
        mission_node.run_mission()
        
        mission_node.destroy_node()
        rclpy.shutdown()
        spinner.join()
        
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
