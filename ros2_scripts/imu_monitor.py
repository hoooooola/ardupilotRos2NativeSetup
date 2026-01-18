import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        # 訂閱 /imu 話題
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.subscription  # 防止變數被垃圾回收
        print("IMU Monitor Node has started! Waiting for data...")

    def imu_callback(self, msg):
        # 取得三軸加速度
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # 計算總加速度大小 (向量長度)
        total_accel = math.sqrt(ax**2 + ay**2 + az**2)

        # 這裡設定閾值為 15.0 m/s^2 (重力約 9.8)
        # 如果超過這個值，通常代表劇烈晃動或撞擊
        if total_accel > 15.0:
            print(f"⚠️  [COLLISION DETECTED] Acceleration: {total_accel:.2f} m/s^2")
        else:
            # 平常只印出目前數值 (為了不洗版，這裡可以選擇註解掉)
            # print(f"Normal: {total_accel:.2f}", end='\r')
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ImuMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
