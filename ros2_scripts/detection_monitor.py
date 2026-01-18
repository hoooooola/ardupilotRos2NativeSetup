#!/usr/bin/env python3
"""
測試形狀偵測器 - 在 Gazebo 中生成測試形狀
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DetectionMonitor(Node):
    def __init__(self):
        super().__init__('detection_monitor')
        
        # 訂閱偵測結果
        self.subscription = self.create_subscription(
            String,
            '/detections',
            self.detection_callback,
            10)
        
        self.get_logger().info('🎯 Detection Monitor Started!')
        self.get_logger().info('Listening for shape detections...')
        
    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            count = data.get('count', 0)
            detections = data.get('detections', [])
            
            if count > 0:
                self.get_logger().info(f'\n{"="*50}')
                self.get_logger().info(f'🔍 Detected {count} shape(s):')
                for i, det in enumerate(detections, 1):
                    shape = det.get('shape', 'unknown')
                    area = det.get('area', 0)
                    center = det.get('center', {})
                    self.get_logger().info(
                        f'  {i}. {shape.upper()} - '
                        f'Area: {area:.0f}px, '
                        f'Center: ({center.get("x", 0)}, {center.get("y", 0)})'
                    )
                self.get_logger().info(f'{"="*50}\n')
                
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')

def main(args=None):
    rclpy.init(args=args)
    monitor = DetectionMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
