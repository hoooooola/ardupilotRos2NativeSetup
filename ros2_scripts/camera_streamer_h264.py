#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np

class CameraStreamerH264(Node):
    def __init__(self):
        super().__init__('camera_streamer_h264')
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # 訂閱相機話題
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 使用更穩定的 GStreamer Pipeline
        gst_pipeline = (
            'fdsrc ! '
            'videoparse width=640 height=480 format=bgr framerate=10/1 ! '
            'videoconvert ! '
            'x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=10 ! '
            'rtph264pay config-interval=1 pt=96 ! '
            'udpsink host=127.0.0.1 port=5600 sync=false'
        )
        
        self.gst_process = subprocess.Popen(
            ['gst-launch-1.0'] + gst_pipeline.split(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        self.get_logger().info('Camera H.264 Streamer Started! 📹')
        self.get_logger().info('UDP H.264 Stream: udp://127.0.0.1:5600')
        self.get_logger().info('Configure QGC: Source=UDP h.264, Port=5600')
        self.get_logger().info('Waiting for camera images...')
        
    def image_callback(self, msg):
        try:
            # 將 ROS Image 轉換為 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 確保尺寸正確 (640x480)
            if cv_image.shape[:2] != (480, 640):
                cv_image = cv2.resize(cv_image, (640, 480))
            
            # 寫入 GStreamer stdin
            self.gst_process.stdin.write(cv_image.tobytes())
            self.gst_process.stdin.flush()
            
            self.frame_count += 1
            if self.frame_count % 50 == 0:
                self.get_logger().info(f'Streaming... {self.frame_count} frames sent')
            
        except Exception as e:
            self.get_logger().error(f'Stream error: {e}')
    
    def destroy_node(self):
        if self.gst_process:
            self.gst_process.terminate()
            self.gst_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    streamer_node = CameraStreamerH264()
    
    try:
        rclpy.spin(streamer_node)
    except KeyboardInterrupt:
        pass
    finally:
        streamer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
