#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np

class AnnotatedStreamerH264(Node):
    def __init__(self):
        super().__init__('annotated_streamer_h264')
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # 訂閱標註後的影像 (如果有形狀偵測器在運行)
        # 否則回退到原始影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/annotated',  # 優先使用標註影像
            self.image_callback,
            10)
        
        # 備用訂閱原始影像
        self.raw_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.raw_callback,
            10)
        
        # 使用穩定的 GStreamer Pipeline
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
        
        self.get_logger().info('📹 Annotated H.264 Streamer Started!')
        self.get_logger().info('UDP H.264 Stream: udp://127.0.0.1:5600')
        self.get_logger().info('Subscribing to: /camera/annotated (with fallback to /camera/image_raw)')
        self.get_logger().info('Configure QGC: Source=UDP h.264, Port=5600')
        
        self.last_frame_time = self.get_clock().now()
        self.use_annotated = False
        
    def image_callback(self, msg):
        """處理標註後的影像"""
        self.use_annotated = True
        self._stream_image(msg)
    
    def raw_callback(self, msg):
        """處理原始影像 (備用)"""
        if not self.use_annotated:
            self._stream_image(msg)
    
    def _stream_image(self, msg):
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
                source = "annotated" if self.use_annotated else "raw"
                self.get_logger().info(f'Streaming ({source})... {self.frame_count} frames sent')
            
        except Exception as e:
            self.get_logger().error(f'Stream error: {e}')
    
    def destroy_node(self):
        if self.gst_process:
            self.gst_process.terminate()
            self.gst_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    streamer_node = AnnotatedStreamerH264()
    
    try:
        rclpy.spin(streamer_node)
    except KeyboardInterrupt:
        pass
    finally:
        streamer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
