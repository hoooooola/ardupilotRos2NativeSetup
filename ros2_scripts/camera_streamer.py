#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import io

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 訂閱相機話題
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info('Camera Streamer Node Started! 📹')
        self.get_logger().info('MJPEG Stream available at: http://localhost:8080/stream')
        
    def image_callback(self, msg):
        # 將 ROS Image 轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        with self.frame_lock:
            self.latest_frame = cv_image

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            
            while True:
                try:
                    with streamer_node.frame_lock:
                        if streamer_node.latest_frame is not None:
                            frame = streamer_node.latest_frame.copy()
                        else:
                            continue
                    
                    # 編碼為 JPEG
                    ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ret:
                        continue
                    
                    # 發送 MJPEG 幀
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b"\r\n")
                    
                except Exception as e:
                    streamer_node.get_logger().error(f'Stream error: {e}')
                    break
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        # 抑制 HTTP 請求日誌
        pass

def main(args=None):
    global streamer_node
    
    rclpy.init(args=args)
    streamer_node = CameraStreamer()
    
    # 啟動 HTTP Server 在背景執行緒
    server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    
    try:
        rclpy.spin(streamer_node)
    except KeyboardInterrupt:
        pass
    finally:
        streamer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
