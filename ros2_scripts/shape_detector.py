#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')
        self.bridge = CvBridge()
        
        # 訂閱相機影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 發布偵測結果 (JSON 格式)
        self.detection_pub = self.create_publisher(String, '/detections', 10)
        
        # 發布標註後的影像 (用於視覺化)
        self.annotated_pub = self.create_publisher(Image, '/camera/annotated', 10)
        
        self.frame_count = 0
        self.get_logger().info('🔍 Shape Detector Started!')
        self.get_logger().info('Subscribing to: /camera/image_raw')
        self.get_logger().info('Publishing to: /detections, /camera/annotated')
        
    def detect_shapes(self, image):
        """使用 OpenCV 偵測形狀"""
        # 轉灰階
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 增強對比度 (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny 邊緣偵測 (降低閾值以偵測更多邊緣)
        edges = cv2.Canny(blurred, 30, 100)
        
        # 形態學操作 - 閉合邊緣
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)
        
        # 尋找輪廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        annotated = image.copy()
        
        for contour in contours:
            # 降低面積閾值,接受更小的物體
            area = cv2.contourArea(contour)
            if area < 100:  # 從 500 降到 100
                continue
            
            # 計算周長和近似多邊形
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            
            # 計算中心點
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            
            # 辨識形狀
            vertices = len(approx)
            shape = "unknown"
            color = (0, 255, 0)  # 綠色
            
            if vertices == 3:
                shape = "triangle"
                color = (0, 255, 255)  # 黃色
            elif vertices == 4:
                # 檢查是否為正方形或矩形
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                shape = "square" if 0.95 <= aspect_ratio <= 1.05 else "rectangle"
                color = (255, 0, 0)  # 藍色
            elif vertices == 5:
                shape = "pentagon"
                color = (255, 255, 0)  # 青色
            elif vertices == 6:
                shape = "hexagon"
                color = (255, 0, 255)  # 洋紅色
            else:
                # 圓形 (多邊形頂點很多)
                shape = "circle"
                color = (0, 0, 255)  # 紅色
            
            # 繪製輪廓和標籤
            cv2.drawContours(annotated, [approx], -1, color, 3)
            cv2.circle(annotated, (cx, cy), 5, color, -1)
            cv2.putText(annotated, shape, (cx - 40, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 記錄偵測結果
            detections.append({
                "shape": shape,
                "vertices": vertices,
                "area": float(area),
                "center": {"x": cx, "y": cy},
                "color": color
            })
        
        return detections, annotated
    
    def image_callback(self, msg):
        try:
            # 轉換為 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 執行形狀偵測
            detections, annotated = self.detect_shapes(cv_image)
            
            # 發布偵測結果
            if detections:
                detection_msg = String()
                detection_msg.data = json.dumps({
                    "timestamp": self.get_clock().now().to_msg(),
                    "detections": detections,
                    "count": len(detections)
                })
                self.detection_pub.publish(detection_msg)
            
            # 發布標註後的影像
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.annotated_pub.publish(annotated_msg)
            
            # 定期日誌
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processed {self.frame_count} frames, detected {len(detections)} shapes')
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def destroy_node(self):
        self.get_logger().info('Shape Detector shutting down...')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector_node = ShapeDetector()
    
    try:
        rclpy.spin(detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
