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
        """使用 OpenCV 偵測形狀 - 高信心度版本"""
        # 轉灰階
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 增強對比度 (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # Canny 邊緣偵測
        edges = cv2.Canny(blurred, 40, 120)
        
        # 形態學操作 - 閉合邊緣
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=2)
        edges = cv2.erode(edges, kernel, iterations=2)
        
        # 尋找輪廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        annotated = image.copy()
        
        for contour in contours:
            # ===== 過濾條件 1: 面積閾值 =====
            area = cv2.contourArea(contour)
            if area < 500:  # 提高到 500,過濾小雜訊
                continue
            
            # ===== 過濾條件 2: 面積上限 =====
            # 過濾過大的區域 (可能是背景或錯誤偵測)
            image_area = image.shape[0] * image.shape[1]
            if area > image_area * 0.8:  # 超過畫面 80% 的物體
                continue
            
            # ===== 過濾條件 3: 輪廓長度 =====
            perimeter = cv2.arcLength(contour, True)
            if perimeter < 50:  # 周長太小
                continue
            
            # ===== 過濾條件 4: 凸包檢測 =====
            # 計算凸包,檢查輪廓的凸性
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            
            solidity = area / hull_area  # 實心度
            if solidity < 0.7:  # 太不規則的形狀
                continue
            
            # ===== 過濾條件 5: 圓形度檢查 =====
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # 計算近似多邊形
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            
            # 計算中心點
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                continue  # 無效的輪廓
            
            # ===== 形狀辨識與信心度計算 =====
            vertices = len(approx)
            shape = "unknown"
            confidence = 0.0
            color = (0, 255, 0)  # 綠色
            
            if vertices == 3:
                shape = "triangle"
                confidence = 0.9 if solidity > 0.85 else 0.7
                color = (0, 255, 255)  # 黃色
                
            elif vertices == 4:
                # 檢查是否為正方形或矩形
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                
                if 0.9 <= aspect_ratio <= 1.1:
                    shape = "square"
                    confidence = 0.9 if 0.95 <= aspect_ratio <= 1.05 else 0.75
                else:
                    shape = "rectangle"
                    confidence = 0.85 if solidity > 0.85 else 0.7
                color = (255, 0, 0)  # 藍色
                
            elif vertices == 5:
                shape = "pentagon"
                confidence = 0.8 if solidity > 0.85 else 0.65
                color = (255, 255, 0)  # 青色
                
            elif vertices == 6:
                shape = "hexagon"
                confidence = 0.8 if solidity > 0.85 else 0.65
                color = (255, 0, 255)  # 洋紅色
                
            elif vertices > 8:
                # 圓形 (多邊形頂點很多)
                if circularity > 0.7:  # 圓形度高
                    shape = "circle"
                    confidence = min(circularity, 0.95)
                    color = (0, 0, 255)  # 紅色
                else:
                    continue  # 不規則形狀,跳過
            else:
                # 其他形狀,信心度較低
                continue
            
            # ===== 過濾條件 6: 最低信心度閾值 =====
            MIN_CONFIDENCE = 0.65  # 最低信心度 65%
            if confidence < MIN_CONFIDENCE:
                continue
            
            # ===== 繪製輪廓和標籤 =====
            cv2.drawContours(annotated, [approx], -1, color, 3)
            cv2.circle(annotated, (cx, cy), 5, color, -1)
            
            # 顯示形狀名稱和信心度
            label = f"{shape} {confidence:.0%}"
            cv2.putText(annotated, label, (cx - 50, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 記錄偵測結果
            detections.append({
                "shape": shape,
                "confidence": float(confidence),
                "vertices": vertices,
                "area": float(area),
                "circularity": float(circularity),
                "solidity": float(solidity),
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
                    "frame": self.frame_count,
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
