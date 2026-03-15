#!/usr/bin/env python3
"""
Landing Zone Detector - Finds red landing pad and guides drone to land
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge


class LandingDetector(Node):
    def __init__(self):
        super().__init__('landing_detector')
        
        self.bridge = CvBridge()
        # Subscribe to BOTTOM camera for red pad detection
        self.image_sub = self.create_subscription(
            Image, '/drone/bottom_camera/image_raw', self.image_callback, 10)
        
        self.landing_pub = self.create_publisher(
            Float64MultiArray, '/drone/landing_target', 10)
        
        self.get_logger().info("Landing Zone Detector initialized - Looking for RED landing pad")
    
    def image_callback(self, msg):
        """Detect red landing zone in camera feed"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = frame.shape[:2]
            
            # Convert BGR to HSV for color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define RED color range in HSV
            # Red wraps around in HSV, so we need two ranges
            # Made more permissive to catch simulation red colors
            lower_red1 = np.array([0, 50, 50])     # Lower saturation/value tolerance
            upper_red1 = np.array([15, 255, 255])   # Slightly wider hue range
            lower_red2 = np.array([165, 50, 50])    # Lower saturation/value tolerance
            upper_red2 = np.array([180, 255, 255])
            
            # Create mask for red color
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Morphological operations to clean up
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            landing_target = Float64MultiArray()
            landing_target.data = [0.0, 0.0, 0.0]  # [x_norm, y_norm, area]
            
            if contours:
                # Find largest red region
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 500:  # Minimum pad size
                    # Get center of red landing pad
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Normalize coordinates to [-1, 1]
                        norm_x = (cx - width / 2.0) / (width / 2.0)
                        norm_y = (cy - height / 2.0) / (height / 2.0)
                        
                        landing_target.data = [norm_x, norm_y, area]
                        
                        # Draw detection
                        cv2.drawContours(frame, [largest_contour], 0, (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                        cv2.putText(frame, "LANDING PAD", (cx - 50, cy - 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        self.get_logger().info(f"🔴 RED PAD DETECTED: cx={cx}, cy={cy}, area={area:.0f}px, norm_pos=[{norm_x:.2f}, {norm_y:.2f}]")
            else:
                cv2.putText(frame, "Searching for RED landing pad...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                self.get_logger().debug("⚠️ No red pad found in frame")
            
            self.landing_pub.publish(landing_target)
            
            # Show detection window
            cv2.imshow("Landing Zone Detector - RED pad", frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in landing detection: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    detector = LandingDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
