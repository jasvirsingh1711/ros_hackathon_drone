#!/usr/bin/env python3
"""
LiDAR Altitude Sensor Diagnostic Tool
Monitors /drone/altitude topic and verifies readings are in expected range
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sys

class AltitudeDiagnostics(Node):
    def __init__(self):
        super().__init__('altitude_diagnostics')
        self.subscription = self.create_subscription(
            LaserScan,
            '/drone/altitude',
            self.altitude_callback,
            10)
        
        self.reading_count = 0
        self.min_reading = float('inf')
        self.max_reading = 0.0
        self.avg_reading = 0.0
        self.sum_readings = 0.0
        self.error_count = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("LiDAR ALTITUDE SENSOR DIAGNOSTICS")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Listening to /drone/altitude topic...")
        self.get_logger().info("Expected readings: 0.1m - 50.0m")
        self.get_logger().info("During takeoff: 0.5m - 2.5m")
        self.get_logger().info("=" * 60)
    
    def altitude_callback(self, msg: LaserScan):
        self.reading_count += 1
        
        if len(msg.ranges) == 0:
            self.get_logger().error("[ERROR] LaserScan has no ranges!")
            self.error_count += 1
            return
        
        # Take center beam (should be single beam)
        center_idx = len(msg.ranges) // 2
        range_val = msg.ranges[center_idx]
        
        # Check for invalid readings
        if range_val <= 0 or range_val > 50:
            status = "❌ OUT OF RANGE"
            self.error_count += 1
        elif range_val < 0.1:
            status = "❌ TOO CLOSE (measuring drone body)"
            self.error_count += 1
        elif range_val < 0.5:
            status = "⚠️  CLOSE (pre-takeoff)"
        elif 0.5 <= range_val <= 2.5:
            status = "✅ GOOD (takeoff range)"
        elif range_val > 10:
            status = "❌ TOO HIGH (measuring wall?)"
            self.error_count += 1
        else:
            status = "✅ OK (search altitude)"
        
        # Update statistics
        self.sum_readings += range_val
        self.avg_reading = self.sum_readings / self.reading_count
        self.min_reading = min(self.min_reading, range_val)
        self.max_reading = max(self.max_reading, range_val)
        
        # Log every 10th reading or errors
        if self.reading_count % 10 == 0 or self.error_count > 0:
            self.get_logger().info(
                f"[#{self.reading_count:4d}] Range: {range_val:6.2f}m | "
                f"Avg: {self.avg_reading:6.2f}m | "
                f"Min: {self.min_reading:6.2f}m | "
                f"Max: {self.max_reading:6.2f}m | "
                f"Errors: {self.error_count:3d} | {status}"
            )
        
        # Print stats every 100 readings
        if self.reading_count == 100:
            self.get_logger().info("=" * 60)
            self.get_logger().info("100-READING SUMMARY:")
            self.get_logger().info(f"  Average altitude: {self.avg_reading:.2f}m")
            self.get_logger().info(f"  Min altitude: {self.min_reading:.2f}m")
            self.get_logger().info(f"  Max altitude: {self.max_reading:.2f}m")
            self.get_logger().info(f"  Total errors: {self.error_count}/100")
            self.get_logger().info("=" * 60)
            
            if self.error_count > 20:
                self.get_logger().error(
                    "⚠️  HIGH ERROR RATE! LiDAR may not be oriented correctly.\n"
                    "   Check URDF pose attribute (should have pitch=1.5708=90°)"
                )

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n" + "=" * 60)
        node.get_logger().info("DIAGNOSTICS SUMMARY")
        node.get_logger().info("=" * 60)
        node.get_logger().info(f"Total readings: {node.reading_count}")
        node.get_logger().info(f"Total errors: {node.error_count}")
        error_rate = (node.error_count / node.reading_count * 100) if node.reading_count > 0 else 0
        node.get_logger().info(f"Error rate: {error_rate:.1f}%")
        node.get_logger().info(f"Average altitude: {node.avg_reading:.2f}m")
        node.get_logger().info(f"Altitude range: {node.min_reading:.2f}m - {node.max_reading:.2f}m")
        
        if error_rate < 5:
            node.get_logger().info("✅ LIDAR WORKING WELL")
        elif error_rate < 20:
            node.get_logger().info("⚠️  LIDAR HAS SOME ISSUES")
        else:
            node.get_logger().error("❌ LIDAR NOT WORKING PROPERLY")
        
        node.get_logger().info("=" * 60)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
