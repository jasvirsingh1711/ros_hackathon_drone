#!/usr/bin/env python3
"""
Component Test Suite for Autonomous Drone System
Tests each module independently before full mission
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Twist
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String, Float64MultiArray, Int32
import subprocess
import time
import sys

class DroneSelfTest(Node):
    def __init__(self):
        super().__init__('drone_self_test')
        self.test_results = {}
        self.sensors_received = {}
        
        # Create subscriptions for all sensors
        self.imu_sub = self.create_subscription(Imu, '/drone/imu', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/drone/front_camera/image_raw', 
                                                     self.camera_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/drone/front_camera/depth/image_raw',
                                                   self.depth_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/drone/altitude', self.lidar_callback, 10)
        self.nav_state_sub = self.create_subscription(String, '/drone/nav_state', 
                                                       self.nav_state_callback, 10)
        self.status_sub = self.create_subscription(String, '/drone/status', 
                                                    self.status_callback, 10)
        
        # Publishers for testing
        self.cmd_force_pub = self.create_publisher(Wrench, '/drone/cmd_force', 10)
        
        self.get_logger().info("Self-test initialized")
    
    def imu_callback(self, msg):
        self.sensors_received['imu'] = True
    
    def camera_callback(self, msg):
        self.sensors_received['camera'] = True
    
    def depth_callback(self, msg):
        self.sensors_received['depth'] = True
    
    def lidar_callback(self, msg):
        self.sensors_received['lidar'] = True
    
    def nav_state_callback(self, msg):
        self.sensors_received['nav_state'] = True
    
    def status_callback(self, msg):
        self.get_logger().info(f"Status: {msg.data}")
    
    def test_gazebo_connection(self):
        """Test if Gazebo is running"""
        self.get_logger().info("\n=== Testing Gazebo Connection ===")
        try:
            result = subprocess.run(['gazebo_model_info', '-m', 'autonomous_drone'], 
                                    capture_output=True, timeout=5)
            if result.returncode == 0:
                self.test_results['gazebo'] = "✓ PASS"
                self.get_logger().info("✓ Gazebo is running and drone is spawned")
                return True
            else:
                self.test_results['gazebo'] = "✗ FAIL - Drone not found in Gazebo"
                self.get_logger().error("✗ Gazebo is running but drone not found")
                return False
        except Exception as e:
            self.test_results['gazebo'] = f"✗ FAIL - {str(e)}"
            self.get_logger().error(f"✗ Gazebo connection failed: {e}")
            return False
    
    def test_sensors(self, timeout=5):
        """Test if all sensors are publishing"""
        self.get_logger().info("\n=== Testing Sensor Streams ===")
        self.sensors_received = {}
        
        # Wait for sensor data
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        expected_sensors = ['imu', 'camera', 'depth', 'lidar']
        for sensor in expected_sensors:
            if sensor in self.sensors_received:
                self.test_results[sensor] = "✓ PASS"
                self.get_logger().info(f"✓ {sensor} is publishing data")
            else:
                self.test_results[sensor] = "✗ FAIL"
                self.get_logger().error(f"✗ {sensor} is NOT publishing data")
        
        return all(sensor in self.sensors_received for sensor in expected_sensors)
    
    def test_thruster_command(self):
        """Test if force commands are accepted"""
        self.get_logger().info("\n=== Testing Thruster Command ===")
        try:
            wrench = Wrench()
            wrench.force.z = 1.0  # Hover-like command
            
            for _ in range(10):
                self.cmd_force_pub.publish(wrench)
                time.sleep(0.1)
            
            self.test_results['thruster'] = "✓ PASS"
            self.get_logger().info("✓ Thruster commands are being published")
            return True
        except Exception as e:
            self.test_results['thruster'] = f"✗ FAIL - {str(e)}"
            self.get_logger().error(f"✗ Thruster command failed: {e}")
            return False
    
    def test_ros_topics(self):
        """Verify all expected ROS topics exist"""
        self.get_logger().info("\n=== Testing ROS Topic Availability ===")
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                    capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            
            required_topics = [
                '/drone/imu',
                '/drone/altitude',
                '/drone/front_camera/image_raw',
                '/drone/cmd_force',
                '/drone/nav_state',
                '/drone/status'
            ]
            
            missing = []
            for topic in required_topics:
                if topic in topics:
                    self.test_results[f'topic_{topic}'] = "✓"
                else:
                    missing.append(topic)
                    self.test_results[f'topic_{topic}'] = "✗"
            
            if not missing:
                self.get_logger().info("✓ All required topics are available")
                return True
            else:
                self.get_logger().error(f"✗ Missing topics: {missing}")
                return False
        except Exception as e:
            self.get_logger().error(f"✗ Topic check failed: {e}")
            return False
    
    def print_summary(self):
        """Print test summary"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("SELF-TEST SUMMARY")
        self.get_logger().info("="*50)
        
        passed = 0
        failed = 0
        
        for test, result in self.test_results.items():
            if "PASS" in result or "✓" in result:
                passed += 1
            else:
                failed += 1
            self.get_logger().info(f"{test}: {result}")
        
        self.get_logger().info("="*50)
        self.get_logger().info(f"PASSED: {passed}")
        self.get_logger().info(f"FAILED: {failed}")
        
        if failed == 0:
            self.get_logger().info("\n✓ ALL TESTS PASSED - Ready for mission!")
            return True
        else:
            self.get_logger().error("\n✗ Some tests failed - Check configuration")
            return False
    
    def run_all_tests(self):
        """Run complete test suite"""
        self.get_logger().info("\nStarting Complete Self-Test Suite...")
        
        # Run all tests
        self.test_gazebo_connection()
        self.test_ros_topics()
        self.test_sensors(timeout=8)
        self.test_thruster_command()
        
        # Print summary
        success = self.print_summary()
        
        return success


def main():
    rclpy.init()
    
    print("\n" + "="*70)
    print("IIT MANDI TECH FEST 2026 - AUTONOMOUS DRONE SELF-TEST")
    print("="*70 + "\n")
    
    print("Make sure:")
    print("1. Gazebo is running with the drone spawned")
    print("2. All autonomous control nodes are running")
    print("3. ROS 2 environment is properly sourced\n")
    
    input("Press Enter to start self-test...")
    
    test_node = DroneSelfTest()
    
    try:
        success = test_node.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        sys.exit(1)
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
