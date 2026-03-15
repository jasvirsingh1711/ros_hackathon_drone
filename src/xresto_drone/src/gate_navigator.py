#!/usr/bin/env python3
"""
Gate Navigator Node - LiDAR Altitude Control + Camera Gate Detection
Controls drone altitude using LiDAR, searches for gates with camera, approaches and traverses
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from enum import Enum
import numpy as np
import time

class NavigationState(Enum):
    IDLE = 0
    TAKEOFF = 1
    SEARCH_FOR_GATE = 2
    APPROACH_GATE = 3
    TRAVERSE_GATE = 4
    LANDING = 5
    MISSION_COMPLETE = 6

class GateNavigator(Node):
    def __init__(self):
        super().__init__('gate_navigator')
        
        # Publishers
        self.waypoint_pub = self.create_publisher(
            Float64MultiArray, '/drone/target_waypoint', 10)
        self.nav_state_pub = self.create_publisher(String, '/drone/nav_state', 10)
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        
        # Subscribers
        self.altitude_sub = self.create_subscription(
            LaserScan, '/drone/altitude', self.altitude_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odometry_callback, 10)
        self.gate_detection_sub = self.create_subscription(
            Float64MultiArray, '/drone/gate_detection', self.gate_detection_callback, 10)
        self.landing_target_sub = self.create_subscription(
            Float64MultiArray, '/drone/landing_target', self.landing_target_callback, 10)
        
        # State machine
        self.state = NavigationState.IDLE
        self.state_start_time = time.time()
        self.current_altitude = 0.0  # From LiDAR
        self.last_altitude = 0.0  # For velocity calculation
        self.last_altitude_time = time.time()
        self.altitude_velocity = 0.0  # dAlt/dt
        self.drone_xy = np.array([0.0, 0.0])  # From odometry
        self.detected_gate = None  # [center_x, center_y, gate_height]
        self.landing_target = None  # [norm_x, norm_y, pad_area]
        self.gate_count = 0
        self.total_gates = 2  # Only 2 gates in this arena - stop after gate 2
        
        # Control parameters
        self.target_takeoff_altitude = 2.5  # m - final altitude target
        self.commanded_altitude = 0.5  # m - altitude we're currently commanding (ramps up gradually)
        self.gate_approach_distance = 5.0  # m - keep this distance while approaching
        self.traverse_speed = 1.0  # m/s
        self.max_climb_speed = 0.3  # m/s - max allowed upward velocity BEFORE increasing target
        self.altitude_increment = 0.1  # m - increase target by this much per update
        self.last_commanded_update = time.time()
        self.min_wall_distance = 0.08  # 8cm minimum distance from walls
        
        # Timer for state machine
        self.timer = self.create_timer(0.2, self.state_machine_update)
        
        self.get_logger().info("Gate Navigator initialized - LiDAR altitude mode")
    
    def altitude_callback(self, msg: LaserScan):
        """Read altitude from LiDAR and compute vertical velocity"""
        if len(msg.ranges) > 0:
            # Take the center beam
            center_idx = len(msg.ranges) // 2
            range_val = msg.ranges[center_idx]
            
            # DEBUG: Log raw altitude readings
            self.get_logger().info(f"[DIAG] Raw LiDAR: idx={center_idx} range={range_val:.2f}m | Min={min(msg.ranges):.2f}m Max={max(msg.ranges):.2f}m")
            
            # Filter out bad readings (NaN, inf)
            if not np.isnan(range_val) and not np.isinf(range_val) and 0.0 < range_val < 100:
                now = time.time()
                
                # Only calculate velocity if we have a previous timestamp
                if self.last_altitude_time > 0:
                    dt = now - self.last_altitude_time
                    
                    if dt > 0.01:  # Only update if enough time has passed
                        # Compute vertical velocity
                        self.altitude_velocity = (range_val - self.last_altitude) / dt
                        # Sanity check - velocity should be < 10 m/s for realistic flight
                        self.altitude_velocity = np.clip(self.altitude_velocity, -10, 10)
                else:
                    # First reading - just store it
                    self.altitude_velocity = 0.0
                
                self.last_altitude = range_val
                self.last_altitude_time = now
                self.current_altitude = float(range_val)
                self.get_logger().debug(f"[DIAG] Altitude: {self.current_altitude:.2f}m Velocity: {self.altitude_velocity:.2f}m/s")
    
    def odometry_callback(self, msg: Odometry):
        """Update drone XY position from odometry"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Clamp to avoid divergence
        x = np.clip(x, -100, 100)
        y = np.clip(y, -100, 100)
        
        self.drone_xy = np.array([x, y])
    
    def gate_detection_callback(self, msg: Float64MultiArray):
        """Receive detected gate information from camera"""
        if len(msg.data) >= 3:
            # [center_x_pixels, center_y_pixels, gate_size]
            gate_x, gate_y, gate_sz = msg.data[:3]
            if gate_sz > 50:  # Only accept if substantial detection
                self.detected_gate = [gate_x, gate_y, gate_sz]
                self.get_logger().debug(f"[GATE DETECTION] Gate found! Size: {gate_sz:.0f}px at ({gate_x:.0f}, {gate_y:.0f})")
            else:
                # Too small, probably noise
                self.detected_gate = None
    
    def landing_target_callback(self, msg: Float64MultiArray):
        """Receive landing pad detection from landing detector"""
        if len(msg.data) >= 3:
            # [norm_x, norm_y, pad_area]
            norm_x, norm_y, pad_area = msg.data[:3]
            if pad_area > 100:  # Only accept if substantial pad detected
                self.landing_target = [norm_x, norm_y, pad_area]
                self.get_logger().debug(f"[LANDING] Pad detected! Area: {pad_area:.0f}px at norm_pos=[{norm_x:.2f}, {norm_y:.2f}]")
            else:
                self.landing_target = None
        else:
            self.landing_target = None
    
    def state_machine_update(self):
        """Main state machine loop"""
        now = time.time()
        elapsed = now - self.state_start_time
        
        # Publish current state
        state_msg = String()
        
        if self.state == NavigationState.IDLE:
            state_msg.data = "IDLE"
            self.nav_state_pub.publish(state_msg)
            # Transition to takeoff
            self.state = NavigationState.TAKEOFF
            self.state_start_time = now
            self.get_logger().info("Starting TAKEOFF phase")
        
        elif self.state == NavigationState.TAKEOFF:
            state_msg.data = f"TAKEOFF - Alt: {self.current_altitude:.2f}m Cmd: {self.commanded_altitude:.2f}m Vel: {self.altitude_velocity:.2f}m/s"
            self.nav_state_pub.publish(state_msg)
            
            # Safety check - if altitude goes too high, emergency descent
            if self.current_altitude > 10.0:
                target_waypoint = Float64MultiArray()
                target_waypoint.data = [0.0, 0.0, 1.0]
                self.waypoint_pub.publish(target_waypoint)
                self.get_logger().warn(f"EMERGENCY DESCENT - Alt {self.current_altitude:.1f}m!")
                return
            
            # VELOCITY-LIMITED RAMPING: Only increase altitude target if climbing slowly
            time_since_update = now - self.last_commanded_update
            if time_since_update > 0.5:  # Update every 0.5 seconds
                if abs(self.altitude_velocity) < self.max_climb_speed:
                    # Climbing slowly enough - increase target altitude
                    if self.commanded_altitude < self.target_takeoff_altitude:
                        self.commanded_altitude = min(self.commanded_altitude + self.altitude_increment, 
                                                     self.target_takeoff_altitude)
                        self.get_logger().info(f"Increased altitude target to {self.commanded_altitude:.2f}m")
                else:
                    # Climbing too fast - HOLD current altitude, force braking
                    self.get_logger().info(f"BRAKING: Velocity {self.altitude_velocity:.2f}m/s > {self.max_climb_speed}m/s")
                self.last_commanded_update = now
            
            # Send current commanded altitude
            target_waypoint = Float64MultiArray()
            target_waypoint.data = [0.0, 0.0, self.commanded_altitude]
            self.waypoint_pub.publish(target_waypoint)
            
            # Check if reached target altitude with low velocity
            if (self.current_altitude >= self.target_takeoff_altitude - 0.2 and 
                abs(self.altitude_velocity) < 0.2):
                self.state = NavigationState.SEARCH_FOR_GATE
                self.state_start_time = now
                self.detected_gate = None
                self.get_logger().info(f"✓ TAKEOFF COMPLETE at {self.current_altitude:.2f}m - Searching for gates")
        
        elif self.state == NavigationState.SEARCH_FOR_GATE:
            state_msg.data = f"SEARCH_FOR_GATE - Patrolling for gates (#{self.gate_count + 1})"
            self.nav_state_pub.publish(state_msg)
            
            # ACTIVE SEARCH: Move back and forth to find next gate
            # Oscillate forward/backward with period ~20 seconds
            search_cycle = (elapsed % 20.0)  # Repeats every 20 seconds
            
            if search_cycle < 10.0:
                # Moving forward
                forward_velocity = 1.0
            else:
                # Moving backward
                forward_velocity = -0.8
            
            # Scan left-right while moving
            sweep_position = np.sin(elapsed * 0.5)  # Oscillate left-right
            lateral_velocity = sweep_position * 0.3
            
            target_waypoint = Float64MultiArray()
            target_waypoint.data = [forward_velocity, lateral_velocity, self.target_takeoff_altitude]
            self.waypoint_pub.publish(target_waypoint)
            
            if elapsed % 5.0 < 0.1:  # Log every 5 seconds
                self.get_logger().info(f"🔍 Searching for gate #{self.gate_count + 1}... (cycle: {search_cycle:.1f}s)")
            
            # If gate detected, approach it
            if self.detected_gate is not None:
                self.state = NavigationState.APPROACH_GATE
                self.state_start_time = now
                self.get_logger().info(f"🚪 GATE #{self.gate_count + 1} DETECTED! Approaching...")
            
            # Timeout - assume no more gates
            if elapsed > 60.0:  # Search for 1 minute max
                self.state = NavigationState.LANDING
                self.state_start_time = now
                self.get_logger().info(f"✓ Search finished! Proceeding to landing zone. Gates passed: {self.gate_count}")
        
        elif self.state == NavigationState.APPROACH_GATE:
            state_msg.data = f"APPROACH_GATE - Flying forward into gate"
            self.nav_state_pub.publish(state_msg)
            
            if self.detected_gate is None:
                # Lost the gate
                self.state = NavigationState.SEARCH_FOR_GATE
                self.state_start_time = now
                self.get_logger().info("Lost gate - Returning to search")
                return
            
            # Get gate size from detection
            gate_center_x, gate_center_y, gate_size = self.detected_gate
            
            # SIMPLE FORWARD APPROACH - just go straight at the gate
            # Speed increases as we get closer (larger detected size = closer = faster)
            forward_velocity = np.clip(1.0 + (gate_size / 40000.0), 1.0, 3.5)  # Faster approach
            
            # Send forward motion command ONLY (no steering - keep it simple)
            target_waypoint = Float64MultiArray()
            target_waypoint.data = [forward_velocity, 0.0, self.target_takeoff_altitude]
            self.waypoint_pub.publish(target_waypoint)
            
            if elapsed < 1.0:  # Log only on first update
                self.get_logger().info(f"→ ENTERING gate | Gate size: {gate_size:.0f}px | Forward velocity: {forward_velocity:.2f}m/s")

            
            # If gate is large and fills screen, we've passed through it!
            if gate_size > 20000:  # Lowered threshold for gate 2
                self.state = NavigationState.TRAVERSE_GATE
                self.state_start_time = now
                self.gate_count += 1
                self.get_logger().info(f"✓ PASSED GATE {self.gate_count}! Searching for next gate...")
            
            # Timeout to prevent being stuck
            if elapsed > 30.0:
                self.state = NavigationState.SEARCH_FOR_GATE
                self.state_start_time = now
                self.get_logger().info("Approach timeout - Returning to search")
        
        elif self.state == NavigationState.TRAVERSE_GATE:
            state_msg.data = f"TRAVERSE_GATE - Flying through gate {self.gate_count}/{self.total_gates}"
            self.nav_state_pub.publish(state_msg)
            
            # Fly forward through gate at constant speed - FASTER
            target_waypoint = Float64MultiArray()
            target_waypoint.data = [1.5, 0.0, self.target_takeoff_altitude]  # Faster traverse
            self.waypoint_pub.publish(target_waypoint)
            
            # After traversal time, decide next action
            if elapsed > 3.0:  # Reduced traverse time - 3 seconds
                if self.gate_count >= self.total_gates:
                    # All gates passed - STOP and hover at landing pad
                    self.state = NavigationState.LANDING
                    self.state_start_time = now
                    self.get_logger().info(f"✓ GATE {self.gate_count} PASSED! All {self.gate_count} gates done. Stopping in air to find landing pad...")
                else:
                    # More gates to go - search for next
                    self.state = NavigationState.SEARCH_FOR_GATE
                    self.state_start_time = now
                    self.detected_gate = None
                    self.get_logger().info(f"Gate {self.gate_count} traversed - Searching for next gate")
        
        elif self.state == NavigationState.LANDING:
            state_msg.data = f"LANDING - Altitude: {self.current_altitude:.2f}m | Target: {self.target_takeoff_altitude:.2f}m"
            self.nav_state_pub.publish(state_msg)
            
            # Start landing sequence
            if self.landing_target is not None and len(self.landing_target) >= 3:
                norm_x, norm_y, pad_area = self.landing_target[:3]
                
                # If pad is large enough and centered, descend continuously
                if pad_area > 500:  # Red pad is visible
                    # Simple descent while steering toward pad center if needed
                    forward_vel = norm_x * 0.2  # Steer based on X position of pad
                    lateral_vel = norm_y * 0.2  # Steer based on Y position of pad
                    
                    # Descend continuously - reduce altitude target gradually
                    descent_rate = 0.05  # 5cm per update (0.2s) = 25cm/s descent
                    self.target_takeoff_altitude = max(self.target_takeoff_altitude - descent_rate, 0.15)
                    
                    target_waypoint = Float64MultiArray()
                    target_waypoint.data = [forward_vel, lateral_vel, self.target_takeoff_altitude]
                    self.waypoint_pub.publish(target_waypoint)
                    
                    if elapsed % 1.0 < 0.2:  # Log every second
                        self.get_logger().info(f"🛬 LANDING: Descending to {self.target_takeoff_altitude:.2f}m | Pad area: {pad_area:.0f}px | Pos: [{norm_x:.2f}, {norm_y:.2f}]")
                else:
                    # Pad too small - maintain altitude and search
                    target_waypoint = Float64MultiArray()
                    target_waypoint.data = [0.0, 0.0, self.target_takeoff_altitude]
                    self.waypoint_pub.publish(target_waypoint)
                    self.get_logger().debug("🔍 Pad too small, maintaining altitude...")
            else:
                # No pad detected - descend anyway (may have passed over it)
                self.target_takeoff_altitude = max(self.target_takeoff_altitude - 0.03, 0.15)
                target_waypoint = Float64MultiArray()
                target_waypoint.data = [0.0, 0.0, self.target_takeoff_altitude]
                self.waypoint_pub.publish(target_waypoint)
                
                if elapsed % 1.0 < 0.2:
                    self.get_logger().info(f"⬇️ No pad detected, descending anyway to {self.target_takeoff_altitude:.2f}m")
            
            # Check if landed - when very close to ground
            if self.current_altitude < 0.15:
                self.state = NavigationState.MISSION_COMPLETE
                self.state_start_time = now
                self.get_logger().info("✓ LANDED SUCCESSFULLY!")
        
        elif self.state == NavigationState.MISSION_COMPLETE:
            state_msg.data = f"MISSION_COMPLETE - {self.gate_count} gates traversed"
            self.nav_state_pub.publish(state_msg)
            self.get_logger().info(f"✓ MISSION COMPLETE - Total gates: {self.gate_count}")

def main(args=None):
    rclpy.init(args=args)
    navigator = GateNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
