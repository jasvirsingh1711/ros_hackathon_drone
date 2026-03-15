#!/usr/bin/env python3
"""
Trajectory Planner Node for Autonomous Drone
Generates collision-free trajectories through gates
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Point, Vector3, Twist
from std_msgs.msg import Float64MultiArray, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import math
import time

class PathSegment(Enum):
    APPROACH = 1
    CENTER = 2
    DEPART = 3

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Publishers
        self.nav_force_pub = self.create_publisher(Wrench, '/drone/nav_force', 10)
        self.trajectory_pub = self.create_publisher(Float64MultiArray, '/drone/planned_trajectory', 10)
        self.current_waypoint_pub = self.create_publisher(Int32, '/drone/current_waypoint', 10)
        
        # Subscribers
        self.target_waypoint_sub = self.create_subscription(
            Float64MultiArray, '/drone/target_waypoint', self.waypoint_callback, 10)
        self.obstacles_sub = self.create_subscription(
            Float64MultiArray, '/drone/detected_obstacles', self.obstacles_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odometry_callback, 10)
        self.altitude_sub = self.create_subscription(
            LaserScan, '/drone/altitude', self.altitude_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/drone/depth_camera/depth/image_raw', self.depth_callback, 10)
        
        # Timer for trajectory updates
        self.timer = self.create_timer(0.05, self.update_trajectory)  # 20 Hz
        
        # State variables
        self.target_waypoint = None
        self.detected_obstacles = []
        self.drone_pos = np.array([0.0, 0.0, 1.5])  # Current drone position (takeoff altitude)
        self.last_altitude = 0.0  # For velocity calculation
        self.last_altitude_time = 0.0
        self.altitude_velocity = 0.0  # Vertical velocity
        self.current_waypoint_idx = 0
        self.waypoints = []
        
        # Collision avoidance
        self.bridge = CvBridge()
        self.min_distance_front = 10.0  # Forward distance in meters
        self.min_distance_left = 10.0   # Left distance in meters
        self.min_distance_right = 10.0  # Right distance in meters
        self.min_distance_back = 10.0   # Back distance in meters
        self.collision_min_distance = 0.08  # 8cm minimum distance from walls
        
        # Arena boundaries (typical test arena is ~3m x 3m)
        self.arena_x_min = -1.5  # Left wall
        self.arena_x_max = 1.5   # Right wall
        self.arena_y_min = -1.5  # Back wall
        self.arena_y_max = 1.5   # Front wall
        
        # Motion parameters
        self.max_velocity_xy = 1.5  # m/s
        self.max_velocity_z = 1.0   # m/s
        self.target_altitude = 2.0   # m
        self.update_count = 0  # For debug logging
        
        self.get_logger().info("Waypoint-based Trajectory Planner initialized for real world")
    
    def waypoint_callback(self, msg):
        """Receive target waypoint from navigator"""
        if len(msg.data) >= 3:
            self.target_waypoint = {
                'x': msg.data[0],
                'y': msg.data[1],
                'z': msg.data[2]
            }
    
    def obstacles_callback(self, msg):
        """Receive detected obstacles for avoidance"""
        obstacles = []
        for i in range(0, len(msg.data), 3):
            if i + 2 < len(msg.data):
                obstacles.append({
                    'x': msg.data[i],
                    'y': msg.data[i+1],
                    'distance': msg.data[i+2]
                })
        self.detected_obstacles = obstacles
    
    def odometry_callback(self, msg):
        """Receive current drone X,Y position from odometry"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Clamp XY to reasonable range
        x = np.clip(x, -100, 100)
        y = np.clip(y, -100, 100)
        self.drone_pos[0] = x
        self.drone_pos[1] = y
        # Z will be updated by altitude_callback
    
    def altitude_callback(self, msg):
        """Receive current drone Z position from LiDAR altitude and compute velocity"""
        if len(msg.ranges) > 0 and msg.ranges[0] > 0:
            altitude = msg.ranges[0]
            # Clamp to reasonable range (0-10m flying height)
            altitude = np.clip(altitude, 0.0, 10.0)
            
            # Compute vertical velocity
            now = time.time()
            
            # Only calculate velocity if we have a previous timestamp
            if self.last_altitude_time > 0:
                dt = now - self.last_altitude_time
                if dt > 0.01:
                    self.altitude_velocity = (altitude - self.last_altitude) / dt
                    # Sanity check - velocity should be < 10 m/s for realistic flight
                    self.altitude_velocity = np.clip(self.altitude_velocity, -10, 10)
            else:
                # First reading - just store it
                self.altitude_velocity = 0.0
            
            self.last_altitude = altitude
            self.last_altitude_time = now
            self.drone_pos[2] = altitude
            self.drone_pos[2] = altitude
    
    def depth_callback(self, msg):
        """Process depth image for collision avoidance - maintain 8cm distance from walls"""
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            h, w = depth_image.shape
            
            # Replace NaN and inf with max range (10m)
            depth_image = np.nan_to_num(depth_image, nan=10.0, posinf=10.0, neginf=0.0)
            
            # Divide image into regions for directional collision detection
            # Front region (center of image)
            front_region = depth_image[int(h*0.3):int(h*0.7), int(w*0.3):int(w*0.7)]
            self.min_distance_front = float(np.min(front_region)) if front_region.size > 0 else 10.0
            
            # Left region
            left_region = depth_image[int(h*0.3):int(h*0.7), 0:int(w*0.3)]
            self.min_distance_left = float(np.min(left_region)) if left_region.size > 0 else 10.0
            
            # Right region
            right_region = depth_image[int(h*0.3):int(h*0.7), int(w*0.7):w]
            self.min_distance_right = float(np.min(right_region)) if right_region.size > 0 else 10.0
            
            # Back region (bottom of image)
            back_region = depth_image[int(h*0.7):h, int(w*0.3):int(w*0.7)]
            self.min_distance_back = float(np.min(back_region)) if back_region.size > 0 else 10.0
            
        except Exception as e:
            self.get_logger().debug(f"Depth processing error: {str(e)}")
    
    def check_wall_collision(self, vx, vy):
        """
        Check if drone motion will hit walls based on odometry position
        Simple hard stop at boundary - no predictive checking
        """
        drone_x = self.drone_pos[0]
        drone_y = self.drone_pos[1]
        
        # Check distance to each wall
        dist_to_left_wall = drone_x - self.arena_x_min
        dist_to_right_wall = self.arena_x_max - drone_x
        dist_to_back_wall = drone_y - self.arena_y_min
        dist_to_front_wall = self.arena_y_max - drone_y
        
        # STRICT but non-predictive: if already at boundary, stop
        # Left wall (negative X motion)
        if dist_to_left_wall < self.collision_min_distance and vx < 0:
            vx = 0.0
        
        # Right wall (positive X motion)
        if dist_to_right_wall < self.collision_min_distance and vx > 0:
            vx = 0.0
        
        # Back wall (negative Y motion)
        if dist_to_back_wall < self.collision_min_distance and vy < 0:
            vy = 0.0
        
        # Front wall (positive Y motion)
        if dist_to_front_wall < self.collision_min_distance and vy > 0:
            vy = 0.0
        
        return vx, vy

        """Get information about the current target gate"""
        if not self.gate_sequence or self.current_gate_id >= len(self.gate_sequence):
            return None
        
        target_gate_id = self.gate_sequence[self.current_gate_id]
        
        for gate in self.detected_gates:
            if gate['id'] == target_gate_id:
                return gate
        return None
    
    def generate_path_to_gate(self, gate_info):
        """
        Generate approach trajectory to the gate
        Returns list of waypoints
        """
        if not gate_info:
            return []
        
        waypoints = []
        
        # 1. Approach waypoint (before the gate)
        approach_dist = self.min_approach_dist
        approach_x = gate_info['x'] - approach_dist * math.cos(gate_info['yaw'])
        approach_y = gate_info['y'] - approach_dist * math.sin(gate_info['yaw'])
        approach_z = self.gate_start_height
        
        waypoints.append({
            'x': approach_x, 'y': approach_y, 'z': approach_z,
            'type': PathSegment.APPROACH,
            'yaw': gate_info['yaw']
        })
        
        # 2. Gate center waypoint (passing through)
        waypoints.append({
            'x': gate_info['x'], 'y': gate_info['y'], 'z': gate_info['z'],
            'type': PathSegment.CENTER,
            'yaw': gate_info['yaw']
        })
        
        # 3. Departure waypoint (after the gate)
        departure_dist = self.min_approach_dist
        dept_x = gate_info['x'] + departure_dist * math.cos(gate_info['yaw'])
        dept_y = gate_info['y'] + departure_dist * math.sin(gate_info['yaw'])
        dept_z = self.gate_start_height
        
        waypoints.append({
            'x': dept_x, 'y': dept_y, 'z': dept_z,
            'type': PathSegment.DEPART,
            'yaw': gate_info['yaw']
        })
        
        return waypoints
    
    def compute_velocity_command(self, drone_pos, target_waypoint):
        """
        Compute velocity command to reach target waypoint
        """
        dx = target_waypoint['x'] - drone_pos[0]
        dy = target_waypoint['y'] - drone_pos[1]
        dz = target_waypoint['z'] - drone_pos[2]
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # If we've reached the waypoint, return minimal command
        if distance < 0.1:
            return 0.0, 0.0, 0.0
        
        # Normalize and scale velocity
        vx = (dx / distance) * self.max_velocity_xy
        vy = (dy / distance) * self.max_velocity_xy
        vz = (dz / distance) * self.max_velocity_z
        
        # Limit individual components
        vx = np.clip(vx, -self.max_velocity_xy, self.max_velocity_xy)
        vy = np.clip(vy, -self.max_velocity_xy, self.max_velocity_xy)
        vz = np.clip(vz, -self.max_velocity_z, self.max_velocity_z)
        
        return vx, vy, vz
    
    def update_trajectory(self):
        """
        Main trajectory update loop
        Simply forwards velocity commands from navigator to hover controller
        with collision avoidance applied
        """
        self.update_count += 1
        
        if not self.target_waypoint:
            # No target waypoint - hover at safe default altitude
            wrench = Wrench()
            wrench.force.z = 2.5  # Safe default hover altitude
            self.nav_force_pub.publish(wrench)
            if self.update_count % 100 == 0:
                self.get_logger().debug("No waypoint received, hovering at 2.5m")
            return
        
        # Extract velocity commands from waypoint
        vx = self.target_waypoint.get('x', 0.0)
        vy = self.target_waypoint.get('y', 0.0)
        target_alt = self.target_waypoint.get('z', 2.5)
        
        # Apply wall collision avoidance based on odometry position
        vx, vy = self.check_wall_collision(vx, vy)
        
        # Depth-based collision avoidance: MINIMAL during gate crossing
        # Don't interfere with forward motion commanded by navigator
        # Only stop if we hit the EXTREME safety limit (2cm)
        if self.min_distance_front < 0.02 and vx > 1.5:  # Only stop if VERY close AND high speed
            vx = np.clip(vx, -2.0, 0.5)  # Gentle brake, don't full stop
        if self.min_distance_back < 0.03 and vx < 0:
            vx = 0.0  # Stop backward motion
        
        # Clamp values to reasonable ranges
        vx = np.clip(vx, -2.0, 2.0)
        vy = np.clip(vy, -2.0, 2.0)
        target_alt = np.clip(target_alt, 1.0, 5.0)
        
        wrench = Wrench()
        wrench.force.x = vx
        wrench.force.y = vy
        wrench.force.z = target_alt
        wrench.torque.z = 0.0
        
        # Apply altitude velocity braking if needed
        if self.altitude_velocity > 0.4:
            wrench.force.z = max(self.drone_pos[2] - 0.3, 1.0)  # Emergency descent
            if self.update_count % 50 == 0:
                self.get_logger().warn(f"ALTITUDE BRAKING! Velocity {self.altitude_velocity:.2f}m/s")
        
        self.nav_force_pub.publish(wrench)
        
        # Debug log with collision info every 100 updates
        if self.update_count % 100 == 0:
            dist_to_left_wall = self.drone_pos[0] - self.arena_x_min
            dist_to_right_wall = self.arena_x_max - self.drone_pos[0]
            dist_to_back_wall = self.drone_pos[1] - self.arena_y_min
            dist_to_front_wall = self.arena_y_max - self.drone_pos[1]
            
            # Log warning if close to any wall
            wall_warnings = []
            if dist_to_left_wall < 0.15:
                wall_warnings.append(f"LEFT: {dist_to_left_wall:.3f}m")
            if dist_to_right_wall < 0.15:
                wall_warnings.append(f"RIGHT: {dist_to_right_wall:.3f}m")
            if dist_to_back_wall < 0.15:
                wall_warnings.append(f"BACK: {dist_to_back_wall:.3f}m")
            if dist_to_front_wall < 0.15:
                wall_warnings.append(f"FRONT: {dist_to_front_wall:.3f}m")
            
            if wall_warnings:
                self.get_logger().warn(f"⚠️ WALL PROXIMITY: {' | '.join(wall_warnings)} | Cmd: vx={vx:.2f} vy={vy:.2f}")
            else:
                self.get_logger().debug(
                    f"Pos: [{self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f}] | "
                f"Cmd: vx={vx:.2f}m/s, vy={vy:.2f}m/s, alt_target={target_alt:.2f}m | "
                f"Wall Dist: L={dist_to_left_wall:.3f}m R={dist_to_right_wall:.3f}m "
                f"B={dist_to_back_wall:.3f}m F={dist_to_front_wall:.3f}m"
            )
    
    def check_obstacle_ahead(self, drone_pos):
        """Check if obstacle is directly ahead"""
        if not self.detected_obstacles:
            return False
        
        # Check if any obstacle is within critical distance
        for obs in self.detected_obstacles:
            if obs['distance'] < 0.05:  # 5cm minimum clearance (tight gate passage)
                return True
        
        return False
    
    def compute_obstacle_avoidance_command(self):
        """Compute evasive velocity command"""
        wrench = Wrench()
        wrench.force.x = -0.3  # Move backward
        wrench.force.y = 0.5   # Strafe sideways
        wrench.force.z = 0.5   # Move upward
        wrench.torque.z = 0.2  # Rotate to find clear path
        return wrench
    
    def compute_velocity_to_waypoint(self, drone_pos, target_pos):
        """Compute velocity vector toward target waypoint"""
        delta = target_pos - drone_pos
        distance = np.linalg.norm(delta)
        
        if distance < 0.1:  # Very close to waypoint
            return 0.0, 0.0, 0.0
        
        # Normalize direction vector
        direction = delta / distance
        
        # Scale by maximum velocities
        vx = direction[0] * self.max_velocity_xy
        vy = direction[1] * self.max_velocity_xy
        vz = direction[2] * self.max_velocity_z
        
        # Clip to limits
        vx = np.clip(vx, -self.max_velocity_xy, self.max_velocity_xy)
        vy = np.clip(vy, -self.max_velocity_xy, self.max_velocity_xy)
        vz = np.clip(vz, -self.max_velocity_z, self.max_velocity_z)
        
        return vx, vy, vz
    
    def publish_trajectory(self):
        """Publish current target waypoint for visualization"""
        if self.target_waypoint:
            msg = Float64MultiArray()
            msg.data = [self.target_waypoint['x'], 
                       self.target_waypoint['y'], 
                       self.target_waypoint['z']]
            self.trajectory_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    planner = TrajectoryPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
