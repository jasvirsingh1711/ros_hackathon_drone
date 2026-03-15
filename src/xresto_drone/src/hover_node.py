#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu, LaserScan
import math


def euler_from_quaternion(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    return roll, pitch


class HoverController(Node):
    def __init__(self):
        super().__init__('hover_controller')
        self.pub_       = self.create_publisher(Wrench, '/drone/cmd_force', 10)
        self.imu_sub    = self.create_subscription(Imu,       '/drone/imu',       self.imu_cb,   10)
        self.lidar_sub  = self.create_subscription(LaserScan, '/drone/altitude',   self.lidar_cb, 10)
        self.nav_sub    = self.create_subscription(Wrench,    '/drone/nav_force',  self.nav_cb,   10)

        self.nav_cmd        = Wrench()
        self.base_thrust    = 8.86
        self.target_alt     = 0.5  # Default, overridden by trajectory planner
        self.current_thrust = self.base_thrust

        self.kp_alt, self.ki_alt, self.kd_alt = 3.0, 0.5, 2.0
        self.int_alt    = 0.0
        self.last_alt_e = 0.0

        self.kp_att, self.ki_att, self.kd_att = 4.0, 0.5, 1.2
        self.int_roll   = 0.0
        self.int_pitch  = 0.0

        self.vel_x         = 0.0
        self.VEL_DECAY     = 0.98
        self.VEL_DAMP_GAIN = 0.4

        self.last_lidar_t = self.get_clock().now()
        self.last_imu_t   = self.get_clock().now()
        self.get_logger().info(f"Hover active, target alt: {self.target_alt}m")

    def nav_cb(self, msg):
        self.nav_cmd = msg
        # Update altitude setpoint from trajectory planner (force.z is altitude in meters)
        if msg.force.z > 0.1:
            self.target_alt = msg.force.z

    def lidar_cb(self, msg):
        if not msg.ranges: return
        alt = msg.ranges[0]
        if not math.isfinite(alt): return
        now = self.get_clock().now()
        dt  = (now - self.last_lidar_t).nanoseconds / 1e9
        if dt < 0.001: return
        self.last_lidar_t = now

        e   = self.target_alt - alt
        de  = (e - self.last_alt_e) / dt
        self.int_alt       = max(min(self.int_alt + e * dt, 3.0), -3.0)
        adj                = self.kp_alt*e + self.ki_alt*self.int_alt + self.kd_alt*de
        self.current_thrust = max(min(self.base_thrust + adj, 15.0), 0.0)
        self.last_alt_e    = e

    def imu_cb(self, msg):
        q  = msg.orientation
        roll, pitch = euler_from_quaternion(q.x, q.y, q.z, q.w)

        now = self.get_clock().now()
        dt  = (now - self.last_imu_t).nanoseconds / 1e9
        self.last_imu_t = now
        if dt < 0.001: dt = 0.01

        self.int_roll  = max(min(self.int_roll  + roll  * dt, 2.0), -2.0)
        self.int_pitch = max(min(self.int_pitch + pitch * dt, 2.0), -2.0)

        tx = -(self.kp_att*roll  + self.ki_att*self.int_roll  + self.kd_att*msg.angular_velocity.x)
        ty = -(self.kp_att*pitch + self.ki_att*self.int_pitch + self.kd_att*msg.angular_velocity.y)

        self.vel_x = (self.vel_x + msg.linear_acceleration.x * dt) * self.VEL_DECAY

        nav_fx = self.nav_cmd.force.x
        if abs(nav_fx) < 0.01:
            damping_fx = -self.vel_x * self.VEL_DAMP_GAIN
            damping_fx = max(min(damping_fx, 0.6), -0.6)
        else:
            damping_fx = 0.0

        w = Wrench()
        w.force.x  = nav_fx + damping_fx
        w.force.y  = self.nav_cmd.force.y
        w.force.z  = self.current_thrust
        w.torque.x = tx
        w.torque.y = ty
        w.torque.z = self.nav_cmd.torque.z - 1.2 * msg.angular_velocity.z
        self.pub_.publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
