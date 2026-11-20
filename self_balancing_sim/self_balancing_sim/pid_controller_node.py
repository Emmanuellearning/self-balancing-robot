#!/usr/bin/env python3
"""
Simple PID Controller Node for a Self-Balancing Robot.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def get_pitch_from_quaternion(x, y, z, w):
    # Convert quaternion to pitch angle (in radians)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


class BalancingController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Inner loop: Pitch balancing PID gains
        self.kp = 22.0
        self.ki = 0.4
        self.kd = 2.5

        # Outer loop: Position holding PD gains
        self.kp_pos = 0.15
        self.kd_pos = 0.30

        # State variables
        self.pitch = 10
        self.pitch_offset = 0.0# Balance pitch trim / offset (radians)
        self.pitch_rate = 0.0
        self.pos_x = 0.0
        self.vel_x = 0.0
        self.target_x = 5.0

        self.integral_error = 0.0
        self.has_imu = False
        self.has_odom = False
        self.target_set = False

        # Subscribers
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for wheel velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: 100 Hz (every 0.01s)
        self.dt = 0.01
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Self-balancing PID controller started.')

    def imu_callback(self, msg):
        q = msg.orientation
        self.pitch = get_pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self.pitch_rate = msg.angular_velocity.y
        self.has_imu = True

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.vel_x = msg.twist.twist.linear.x
        if not self.target_set:
            self.target_x = self.pos_x
            self.target_set = True
        self.has_odom = True

    def control_loop(self):
        if not self.has_imu:
            return

        # Safety check: if robot has fallen (pitch > 40 degrees), stop wheels
        if abs(self.pitch) > 0.70:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.integral_error = 0.0
            return

        # 1. Outer Loop: calculate desired lean angle to stay in place
        pos_error = self.pos_x - self.target_x
        target_pitch = -(self.kp_pos * pos_error + self.kd_pos * self.vel_x)

        # Clamp target pitch to +/- 8.5 degrees (0.15 rad)
        target_pitch = max(-0.15, min(0.15, target_pitch))

        # 2. Inner Loop: PID control on pitch error
        pitch_error = (self.pitch - self.pitch_offset) - target_pitch

        # Integral term with anti-windup clamping
        self.integral_error += pitch_error * self.dt
        self.integral_error = max(-0.2, min(0.2, self.integral_error))

        # Calculate wheel velocity command
        u = (self.kp * pitch_error) + (self.ki * self.integral_error) + (self.kd * self.pitch_rate)

        # Limit maximum speed to 4.0 m/s
        u = max(-4.0, min(4.0, u))

        # Publish command to wheels
        cmd = Twist()
        cmd.linear.x = float(u)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BalancingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
