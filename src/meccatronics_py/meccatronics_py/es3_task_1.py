#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class GoToPosePID(Node):
    def __init__(self):
        super().__init__('go_to_pose_pid')

        # Initialize publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for errors
        self.error_pub = self.create_publisher(Point, '/control_errors', 10)

        # Initialize subscriber to odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Declare and get parameters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)  # Provide a default value
        self.declare_parameter('kp_lin', 0.5)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.0)
        self.declare_parameter('kp_ang', 1.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.0)
        self.declare_parameter('tolerance_lin', 0.05)
        self.declare_parameter('tolerance_ang', 0.05)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_theta = self.get_parameter('goal_theta').value
        self.kp_lin = self.get_parameter('kp_lin').value
        self.ki_lin = self.get_parameter('ki_lin').value
        self.kd_lin = self.get_parameter('kd_lin').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.ki_ang = self.get_parameter('ki_ang').value
        self.kd_ang = self.get_parameter('kd_ang').value
        self.tolerance_lin = self.get_parameter('tolerance_lin').value
        self.tolerance_ang = self.get_parameter('tolerance_ang').value

        self.get_logger().info(
            f"New goal set to x: {self.goal_x}, y: {self.goal_y}, theta: {self.goal_theta}")

        # Set control loop rate
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.control_loop)

        # Robot state
        self.x = None  # Initialize as None to wait for odometry data
        self.y = None
        self.yaw = None

        # PID control variables
        self.distance_error = 0.0
        self.last_distance_error = 0.0
        self.sum_distance_error = 0.0

        self.angle_error = 0.0
        self.last_angle_error = 0.0
        self.sum_angle_error = 0.0

        self.orientation_error = 0.0  # For final orientation adjustment

        self.state = 'wait_for_odom'  # Start by waiting for odometry data

        # Flags for logging
        self.movement_started = False
        self.orientation_adjustment_started = False

    def odom_callback(self, msg):
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract orientation as a quaternion
        orientation_q = msg.pose.pose.orientation

        # Convert quaternion to yaw angle
        self.yaw = self.quaternion_to_yaw(orientation_q)

    def quaternion_to_yaw(self, quat):
        """
        Convert quaternion to yaw angle (in radians).
        """
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.state == 'goal_reached':
            return

        if self.x is None or self.y is None or self.yaw is None:
            # Waiting for odometry data
            self.get_logger().info("Waiting for odometry data...")
            return

        if self.state == 'wait_for_odom':
            # Check if we are already at the goal position
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            self.distance_error = math.hypot(dx, dy)

            # Compute orientation error
            orientation_error = self.normalize_angle(self.goal_theta - self.yaw)

            # Check position and orientation tolerances
            position_reached = self.distance_error < self.tolerance_lin
            orientation_reached = abs(orientation_error) < self.tolerance_ang

            if position_reached and orientation_reached:
                self.get_logger().info(
                    "Already at the goal position and orientation.")
                self.state = 'goal_reached'
                return
            elif position_reached:
                self.get_logger().info(
                    "At goal position. Adjusting orientation...")
                self.state = 'adjust_orientation'
            else:
                self.state = 'move_to_goal'
                self.get_logger().info("Starting movement towards the goal.")

        # Compute the distance error
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        self.distance_error = math.hypot(dx, dy)

        # Compute the angle to the goal
        target_angle = math.atan2(dy, dx)
        self.angle_error = self.normalize_angle(target_angle - self.yaw)

        cmd = Twist()

        if self.state == 'move_to_goal':
            if not self.movement_started:
                self.get_logger().info("Starting movement towards the goal.")
                self.movement_started = True

            # PID calculations for linear velocity
            self.sum_distance_error += self.distance_error
            delta_distance_error = self.distance_error - self.last_distance_error

            linear_vel = (self.kp_lin * self.distance_error) + \
                (self.ki_lin * self.sum_distance_error) + \
                (self.kd_lin * delta_distance_error)

            # PID calculations for angular velocity
            self.sum_angle_error += self.angle_error
            delta_angle_error = self.angle_error - self.last_angle_error

            angular_vel = (self.kp_ang * self.angle_error) + \
                (self.ki_ang * self.sum_angle_error) + \
                (self.kd_ang * delta_angle_error)

            # If the angle error is significant, rotate in place
            if abs(self.angle_error) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = angular_vel
            else:
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel

            self.cmd_vel_pub.publish(cmd)

            # Update last errors
            self.last_distance_error = self.distance_error
            self.last_angle_error = self.angle_error

            # Check if position goal is reached
            if self.distance_error < self.tolerance_lin:
                self.get_logger().info(
                    "Position reached. Adjusting orientation...")
                self.state = 'adjust_orientation'
                self.movement_started = False  # Reset flag

        elif self.state == 'adjust_orientation':
            if not self.orientation_adjustment_started:
                self.get_logger().info(
                    f"Adjusting orientation to θ={self.goal_theta} radians.")
                self.orientation_adjustment_started = True

            # Compute orientation error
            self.orientation_error = self.normalize_angle(
                self.goal_theta - self.yaw)

            # PID calculations for angular velocity
            self.sum_angle_error += self.orientation_error
            delta_angle_error = self.orientation_error - self.last_angle_error

            angular_vel = (self.kp_ang * self.orientation_error) + \
                (self.ki_ang * self.sum_angle_error) + \
                (self.kd_ang * delta_angle_error)

            cmd.linear.x = 0.0
            cmd.angular.z = angular_vel

            self.cmd_vel_pub.publish(cmd)

            # Update last error
            self.last_angle_error = self.orientation_error

            # Check if orientation goal is reached
            if abs(self.orientation_error) < self.tolerance_ang:
                self.get_logger().info(
                    "Goal reached with desired orientation!")
                self.state = 'goal_reached'
                self.cmd_vel_pub.publish(Twist())  # Stop the robot
                self.orientation_adjustment_started = False  # Reset flag

        # Publish errors for data collection
        error_msg = Point()
        error_msg.x = self.distance_error
        error_msg.y = self.angle_error if self.state == 'move_to_goal' else self.orientation_error
        error_msg.z = 0.0
        self.error_pub.publish(error_msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)

    node = GoToPosePID()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
