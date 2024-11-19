#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math


class FollowPathPID(Node):
    def __init__(self):
        super().__init__('follow_path_pid')

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Point, '/control_errors', 10)  # For errors

        # Initialize subscriber to odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Declare parameters
        self.declare_parameter('path', None)  # List of waypoints
        self.declare_parameter('kp_lin', 0.5)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.0)
        self.declare_parameter('kp_ang', 1.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.0)
        self.declare_parameter('tolerance_lin', 0.05)
        self.declare_parameter('tolerance_ang', 0.05)

        # PID gains
        self.kp_lin = self.get_parameter('kp_lin').value
        self.ki_lin = self.get_parameter('ki_lin').value
        self.kd_lin = self.get_parameter('kd_lin').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.ki_ang = self.get_parameter('ki_ang').value
        self.kd_ang = self.get_parameter('kd_ang').value
        self.tolerance_lin = self.get_parameter('tolerance_lin').value
        self.tolerance_ang = self.get_parameter('tolerance_ang').value

        # Initialize path
        self.path = self.get_parameter('path').value
        if self.path is None:
            self.get_logger().info("No path provided. Using default square path.")
            self.path = [
                {'x': 1.0, 'y': 0.0, 'theta': 0.0},
                {'x': 1.0, 'y': 1.0, 'theta': math.pi / 2},
                {'x': 0.0, 'y': 1.0, 'theta': math.pi},
                {'x': 0.0, 'y': 0.0, 'theta': -math.pi / 2},
            ]

        self.current_waypoint_index = 0
        self.get_logger().info(f"Path initialized with {len(self.path)} waypoints.")

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.x = None
        self.y = None
        self.yaw = None

        # PID control variables
        self.distance_error = 0.0
        self.last_distance_error = 0.0
        self.sum_distance_error = 0.0

        self.angle_error = 0.0
        self.last_angle_error = 0.0
        self.sum_angle_error = 0.0

        self.state = 'wait_for_odom'

        # Flags for debugging
        self.movement_started = False
        self.orientation_adjustment_started = False
        self.waypoints_initialized = False

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        orientation_q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(orientation_q)

    def quaternion_to_yaw(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().info("Waiting for odometry data...")
            return

        if not self.waypoints_initialized:
            start_x = self.x
            start_y = self.y
            for wp in self.path:
                wp['x'] += start_x
                wp['y'] += start_y
            self.waypoints_initialized = True
            self.get_logger().info("Waypoints adjusted relative to starting position.")

        if self.current_waypoint_index >= len(self.path):
            self.get_logger().info("All waypoints have been reached.")
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            return

        waypoint = self.path[self.current_waypoint_index]
        goal_x = waypoint['x']
        goal_y = waypoint['y']
        goal_theta = waypoint['theta']

        dx = goal_x - self.x
        dy = goal_y - self.y
        self.distance_error = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        self.angle_error = self.normalize_angle(target_angle - self.yaw)

        cmd = Twist()

        if self.distance_error > self.tolerance_lin:
            if not self.movement_started:
                self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index + 1}.")
                self.movement_started = True

            self.sum_distance_error += self.distance_error
            delta_distance_error = self.distance_error - self.last_distance_error

            linear_vel = (self.kp_lin * self.distance_error) + \
                         (self.ki_lin * self.sum_distance_error) + \
                         (self.kd_lin * delta_distance_error)

            self.sum_angle_error += self.angle_error
            delta_angle_error = self.angle_error - self.last_angle_error

            angular_vel = (self.kp_ang * self.angle_error) + \
                          (self.ki_ang * self.sum_angle_error) + \
                          (self.kd_ang * delta_angle_error)

            if abs(self.angle_error) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = angular_vel
            else:
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel

            self.last_distance_error = self.distance_error
            self.last_angle_error = self.angle_error
        else:
            self.orientation_error = self.normalize_angle(goal_theta - self.yaw)
            if abs(self.orientation_error) > self.tolerance_ang:
                if not self.orientation_adjustment_started:
                    self.get_logger().info("Adjusting orientation at waypoint.")
                    self.orientation_adjustment_started = True

                self.sum_angle_error += self.orientation_error
                delta_angle_error = self.orientation_error - self.last_angle_error

                angular_vel = (self.kp_ang * self.orientation_error) + \
                              (self.ki_ang * self.sum_angle_error) + \
                              (self.kd_ang * delta_angle_error)

                cmd.linear.x = 0.0
                cmd.angular.z = angular_vel

                self.last_angle_error = self.orientation_error
            else:
                self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached.")
                self.current_waypoint_index += 1
                self.movement_started = False
                self.orientation_adjustment_started = False
                self.sum_distance_error = 0.0
                self.sum_angle_error = 0.0
                cmd = Twist()  # Stop before proceeding

        self.cmd_vel_pub.publish(cmd)

        # Publish errors for debugging
        error_msg = Point()
        error_msg.x = self.distance_error
        error_msg.y = self.angle_error if self.distance_error > self.tolerance_lin else self.orientation_error
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
    node = FollowPathPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
