#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math


class FollowCircularPathPID(Node):
    def __init__(self):
        super().__init__('follow_circular_path_pid')

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Point, '/control_errors', 10)

        # Initialize subscriber to odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Declare parameters
        self.declare_parameter('radius', 1.0)  # Radius of the circle
        self.declare_parameter('center_x', 0.0)  # Center X
        self.declare_parameter('center_y', 0.0)  # Center Y
        self.declare_parameter('num_points', 36)  # Number of points to generate
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

        # Generate circular path
        self.radius = self.get_parameter('radius').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.num_points = self.get_parameter('num_points').value
        self.path = self.generate_circular_path(self.radius, self.center_x, self.center_y, self.num_points)
        self.current_waypoint_index = 0
        self.get_logger().info(f"Circular path initialized with {self.num_points} waypoints around center ({self.center_x}, {self.center_y}).")

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

        # State tracking
        self.state = 'wait_for_odom'
        self.movement_started = False

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

    def generate_circular_path(self, radius, center_x, center_y, num_points):
        path = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            path.append({'x': x, 'y': y, 'theta': angle})
        return path

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().info("Waiting for odometry data...")
            return

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

        if abs(self.angle_error) > self.tolerance_ang:
            angular_vel = self.kp_ang * self.angle_error + \
                          self.ki_ang * self.sum_angle_error + \
                          self.kd_ang * (self.angle_error - self.last_angle_error)

            cmd.angular.z = angular_vel
            cmd.linear.x = 0.0
            self.get_logger().info(f"Adjusting to angle: {self.angle_error:.2f} radians.")
        else:
            linear_vel = self.kp_lin * self.distance_error + \
                         self.ki_lin * self.sum_distance_error + \
                         self.kd_lin * (self.distance_error - self.last_distance_error)

            cmd.linear.x = linear_vel
            cmd.angular.z = 0.0
            self.get_logger().info(f"Moving towards waypoint: Distance error {self.distance_error:.2f} meters.")

        self.last_distance_error = self.distance_error
        self.last_angle_error = self.angle_error
        self.sum_distance_error += self.distance_error
        self.sum_angle_error += self.angle_error

        if self.distance_error < self.tolerance_lin:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached.")
            self.current_waypoint_index += 1
            self.sum_distance_error = 0.0
            self.sum_angle_error = 0.0

        self.cmd_vel_pub.publish(cmd)

        # Publish errors for debugging
        error_msg = Point()
        error_msg.x = self.distance_error
        error_msg.y = self.angle_error
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
    node = FollowCircularPathPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
