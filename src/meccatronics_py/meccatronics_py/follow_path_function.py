#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FollowPathPurePursuit(Node):
    def __init__(self):
        super().__init__('follow_path_pure_pursuit')

        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to /odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Parameters
        self.declare_parameter('look_ahead_distance', 0.5)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('tolerance', 0.05)

        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.path_resolution = self.get_parameter('path_resolution').value
        self.kp = self.get_parameter('kp').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.tolerance = self.get_parameter('tolerance').value

        # Generate circular path
        self.path_points = self.generate_circular_path(center_x=0.0, center_y=0.0, radius=1.0, num_points=10)

        # Robot state
        self.x = None
        self.y = None
        self.yaw = None

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # State
        self.state = 'approach_start'  # States: approach_start, follow_path

    def generate_circular_path(self, center_x, center_y, radius, num_points):
        """
        Generate points forming a circle.
        """
        path = []
        for i in range(num_points):
            theta = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            path.append((x, y))
        self.get_logger().info(f"Generated circular path with {len(path)} points.")
        return path

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(orientation_q)

    def quaternion_to_yaw(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_target_point(self):
        """
        Find the closest point on the path ahead of the robot.
        """
        closest_distance = float('inf')
        target_point = None

        for point in self.path_points:
            dist = math.hypot(point[0] - self.x, point[1] - self.y)
            if dist >= self.look_ahead_distance and dist < closest_distance:
                closest_distance = dist
                target_point = point

        return target_point

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().info("Waiting for odometry data...")
            return

        if self.state == 'approach_start':
            # Approach the first point in the path
            start_point = self.path_points[0]
            distance_to_start = math.hypot(start_point[0] - self.x, start_point[1] - self.y)

            if distance_to_start < self.tolerance:
                self.get_logger().info("Reached starting point. Following path.")
                self.state = 'follow_path'
            else:
                self.go_to_point(start_point)
            return

        if self.state == 'follow_path':
            # Find the target point
            target_point = self.find_target_point()
            if target_point is None:
                self.get_logger().info("No valid target point found. Stopping robot.")
                self.cmd_vel_pub.publish(Twist())
                return

            # Compute control commands
            self.go_to_point(target_point)

    def go_to_point(self, point):
        """
        Navigate to a specific point (local target point).
        """
        dx = point[0] - self.x
        dy = point[1] - self.y

        # Transform to robot frame
        local_x = math.cos(-self.yaw) * dx - math.sin(-self.yaw) * dy
        local_y = math.sin(-self.yaw) * dx + math.cos(-self.yaw) * dy

        # Compute control commands
        cmd = Twist()
        cmd.linear.x = min(self.max_linear_speed, self.kp * local_x)
        cmd.angular.z = 2 * local_y / (self.look_ahead_distance ** 2)

        # Clamp angular speed
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
