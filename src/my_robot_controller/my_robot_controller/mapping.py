#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

        # Parameters for navigation and obstacle detection
        self._front_threshold = 1.0  # Minimum distance in meters to consider obstacle in front
        self._turn_speed = 0.5  # Turn speed (angular velocity)
        self._move_speed = 0.2  # Forward speed (linear velocity)
        self._safety_distance = 0.5  # Distance to be maintained from obstacles
        self._scan_range_width = 30  # Range width on each side (degrees)

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Convert scan ranges into numpy array for easier manipulation
        scan_ranges = np.array(scan.ranges)

        # Handle invalid or NaN readings
        scan_ranges = np.nan_to_num(scan_ranges, nan=float('inf'))

        # Extract directional distances
        front_distances = np.concatenate([scan_ranges[:self._scan_range_width], scan_ranges[-self._scan_range_width:]])
        left_distances = scan_ranges[90 - self._scan_range_width:90 + self._scan_range_width]
        right_distances = scan_ranges[270 - self._scan_range_width:270 + self._scan_range_width]

        front_min = np.min(front_distances)
        left_min = np.min(left_distances)
        right_min = np.min(right_distances)

        self.get_logger().info(f"Front: {front_min}, Left: {left_min}, Right: {right_min}")

        # Navigation logic based on obstacle detection
        if front_min < self._front_threshold:  # Obstacle detected in front
            if left_min > right_min:  # More space on the left side
                cmd.linear.x = 0.0
                cmd.angular.z = self._turn_speed  # Turn left
            else:  # More space on the right side
                cmd.linear.x = 0.0
                cmd.angular.z = -self._turn_speed  # Turn right
        elif front_min < self._safety_distance:  # A safety threshold
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0  # Stop if too close to an obstacle
        else:  # No immediate obstacles
            cmd.linear.x = self._move_speed  # Move forward
            cmd.angular.z = 0.0  # Go straight

        # Publish the movement command
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()

