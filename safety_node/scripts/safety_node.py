#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribe to LaserScan for distance to obstacles
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subscribe to Odometry to get current speed
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Publisher to publish the braking message if needed
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.speed = 0.0  # Initialize speed
        self.get_logger().info('SafetyNode has been started.')

    def odom_callback(self, odom_msg):
        # Update speed from the Odometry message and print the speed
        self.speed = odom_msg.twist.twist.linear.x
        self.get_logger().info(f'Received Odometry: speed={self.speed}')

    def scan_callback(self, scan_msg):
        # Print the number of ranges in the LaserScan message to ensure it's working
        num_ranges = len(scan_msg.ranges)
        self.get_logger().info(f'Received LaserScan: number of ranges={num_ranges}')

        # For now, just print the minimum range (closest object) to verify LaserScan data
        min_range = min(scan_msg.ranges)
        self.get_logger().info(f'Min range from LaserScan: {min_range}')
    
    def brake(self):
        # Create an AckermannDriveStamped message to stop the car
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        self.drive_publisher.publish(brake_msg)
        self.get_logger().info('Emergency braking!')


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
