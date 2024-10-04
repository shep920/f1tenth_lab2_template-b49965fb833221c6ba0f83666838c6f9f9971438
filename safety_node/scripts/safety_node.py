#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

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

    self.speed = 0.

    def odom_callback(self, odom_msg):
            # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        min_ttc = float('inf')  # Track the minimum iTTC
    
    for i, r in enumerate(scan_msg.ranges):
        if np.isinf(r) or np.isnan(r):
            continue  # Ignore invalid range values
        
        # Calculate the angle of the beam
        angle = scan_msg.angle_min + i * scan_msg.angle_increment
        
        # Calculate the range rate based on the vehicle's speed and angle of the scan
        range_rate = -self.speed * np.cos(angle)
        
        # Calculate iTTC if range rate is negative (moving towards obstacle)
        if range_rate < 0:
            ttc = r / abs(range_rate)
            min_ttc = min(min_ttc, ttc)  # Keep track of the minimum iTTC
    
    # Check if the minimum iTTC is below a threshold, brake if necessary
    if min_ttc < 0.5:  # Example threshold for braking
        self.brake()

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
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
