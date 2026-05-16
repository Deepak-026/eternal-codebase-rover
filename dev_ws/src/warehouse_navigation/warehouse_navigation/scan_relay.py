#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        # Last odom stamp holder
        self.last_odom_stamp = None

        # Publisher to /scan
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

        # Subscriber to /scan_raw
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10)

        # Subscriber to /odom to update timestamp
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.get_logger().info("ScanRelay started: /scan_raw → /scan (timestamp synced with /odom)")

    def odom_callback(self, msg: Odometry):
        # Save the new odom timestamp
        self.last_odom_stamp = msg.header.stamp

    def scan_callback(self, msg: LaserScan):
        # Replace timestamp with most recent odom timestamp
        if self.last_odom_stamp is not None:
            msg.header.stamp = self.last_odom_stamp
        else:
            self.get_logger().warn("Waiting for /odom messages to sync timestamp...")

        # Publish modified message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()