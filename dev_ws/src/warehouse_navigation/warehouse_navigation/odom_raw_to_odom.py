#!/usr/bin/env python3
"""
Subscribes to /odom_raw (unsynced time from Pico)
Republishes to /odom with ROS clock timestamp
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import copy

class OdomRawToOdom(Node):
    def __init__(self):
        super().__init__('odom_raw_to_odom')

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("odom_raw_to_odom node started. Waiting for /odom_raw...")

    def odom_callback(self, msg: Odometry):
        # 1. Deep copy the message
        odom_out = copy.deepcopy(msg)

        # 2. Update timestamp with ROS time
        now = self.get_clock().now().to_msg()
        odom_out.header.stamp = now

        # 3. Publish to /odom
        self.odom_pub.publish(odom_out)

        # 4. Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = odom_out.pose.pose.position.x
        t.transform.translation.y = odom_out.pose.pose.position.y
        t.transform.translation.z = odom_out.pose.pose.position.z
        t.transform.rotation = odom_out.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRawToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()