import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # --- CONFIGURATION (Must match URDF) ---
        self.wheel_radius = 0.05  # Fixed to match your URDF (0.05)
        self.wheel_base = 0.35    # Fixed to match your URDF/Pico code

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # --- STATE VARIABLES ---
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.last_time = self.get_clock().now()

        # Initial Message
        self.joint_state_msg = JointState()
        # FIXED: Names must match URDF exactly
        self.joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_state_msg.position = [0.0, 0.0]
        self.joint_state_msg.velocity = [0.0, 0.0]
        self.joint_state_msg.effort = []

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now()
        
        # Calculate time passed since last message (dt)
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # If dt is huge (first run), skip
        if dt > 1.0:
            return

        # 1. Get Velocities from Odom
        v = msg.twist.twist.linear.x        # Linear Velocity
        omega = msg.twist.twist.angular.z   # Angular Velocity
        
        # 2. Convert to Wheel Velocities (rad/s)
        # v_left = (v - omega * L/2) / r
        left_vel = (v - (omega * self.wheel_base / 2.0)) / self.wheel_radius
        right_vel = (v + (omega * self.wheel_base / 2.0)) / self.wheel_radius
        
        # 3. Integrate to get Position (Angle)
        # new_angle = old_angle + (velocity * time)
        self.left_wheel_angle += left_vel * dt
        self.right_wheel_angle += right_vel * dt

        # 4. Update Message
        self.joint_state_msg.header.stamp = msg.header.stamp
        self.joint_state_msg.header.frame_id="odom"
        self.joint_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_state_msg.velocity = [left_vel, right_vel]
        
        # 5. Publish
        self.joint_state_pub.publish(self.joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()