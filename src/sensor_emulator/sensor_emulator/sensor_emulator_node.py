#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transformations import quaternion_from_euler
import math

class SensorEmulatorNode(Node):
    def __init__(self):
        super().__init__('sensor_emulator_node')
        
        # Create a publisher to /imu/data topic
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # Create a timer to publish data every 0.1 seconds (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        # Track current step or rotation angle
        self.counter = 0

        # Choose motion type – useful for config extension later
        self.motion_type = "rotation"  # Could be extended to linear, zig-zag etc.

        self.get_logger().info('Sensor Emulator Node has been started.')

    def publish_imu_data(self):
        msg = Imu()

        # Set message timestamp and frame ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"


        # Simulate yaw rotation based on counter
        angle = math.radians(self.counter)
        
        # Apply rotation if motion_type is rotation
        if self.motion_type == "rotation":
            q = quaternion_from_euler(0.0, 0.0, angle)  # Yaw (Z-axis)
        else:
            q = quaternion_from_euler(0.0, 0.0, 0.0)    # No rotation

        # Set orientation (quaternion)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        # Simulated angular velocity in rad/s
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.1  # Constant yaw rotation

        # Simulated linear acceleration in m/s^2
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Gravity along Z

        # Publish the message to ROS 2 topic
        self.publisher.publish(msg)

        # Increment counter to simulate changing orientation
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SensorEmulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

