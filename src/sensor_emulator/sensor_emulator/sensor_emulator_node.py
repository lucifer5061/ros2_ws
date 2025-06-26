#!/usr/bin/env python3

import os
import math
import yaml
import random
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from transformations import quaternion_from_euler

# Import motion profiles from module
import sensor_emulator.motion_profiles as motion_profiles

class SensorEmulatorNode(Node):
    def __init__(self):
        super().__init__('sensor_emulator_node')

        # Locate and load unified config
        share_dir = get_package_share_directory('sensor_emulator')
        cfg_path = os.path.join(share_dir, 'config', 'motion_profile.yaml')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        mot = cfg['motion']
        imu_cfg = cfg['imu']
        gps_cfg = cfg['gps']

        # Store config
        self.motion_type = mot['type']
        self.motion_cfg = mot
        self.dt = 1.0 / imu_cfg['rate']
        self.base_lat = gps_cfg['base_lat']
        self.base_lon = gps_cfg['base_lon']
        self.base_alt = gps_cfg['base_alt']

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.target_pub = self.create_publisher(String, '/track/fake_target', 10)

        # Timer for synchronized publishing
        self.timer = self.create_timer(self.dt, self.publish_sensors)
        self.counter = 0
        self.last_x = 0.0
        self.last_y = 0.0

        self.get_logger().info(f"Sensor Emulator Node started with motion: {self.motion_type}")

    def publish_sensors(self):
        t = self.counter * self.dt

        # Compute motion state based on profile
        if self.motion_type == 'linear':
            x, y, yaw = motion_profiles.linear_motion(t, speed=self.motion_cfg['speed'])
        elif self.motion_type == 'zigzag':
            x, y, yaw = motion_profiles.zigzag_motion(
                t, speed=self.motion_cfg['speed'], period=self.motion_cfg['period'])
        elif self.motion_type == 'circle':
            x, y, yaw = motion_profiles.circle_motion(
                t, radius=self.motion_cfg['radius'], omega=self.motion_cfg['omega'])
        else:  # hover
            x, y, yaw = motion_profiles.hover_drift(t, amp=self.motion_cfg['drift_amp'])

        # Unified timestamp
        now_msg = self.get_clock().now().to_msg()

        # IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now_msg
        imu_msg.header.frame_id = 'imu_link'
        # Orientation (quaternion)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = q
        # Angular velocity (rad/s)
        if self.motion_type == 'circle':
            imu_msg.angular_velocity.z = self.motion_cfg['omega']
        else:
            imu_msg.angular_velocity.z = 0.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        # Linear acceleration (m/s^2)
        if self.motion_type == 'circle':
            v = self.motion_cfg['omega'] * self.motion_cfg['radius']
            a_c = v**2 / self.motion_cfg['radius']
            imu_msg.linear_acceleration.x = -a_c * math.cos(yaw)
            imu_msg.linear_acceleration.y = -a_c * math.sin(yaw)
        else:
            imu_msg.linear_acceleration.x = (x - self.last_x) / self.dt
            imu_msg.linear_acceleration.y = (y - self.last_y) / self.dt
        imu_msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu_msg)

        # GPS message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = now_msg
        gps_msg.header.frame_id = 'gps_link'
        gps_msg.latitude = self.base_lat + (x / 111000.0)
        gps_msg.longitude = self.base_lon + (y / (111000.0 * math.cos(math.radians(self.base_lat))))
        gps_msg.altitude = self.base_alt
        self.gps_pub.publish(gps_msg)

        # Target detection (simple echo of position)
        target_msg = String()
        target_msg.data = f"target at {gps_msg.latitude:.6f}, {gps_msg.longitude:.6f}"
        self.target_pub.publish(target_msg)

        # Update last position and counter
        self.last_x, self.last_y = x, y
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SensorEmulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

