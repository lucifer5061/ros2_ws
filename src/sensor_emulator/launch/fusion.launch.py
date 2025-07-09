#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_emulator', executable='sensor_emulator_node', name='emulator'),
        Node(package='sensor_emulator', executable='test_input_pub', name='test_publisher'),
        Node(package='sensor_emulator', executable='cue_fusion_node', name='fusion'),
    ])

