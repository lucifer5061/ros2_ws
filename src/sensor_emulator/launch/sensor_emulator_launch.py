#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_emulator')

    # Allow user to override motion type and config file
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'motion',
            default_value='linear',
            description='Motion profile: linear, zigzag, circle, hover'
        )
    ])

    # Node that runs your sensor_emulator_node
    emulator_node = Node(
        package='sensor_emulator',
        executable='sensor_emulator_node',
        name='sensor_emulator_node',
        parameters=[{
            # you can set the motion type here; the node reads it from YAML,
            # but you could override via params if you extend your code.
            'motion_type': LaunchConfiguration('motion')
        }],
        output='screen',
    )

    ld.add_action(emulator_node)
    return ld

