#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_emulator')
    plot_script_path = os.path.join(pkg_share, 'scripts', 'plot_fused_detections.py')

    ld = LaunchDescription([
        # Allow user to override motion type
        DeclareLaunchArgument(
            'motion',
            default_value='linear',
            description='Motion profile: linear, zigzag, circle, hover'
        )
    ])

    # 1) Sensor emulator (IMU, GPS, /track/fake_target)
    emulator_node = Node(
        package='sensor_emulator',
        executable='sensor_emulator_node',
        name='sensor_emulator_node',
        parameters=[{
            'motion_type': LaunchConfiguration('motion')
        }],
        output='screen',
    )
    ld.add_action(emulator_node)

    # 2) Test input publisher (mock visual & RF cues)
    test_pub_node = Node(
        package='sensor_emulator',
        executable='test_input_pub',
        name='test_input_pub',
        output='screen',
    )
    ld.add_action(test_pub_node)

    # 3) Cue fusion node (publishes /synthetic_target_cue)
    fusion_node = Node(
        package='sensor_emulator',
        executable='cue_fusion_node',
        name='cue_fusion_node',
        output='screen',
    )
    ld.add_action(fusion_node)

    # 4) Plotting script (plot_fused_detections.py)
    plot_script = ExecuteProcess(
        cmd=['python3', plot_script_path],
        output='screen'
    )
    ld.add_action(plot_script)

    return ld

