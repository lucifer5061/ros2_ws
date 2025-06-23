from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_emulator',
            executable='sensor_emulator_node',
            name='sensor_emulator_node',
            output='screen'
        )
    ])
