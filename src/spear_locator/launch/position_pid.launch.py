"""Launch the X-position outer-loop PID controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('spear_locator')
    config = os.path.join(share, 'config', 'position_pid.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='2'),
        DeclareLaunchArgument('desired_x_m', default_value='0.0'),
        DeclareLaunchArgument('enabled', default_value='false'),
        Node(
            package='spear_locator',
            executable='spear_position_pid_node',
            name='spear_position_pid',
            output='screen',
            parameters=[
                config,
                {
                    'target_id': LaunchConfiguration('target_id'),
                    'desired_x_m': LaunchConfiguration('desired_x_m'),
                    'enabled': LaunchConfiguration('enabled'),
                },
            ],
        ),
    ])
