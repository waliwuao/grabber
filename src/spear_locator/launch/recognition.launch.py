"""Launch multi-frame target recognition for an existing /scan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('spear_locator')
    config = os.path.join(share, 'config', 'recognition.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'expected_count',
            default_value='6',
            description='Known number of targets in the current scene',
        ),
        Node(
            package='spear_locator',
            executable='spear_recognition_node',
            name='spear_recognition',
            output='screen',
            parameters=[
                config,
                {'expected_count': LaunchConfiguration('expected_count')},
            ],
        )
    ])
