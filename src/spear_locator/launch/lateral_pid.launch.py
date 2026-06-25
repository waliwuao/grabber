"""Launch the trigger-based lateral PID controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('spear_locator')
    config = os.path.join(share, 'config', 'lateral_pid.yaml')
    return LaunchDescription([
        Node(
            package='spear_locator',
            executable='lateral_pid_node',
            name='lateral_pid',
            output='screen',
            parameters=[config],
        ),
    ])
