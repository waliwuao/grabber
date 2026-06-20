"""Launch the spear locator against an existing /scan topic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('spear_locator')
    config = os.path.join(package_share, 'config', 'spear_locator.yaml')
    return LaunchDescription([
        Node(
            package='spear_locator',
            executable='spear_locator_node',
            name='spear_locator',
            output='screen',
            parameters=[config],
        )
    ])
