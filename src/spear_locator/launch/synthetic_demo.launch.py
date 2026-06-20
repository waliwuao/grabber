"""Run a complete no-hardware synthetic scan and RViz demonstration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('spear_locator')
    config = os.path.join(package_share, 'config', 'spear_locator.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'spear_locator.rviz')
    return LaunchDescription([
        Node(
            package='spear_locator',
            executable='synthetic_scan_node',
            name='synthetic_spear_scan',
            output='screen',
        ),
        Node(
            package='spear_locator',
            executable='spear_locator_node',
            name='spear_locator',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='spear_locator_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
