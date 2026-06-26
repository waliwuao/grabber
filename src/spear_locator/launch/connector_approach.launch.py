"""Launch the trigger-based connector approach controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('spear_locator')
    config = os.path.join(share, 'config', 'connector_approach.yaml')
    return LaunchDescription([
        Node(
            package='spear_locator',
            executable='connector_approach_node',
            name='connector_approach_node',
            output='screen',
            parameters=[config],
        ),
    ])
