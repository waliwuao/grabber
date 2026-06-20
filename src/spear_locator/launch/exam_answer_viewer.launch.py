"""Launch RViz with the frozen blind-test answer marker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('spear_locator')
    rviz_config = os.path.join(share, 'rviz', 'exam_answer.rviz')
    return LaunchDescription([
        Node(
            package='spear_locator',
            executable='exam_answer_node',
            name='exam_answer_marker',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='exam_answer_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
