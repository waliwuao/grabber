"""Launch the STL-27L driver, spear processing, and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    locator_share = get_package_share_directory('spear_locator')
    driver_share = get_package_share_directory('ldlidar_stl_ros2')
    config = os.path.join(locator_share, 'config', 'spear_locator.yaml')
    rviz_config = os.path.join(locator_share, 'rviz', 'spear_locator.rviz')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_share, 'launch', 'stl27l.launch.py')
        )
    )
    return LaunchDescription([
        driver,
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
