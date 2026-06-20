"""Process one rosbag, loop its scan, and show recognition in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('spear_locator')
    config = os.path.join(share, 'config', 'recognition.yaml')
    rviz_config = os.path.join(share, 'rviz', 'recognition.rviz')

    bag = LaunchConfiguration('bag')
    expected_count = LaunchConfiguration('expected_count')
    playback_rate = LaunchConfiguration('playback_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag',
            description='Path to a rosbag directory containing metadata.yaml',
        ),
        DeclareLaunchArgument(
            'expected_count',
            default_value='6',
            description='Known target count',
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Rosbag playback rate',
        ),
        Node(
            package='spear_locator',
            executable='spear_recognition_node',
            name='spear_recognition',
            output='screen',
            parameters=[
                config,
                {
                    'expected_count': expected_count,
                    'dataset_label': 'ROSBAG AUTO PROCESSING',
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='spear_recognition_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'play',
                bag,
                '--loop',
                '--delay',
                '1.0',
                '--rate',
                playback_rate,
                '--disable-keyboard-controls',
            ],
            output='screen',
        ),
    ])
