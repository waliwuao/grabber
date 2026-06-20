from glob import glob
from setuptools import find_packages, setup


package_name = 'spear_locator'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charlie',
    maintainer_email='charlie@example.com',
    description='2D LiDAR spear-head ROI and relative position processing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'analyze_bag = spear_locator.analyze_bag:main',
            'exam_answer_node = spear_locator.exam_answer_node:main',
            'spear_locator_node = spear_locator.ros_node:main',
            'spear_position_pid_node = spear_locator.position_pid_node:main',
            'spear_recognition_node = spear_locator.temporal_ros_node:main',
            'synthetic_scan_node = spear_locator.synthetic_scan_node:main',
        ],
    },
)
