"""ROS 2 node for extracting target positions from LaserScan."""

import json
import math
from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import Detection, ProcessorConfig, process_scan


class SpearLocatorNode(Node):
    """Subscribe to /scan and publish filtered points and target positions."""

    def __init__(self) -> None:
        super().__init__('spear_locator')
        self._declare_parameters()
        self._config = self._read_config()

        input_topic = self.get_parameter('input_topic').value
        self._cloud_pub = self.create_publisher(
            PointCloud2, 'spear_locator/filtered_points', 10
        )
        self._poses_pub = self.create_publisher(
            PoseArray, 'spear_locator/poses', 10
        )
        self._json_pub = self.create_publisher(
            String, 'spear_locator/detections_json', 10
        )
        self._markers_pub = self.create_publisher(
            MarkerArray, 'spear_locator/markers', 10
        )
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Listening on {input_topic}; ROI x=[{self._config.x_min_m}, '
            f'{self._config.x_max_m}] m, y=[{self._config.y_min_m}, '
            f'{self._config.y_max_m}] m'
        )

    def _declare_parameters(self) -> None:
        defaults = ProcessorConfig()
        self.declare_parameter('input_topic', '/scan')
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)
        self.declare_parameter('marker_scale_m', 0.06)
        self.declare_parameter('id_sort_axis', 'x')
        self.declare_parameter('id_sort_ascending', True)

    def _read_config(self) -> ProcessorConfig:
        values = {
            name: self.get_parameter(name).value
            for name in ProcessorConfig().__dict__
        }
        return ProcessorConfig(**values)

    def _scan_callback(self, scan: LaserScan) -> None:
        points, detections = process_scan(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._config,
            scan.intensities,
            sort_axis=str(self.get_parameter('id_sort_axis').value),
            sort_ascending=bool(
                self.get_parameter('id_sort_ascending').value
            ),
        )
        self._publish_cloud(scan.header, points)
        self._publish_poses(scan.header, detections)
        self._publish_markers(scan.header, detections)
        self._publish_json(scan.header, detections)

    def _publish_cloud(self, header: Header, points) -> None:
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity', offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        cloud_points = [
            (point.x_m, point.y_m, 0.0, point.intensity) for point in points
        ]
        self._cloud_pub.publish(
            point_cloud2.create_cloud(header, fields, cloud_points)
        )

    def _publish_poses(
        self, header: Header, detections: List[Detection]
    ) -> None:
        message = PoseArray()
        message.header = header
        for detection in detections:
            pose = Pose()
            pose.position.x = detection.x_m
            pose.position.y = detection.y_m
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self._poses_pub.publish(message)

    def _publish_markers(
        self, header: Header, detections: List[Detection]
    ) -> None:
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        scale = float(self.get_parameter('marker_scale_m').value)

        for detection in detections:
            sphere = Marker()
            sphere.header = header
            sphere.ns = 'spear_centers'
            sphere.id = detection.target_id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = detection.x_m
            sphere.pose.position.y = detection.y_m
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = scale
            sphere.scale.y = scale
            sphere.scale.z = scale
            sphere.color.r = 1.0
            sphere.color.g = 0.25
            sphere.color.b = 0.05
            sphere.color.a = 1.0
            message.markers.append(sphere)

            label = Marker()
            label.header = header
            label.ns = 'spear_labels'
            label.id = detection.target_id * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = detection.x_m
            label.pose.position.y = detection.y_m
            label.pose.position.z = scale
            label.pose.orientation.w = 1.0
            label.scale.z = scale
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.2
            label.color.a = 1.0
            label.text = (
                f'ID {detection.target_id}\\n'
                f'({detection.x_m:.3f}, {detection.y_m:.3f}) m'
            )
            message.markers.append(label)

        self._markers_pub.publish(message)

    def _publish_json(
        self, header: Header, detections: List[Detection]
    ) -> None:
        reference = detections[0] if detections else None
        adjacent_spacings = [
            math.hypot(
                detections[index].x_m - detections[index - 1].x_m,
                detections[index].y_m - detections[index - 1].y_m,
            )
            for index in range(1, len(detections))
        ]
        payload = {
            'frame_id': header.frame_id,
            'stamp': {
                'sec': header.stamp.sec,
                'nanosec': header.stamp.nanosec,
            },
            'count': len(detections),
            'mean_adjacent_spacing_m': (
                round(sum(adjacent_spacings) / len(adjacent_spacings), 6)
                if adjacent_spacings else None
            ),
            'targets': [
                {
                    'id': item.target_id,
                    'x_m': round(item.x_m, 6),
                    'y_m': round(item.y_m, 6),
                    'relative_to_id0_x_m': (
                        round(item.x_m - reference.x_m, 6)
                        if reference else None
                    ),
                    'relative_to_id0_y_m': (
                        round(item.y_m - reference.y_m, 6)
                        if reference else None
                    ),
                    'range_m': round(item.range_m, 6),
                    'bearing_deg': round(math.degrees(item.bearing_rad), 4),
                    'point_count': item.point_count,
                    'width_m': round(item.width_m, 6),
                    'observed': True,
                }
                for item in detections
            ],
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._json_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpearLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
