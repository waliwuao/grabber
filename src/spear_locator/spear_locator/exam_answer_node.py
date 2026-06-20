"""Publish the frozen six-point blind-test answer as RViz markers."""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


# Frozen before ground truth is revealed. Ordered by signed bearing.
ANSWERS = [
    (-0.275, -0.230),
    (-0.155, -0.260),
    (-0.050, -0.230),
    (0.080, -0.220),
    (0.210, -0.220),
    (0.305, -0.225),
]


class ExamAnswerNode(Node):
    """Overlay the submitted six target points on every replayed scan."""

    def __init__(self) -> None:
        super().__init__('exam_answer_marker')
        self._publisher = self.create_publisher(
            MarkerArray, '/spear_recognition/markers', 10
        )
        self._subscription = self.create_subscription(
            LaserScan, '/scan', self._callback, qos_profile_sensor_data
        )
        self.get_logger().info('Frozen answer contains 6 target points')

    def _callback(self, scan: LaserScan) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for target_id, (x_m, y_m) in enumerate(ANSWERS):
            color = self._color(target_id)
            self._append_circle(markers, scan, target_id, x_m, y_m, color)
            self._append_center(markers, scan, target_id, x_m, y_m, color)
            self._append_label(markers, scan, target_id, x_m, y_m, color)

        self._publisher.publish(markers)

    @staticmethod
    def _color(target_id):
        colors = [
            (1.0, 0.1, 0.1),
            (1.0, 0.5, 0.0),
            (1.0, 1.0, 0.0),
            (0.1, 1.0, 0.2),
            (0.0, 0.8, 1.0),
            (0.8, 0.2, 1.0),
        ]
        return colors[target_id]

    @staticmethod
    def _append_circle(markers, scan, target_id, x_m, y_m, color):
        circle = Marker()
        circle.header = scan.header
        circle.ns = 'exam_six_circles'
        circle.id = target_id
        circle.type = Marker.LINE_STRIP
        circle.action = Marker.ADD
        circle.pose.orientation.w = 1.0
        circle.scale.x = 0.018
        circle.color.r, circle.color.g, circle.color.b = color
        circle.color.a = 1.0
        for index in range(65):
            angle = 2.0 * math.pi * index / 64
            point = Point()
            point.x = x_m + 0.065 * math.cos(angle)
            point.y = y_m + 0.065 * math.sin(angle)
            circle.points.append(point)
        markers.markers.append(circle)

    @staticmethod
    def _append_center(markers, scan, target_id, x_m, y_m, color):
        center = Marker()
        center.header = scan.header
        center.ns = 'exam_six_centers'
        center.id = target_id
        center.type = Marker.SPHERE
        center.action = Marker.ADD
        center.pose.position.x = x_m
        center.pose.position.y = y_m
        center.pose.orientation.w = 1.0
        center.scale.x = 0.045
        center.scale.y = 0.045
        center.scale.z = 0.045
        center.color.r, center.color.g, center.color.b = color
        center.color.a = 1.0
        markers.markers.append(center)

    @staticmethod
    def _append_label(markers, scan, target_id, x_m, y_m, color):
        radius_m = math.hypot(x_m, y_m)
        label = Marker()
        label.header = scan.header
        label.ns = 'exam_six_labels'
        label.id = target_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x_m
        label.pose.position.y = y_m + 0.10
        label.pose.position.z = 0.04
        label.pose.orientation.w = 1.0
        label.scale.z = 0.055
        label.color.r, label.color.g, label.color.b = color
        label.color.a = 1.0
        label.text = (
            f'ID{target_id}\n'
            f'x={x_m:+.3f} y={y_m:+.3f}\n'
            f'r={radius_m:.3f} m'
        )
        markers.markers.append(label)


def main() -> None:
    rclpy.init()
    node = ExamAnswerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
