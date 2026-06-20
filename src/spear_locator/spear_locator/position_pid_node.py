"""ROS 2 outer-loop PID: recognized target X position -> velocity command."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .position_pid import PositionPid, PositionPidConfig


class PositionPidNode(Node):
    """Track one recognized target and command velocity along the X axis."""

    def __init__(self) -> None:
        super().__init__('spear_position_pid')
        self.declare_parameter(
            'result_topic', '/spear_recognition/result'
        )
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'speed_topic', '/spear_pid/speed_mps'
        )
        self.declare_parameter(
            'status_topic', '/spear_pid/status'
        )
        self.declare_parameter('target_id', 2)
        self.declare_parameter('desired_x_m', 0.0)
        self.declare_parameter('kp', 0.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('maximum_speed_mps', 0.01)
        self.declare_parameter('integral_limit_m_s', 0.05)
        self.declare_parameter('deadband_m', 0.005)
        self.declare_parameter('derivative_filter', 0.7)
        self.declare_parameter('direction_sign', 1.0)
        self.declare_parameter('command_hold_s', 0.5)
        self.declare_parameter('recognition_timeout_s', 4.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('enabled', False)

        self._pid = PositionPid(
            PositionPidConfig(
                kp=float(self.get_parameter('kp').value),
                ki=float(self.get_parameter('ki').value),
                kd=float(self.get_parameter('kd').value),
                maximum_speed_mps=float(
                    self.get_parameter('maximum_speed_mps').value
                ),
                integral_limit_m_s=float(
                    self.get_parameter('integral_limit_m_s').value
                ),
                deadband_m=float(
                    self.get_parameter('deadband_m').value
                ),
                derivative_filter=float(
                    self.get_parameter('derivative_filter').value
                ),
            )
        )
        self._last_measurement_time = None
        self._last_command_mps = 0.0
        self._last_status = 'waiting_for_recognition'
        self._last_x_m = None
        self._last_error_m = None

        self._twist_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self._speed_pub = self.create_publisher(
            Float64,
            str(self.get_parameter('speed_topic').value),
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self._subscription = self.create_subscription(
            String,
            str(self.get_parameter('result_topic').value),
            self._recognition_callback,
            10,
        )
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            'Position PID ready; target_id=%d desired_x=%.4f m enabled=%s'
            % (
                int(self.get_parameter('target_id').value),
                float(self.get_parameter('desired_x_m').value),
                self.get_parameter('enabled').value,
            )
        )

    def _stop(self, status: str, reset: bool = True) -> None:
        self._last_command_mps = 0.0
        self._last_status = status
        if reset:
            self._pid.reset()

    def _recognition_callback(self, message: String) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._stop('invalid_json')
            return

        if payload.get('status') != 'recognized':
            self._stop('target_set_not_recognized')
            return

        target_id = int(self.get_parameter('target_id').value)
        target = next(
            (
                item for item in payload.get('targets', [])
                if int(item.get('id', -1)) == target_id
            ),
            None,
        )
        if target is None or 'x_m' not in target:
            self._stop('target_id_missing')
            return

        current_x_m = float(target['x_m'])
        desired_x_m = float(
            self.get_parameter('desired_x_m').value
        )
        error_m = desired_x_m - current_x_m
        dt_s = 0.0
        if self._last_measurement_time is not None:
            dt_s = now - self._last_measurement_time
        command_mps = self._pid.update(error_m, dt_s)
        command_mps *= float(
            self.get_parameter('direction_sign').value
        )

        self._last_measurement_time = now
        self._last_x_m = current_x_m
        self._last_error_m = error_m
        if bool(self.get_parameter('enabled').value):
            self._last_command_mps = command_mps
            self._last_status = (
                'holding'
                if command_mps == 0.0
                else 'tracking'
            )
        else:
            self._last_command_mps = 0.0
            self._last_status = 'disabled'

    def _publish(self) -> None:
        age_s = None
        if self._last_measurement_time is not None:
            age_s = time.monotonic() - self._last_measurement_time
        timeout_s = float(
            self.get_parameter('recognition_timeout_s').value
        )
        if (
            age_s is None
            or age_s > timeout_s
        ):
            self._stop('recognition_timeout')
        elif age_s > float(
            self.get_parameter('command_hold_s').value
        ):
            self._last_command_mps = 0.0
            if self._last_status == 'tracking':
                self._last_status = 'waiting_for_next_measurement'

        twist = Twist()
        twist.linear.x = self._last_command_mps
        self._twist_pub.publish(twist)

        speed = Float64()
        speed.data = self._last_command_mps
        self._speed_pub.publish(speed)

        status = String()
        status.data = json.dumps(
            {
                'status': self._last_status,
                'enabled': bool(
                    self.get_parameter('enabled').value
                ),
                'target_id': int(
                    self.get_parameter('target_id').value
                ),
                'desired_x_m': float(
                    self.get_parameter('desired_x_m').value
                ),
                'current_x_m': self._last_x_m,
                'error_x_m': self._last_error_m,
                'speed_mps': self._last_command_mps,
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node._stop('shutdown')
            node._publish()
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
