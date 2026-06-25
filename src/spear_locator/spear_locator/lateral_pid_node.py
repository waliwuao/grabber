"""Trigger-based lateral one-shot PID control node.

Subscribes to spear recognition results, waits for a trigger service,
picks the target whose X is closest to 0, runs one PID update and issues
a single velocity pulse of configurable duration (command_hold_s).
"""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from .position_pid import PositionPid, PositionPidConfig


class LateralPidNode(Node):
    """Trigger-based one-shot lateral PID controller for spear centering."""

    def __init__(self) -> None:
        super().__init__('lateral_pid')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('chassis_topic', '/t0x0101_')
        self.declare_parameter('status_topic', '/lateral_pid/status')

        self.declare_parameter('kp', 0.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('maximum_speed_mps', 0.1)
        self.declare_parameter('minimum_speed_mps', 0.02)
        self.declare_parameter('integral_limit_m_s', 0.05)
        self.declare_parameter('deadband_m', 0.005)
        self.declare_parameter('derivative_filter', 0.7)

        self.declare_parameter('direction_sign', 1.0)
        self.declare_parameter('command_hold_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._pid = PositionPid(
            PositionPidConfig(
                kp=float(self.get_parameter('kp').value),
                ki=float(self.get_parameter('ki').value),
                kd=float(self.get_parameter('kd').value),
                maximum_speed_mps=float(
                    self.get_parameter('maximum_speed_mps').value
                ),
                minimum_speed_mps=float(
                    self.get_parameter('minimum_speed_mps').value
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

        self._state = 'idle'
        self._target_id = None
        self._last_x_m = None
        self._last_error_m = None
        self._last_command_mps = 0.0
        self._pulse_start_time = None
        self._latest_recognition = None

        self._chassis_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('chassis_topic').value),
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
        self._trigger_srv = self.create_service(
            Trigger,
            '~/trigger',
            self._trigger_callback,
        )

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            'Lateral PID ready (state=%s, deadband=%.4f m, hold=%.2f s)'
            % (
                self._state,
                float(self.get_parameter('deadband_m').value),
                float(self.get_parameter('command_hold_s').value),
            )
        )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'state': self._state,
                'target_id': self._target_id,
                'current_x_m': self._last_x_m,
                'error_x_m': self._last_error_m,
                'speed_mps': self._last_command_mps,
                'deadband_m': float(
                    self.get_parameter('deadband_m').value
                ),
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)

    def _publish_chassis(self) -> None:
        msg = Float32MultiArray()
        sign = float(self.get_parameter('direction_sign').value)
        msg.data = [0.0, sign * self._last_command_mps, 0.0]
        self._chassis_pub.publish(msg)

    def _reset(self) -> None:
        self._state = 'idle'
        self._last_command_mps = 0.0
        self._target_id = None
        self._last_error_m = None
        self._pulse_start_time = None
        self._pid.reset()

    def _recognition_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        self._latest_recognition = payload

    def _trigger_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state == 'pulsing':
            response.success = False
            response.message = 'busy: pulse in progress'
            return response

        if self._latest_recognition is None:
            response.success = False
            response.message = 'no recognition data available yet'
            return response

        if self._latest_recognition.get('status') != 'recognized':
            response.success = False
            response.message = (
                'recognition status is %s'
                % self._latest_recognition.get('status', 'unknown')
            )
            return response

        targets = self._latest_recognition.get('targets', [])
        if not targets:
            response.success = False
            response.message = 'no targets in recognition result'
            return response

        best = min(targets, key=lambda t: abs(float(t.get('x_m', 0.0))))
        self._target_id = int(best.get('id', -1))
        target_x = float(best['x_m'])
        error_m = 0.0 - target_x

        deadband = float(self.get_parameter('deadband_m').value)
        if abs(target_x) <= deadband:
            self._reset()
            self._last_x_m = target_x
            self._last_error_m = error_m
            self.get_logger().info(
                'Target %d already centred (x=%.4f m, deadband %.4f m)'
                % (self._target_id, target_x, deadband)
            )
            response.success = True
            response.message = (
                'target %d already at x=%.4f m (within deadband)'
                % (self._target_id, target_x)
            )
            return response

        self._pid.reset()
        command_mps = self._pid.update(error_m, 0.0)

        self._state = 'pulsing'
        self._last_x_m = target_x
        self._last_error_m = error_m
        self._last_command_mps = command_mps
        self._pulse_start_time = time.monotonic()
        hold_s = float(self.get_parameter('command_hold_s').value)

        self.get_logger().info(
            'Trigger: target %d x=%.4f m error=%.4f m speed=%.4f m/s hold=%.2f s'
            % (self._target_id, target_x, error_m, command_mps, hold_s)
        )
        response.success = True
        response.message = (
            'pulse target %d (x=%.4f m, speed=%.4f m/s, hold=%.2f s)'
            % (self._target_id, target_x, command_mps, hold_s)
        )
        return response

    def _publish(self) -> None:
        now = time.monotonic()

        if self._state == 'pulsing':
            hold_s = float(self.get_parameter('command_hold_s').value)
            if (
                self._pulse_start_time is not None
                and now - self._pulse_start_time >= hold_s
            ):
                self.get_logger().info('Pulse finished, returning to idle')
                self._reset()

        self._publish_chassis()
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LateralPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node._reset()
            node._publish_chassis()
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
