"""Trigger-based lateral PID control node.

Subscribes to spear recognition results, waits for a trigger service,
picks the target whose X is closest to 0, and uses PID feedback to
drive the chassis laterally until that target's X reaches the deadband.
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
    """Trigger-based lateral PID controller for spear target centering."""

    def __init__(self) -> None:
        super().__init__('lateral_pid')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('chassis_topic', '/t0x0101_')
        self.declare_parameter('status_topic', '/lateral_pid/status')

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

        self._state = 'idle'
        self._target_id = None
        self._last_x_m = None
        self._last_error_m = None
        self._last_command_mps = 0.0
        self._last_measurement_time = None
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
            'Lateral PID ready (state=%s, deadband=%.4f m)'
            % (self._state, float(self.get_parameter('deadband_m').value))
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

    def _stop(self, state: str) -> None:
        self._state = state
        self._last_command_mps = 0.0
        self._target_id = None
        self._last_error_m = None
        self._pid.reset()
        self.get_logger().info('Lateral PID stopped (state=%s)' % state)

    def _recognition_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        self._latest_recognition = payload

        if self._state != 'executing':
            return

        now = time.monotonic()
        if payload.get('status') != 'recognized':
            self._stop('idle')
            return

        target = next(
            (
                item for item in payload.get('targets', [])
                if int(item.get('id', -1)) == self._target_id
            ),
            None,
        )
        if target is None or 'x_m' not in target:
            self.get_logger().warn(
                'Target %d disappeared, aborting' % self._target_id
            )
            self._stop('idle')
            return

        current_x_m = float(target['x_m'])
        error_m = 0.0 - current_x_m
        dt_s = 0.0
        if self._last_measurement_time is not None:
            dt_s = now - self._last_measurement_time

        command_mps = self._pid.update(error_m, dt_s)
        deadband = float(self.get_parameter('deadband_m').value)

        self._last_measurement_time = now
        self._last_x_m = current_x_m
        self._last_error_m = error_m

        if abs(current_x_m) <= deadband:
            self._last_command_mps = 0.0
            self._state = 'success'
            self._pid.reset()
            self.get_logger().info(
                'Target %d centred (x=%.4f m, within deadband %.4f m)'
                % (self._target_id, current_x_m, deadband)
            )
        else:
            self._last_command_mps = command_mps
            self.get_logger().debug(
                'Target %d x=%.4f m error=%.4f m speed=%.4f m/s'
                % (self._target_id, current_x_m, error_m, command_mps)
            )

    def _trigger_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state == 'executing':
            response.success = False
            response.message = 'busy: already executing a plan'
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

        deadband = float(self.get_parameter('deadband_m').value)
        if abs(target_x) <= deadband:
            self._state = 'success'
            self._last_x_m = target_x
            self._last_error_m = -target_x
            self._last_command_mps = 0.0
            self.get_logger().info(
                'Target %d already centred (x=%.4f m within deadband %.4f m)'
                % (self._target_id, target_x, deadband)
            )
            response.success = True
            response.message = (
                'target %d already at x=%.4f m (within deadband)'
                % (self._target_id, target_x)
            )
            return response

        self._pid.reset()
        self._state = 'executing'
        self._last_x_m = target_x
        self._last_error_m = -target_x
        self._last_measurement_time = time.monotonic()
        self.get_logger().info(
            'Trigger accepted: tracking target %d (x=%.4f m) toward x=0'
            % (self._target_id, target_x)
        )
        response.success = True
        response.message = (
            'tracking target %d (x=%.4f m)'
            % (self._target_id, target_x)
        )
        return response

    def _publish(self) -> None:
        now = time.monotonic()

        if self._state == 'executing':
            age_s = None
            if self._last_measurement_time is not None:
                age_s = now - self._last_measurement_time
            timeout_s = float(
                self.get_parameter('recognition_timeout_s').value
            )
            if age_s is not None and age_s > timeout_s:
                self.get_logger().warn(
                    'Recognition timeout (%.1f s), stopping' % age_s
                )
                self._stop('idle')
            elif (
                age_s is not None
                and age_s > float(
                    self.get_parameter('command_hold_s').value
                )
            ):
                self._last_command_mps = 0.0
                self._publish_chassis()
                self._publish_status()
                return

        self._publish_chassis()
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LateralPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node._stop('idle')
            node._publish_chassis()
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
