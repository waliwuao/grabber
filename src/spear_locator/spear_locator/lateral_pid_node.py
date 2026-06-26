"""Trigger-based approach control node.

Subscribes to spear recognition results, waits for a trigger service,
picks the target whose X is closest to 0, and drives both lateral (X→0)
and forward (|Y|→<0.1) axes for up to approach_timeout_s seconds.
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
    """Trigger-based X/Y approach controller for spear target."""

    def __init__(self) -> None:
        super().__init__('lateral_pid')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('chassis_topic', '/t0x0101_')
        self.declare_parameter('status_topic', '/lateral_pid/status')

        self.declare_parameter('kp', 0.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('maximum_speed_mps', 0.1)
        self.declare_parameter('minimum_speed_mps', 0.03)
        self.declare_parameter('integral_limit_m_s', 0.05)
        self.declare_parameter('deadband_x_m', 0.005)
        self.declare_parameter('deadband_y_m', 0.1)
        self.declare_parameter('derivative_filter', 0.7)

        self.declare_parameter('direction_sign_x', 1.0)
        self.declare_parameter('direction_sign_y', 1.0)
        self.declare_parameter('approach_timeout_s', 5.0)
        self.declare_parameter('publish_rate_hz', 100.0)

        def _make_config(deadband_m: float) -> PositionPidConfig:
            return PositionPidConfig(
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
                deadband_m=deadband_m,
                derivative_filter=float(
                    self.get_parameter('derivative_filter').value
                ),
            )

        self._pid_x = PositionPid(
            _make_config(float(self.get_parameter('deadband_x_m').value))
        )
        self._pid_y = PositionPid(
            _make_config(float(self.get_parameter('deadband_y_m').value))
        )

        self._state = 'idle'
        self._target_id: int | None = None
        self._last_x_m: float | None = None
        self._last_y_m: float | None = None
        self._last_error_x_m: float | None = None
        self._last_error_y_m: float | None = None
        self._last_forward_mps = 0.0
        self._last_lateral_mps = 0.0
        self._approach_start_time: float | None = None
        self._last_measurement_time: float | None = None
        self._latest_recognition: dict | None = None

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
            'Approach PID ready (state=%s, dx=%.4f m, dy=%.4f m, timeout=%.1f s)'
            % (
                self._state,
                float(self.get_parameter('deadband_x_m').value),
                float(self.get_parameter('deadband_y_m').value),
                float(self.get_parameter('approach_timeout_s').value),
            )
        )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'state': self._state,
                'target_id': self._target_id,
                'current_x_m': self._last_x_m,
                'current_y_m': self._last_y_m,
                'error_x_m': self._last_error_x_m,
                'error_y_m': self._last_error_y_m,
                'forward_mps': self._last_forward_mps,
                'lateral_mps': self._last_lateral_mps,
                'deadband_x_m': float(
                    self.get_parameter('deadband_x_m').value
                ),
                'deadband_y_m': float(
                    self.get_parameter('deadband_y_m').value
                ),
                'elapsed_s': (
                    time.monotonic() - self._approach_start_time
                    if self._approach_start_time is not None
                    else 0.0
                ),
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)

    def _publish_chassis(self) -> None:
        msg = Float32MultiArray()
        sx = float(self.get_parameter('direction_sign_x').value)
        sy = float(self.get_parameter('direction_sign_y').value)
        msg.data = [sy * self._last_forward_mps, sx * self._last_lateral_mps, 0.0]
        self._chassis_pub.publish(msg)

    def _reset(self) -> None:
        self._state = 'idle'
        self._last_forward_mps = 0.0
        self._last_lateral_mps = 0.0
        self._target_id = None
        self._last_error_x_m = None
        self._last_error_y_m = None
        self._approach_start_time = None
        self._last_measurement_time = None
        self._pid_x.reset()
        self._pid_y.reset()

    def _recognition_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        self._latest_recognition = payload

        if self._state != 'approaching':
            return

        if payload.get('status') != 'recognized':
            self.get_logger().warn('Recognition lost during approach, stopping')
            self._reset()
            return

        target = next(
            (
                item for item in payload.get('targets', [])
                if int(item.get('id', -1)) == self._target_id
            ),
            None,
        )
        if target is None or 'x_m' not in target or 'y_m' not in target:
            self.get_logger().warn(
                'Target %d disappeared during approach, stopping' % self._target_id
            )
            self._reset()
            return

        now = time.monotonic()
        dx_m = float(target['x_m'])
        dy_m = float(target['y_m'])
        error_x = 0.0 - dx_m
        error_y = 0.0 - dy_m

        dt_s = 0.0
        if self._last_measurement_time is not None:
            dt_s = now - self._last_measurement_time
        self._last_measurement_time = now

        forward = self._pid_y.update(error_y, dt_s)
        lateral = self._pid_x.update(error_x, dt_s)

        self._last_x_m = dx_m
        self._last_y_m = dy_m
        self._last_error_x_m = error_x
        self._last_error_y_m = error_y
        self._last_forward_mps = forward
        self._last_lateral_mps = lateral

        if forward == 0.0 and lateral == 0.0:
            self.get_logger().info(
                'Target %d reached (x=%.4f m, y=%.4f m)'
                % (self._target_id, dx_m, dy_m)
            )
            self._state = 'success'

    def _trigger_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state in ('approaching', 'success'):
            response.success = False
            response.message = 'busy: %s' % self._state
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
        dx_m = float(best['x_m'])
        dy_m = float(best['y_m'])
        error_x = 0.0 - dx_m
        error_y = 0.0 - dy_m

        db_x = float(self.get_parameter('deadband_x_m').value)
        db_y = float(self.get_parameter('deadband_y_m').value)

        if abs(dx_m) <= db_x and abs(dy_m) <= db_y:
            self._reset()
            self._last_x_m = dx_m
            self._last_y_m = dy_m
            self._last_error_x_m = error_x
            self._last_error_y_m = error_y
            self.get_logger().info(
                'Target %d already in position (x=%.4f, |y|=%.4f)'
                % (self._target_id, dx_m, abs(dy_m))
            )
            response.success = True
            response.message = (
                'target %d already at x=%.4f m, |y|=%.4f m (within deadbands)'
                % (self._target_id, dx_m, abs(dy_m))
            )
            return response

        self._pid_x.reset()
        self._pid_y.reset()
        forward = self._pid_y.update(error_y, 0.0)
        lateral = self._pid_x.update(error_x, 0.0)

        self._state = 'approaching'
        self._last_x_m = dx_m
        self._last_y_m = dy_m
        self._last_error_x_m = error_x
        self._last_error_y_m = error_y
        self._last_forward_mps = forward
        self._last_lateral_mps = lateral
        self._approach_start_time = time.monotonic()
        self._last_measurement_time = time.monotonic()
        timeout_s = float(self.get_parameter('approach_timeout_s').value)

        self.get_logger().info(
            'Trigger: target %d x=%.4f y=%.4f |y|=%.4f'
            ' fwd=%.4f lat=%.4f m/s timeout=%.1f s'
            % (
                self._target_id, dx_m, dy_m, abs(dy_m),
                forward, lateral, timeout_s,
            )
        )
        response.success = True
        response.message = (
            'approaching target %d (x=%.4f, |y|=%.4f, fwd=%.4f, lat=%.4f m/s)'
            % (self._target_id, dx_m, abs(dy_m), forward, lateral)
        )
        return response

    def _publish(self) -> None:
        now = time.monotonic()

        if self._state == 'approaching':
            timeout_s = float(self.get_parameter('approach_timeout_s').value)
            if (
                self._approach_start_time is not None
                and now - self._approach_start_time >= timeout_s
            ):
                self.get_logger().info(
                    'Approach timeout (%.1f s), stopping'
                    % (now - self._approach_start_time)
                )
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
