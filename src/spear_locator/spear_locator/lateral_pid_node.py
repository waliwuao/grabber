"""Trigger-based open-loop two-phase approach node.

Subscribes to spear recognition results, waits for a trigger service,
picks the target whose X is closest to 0. Phase 1 moves laterally at
minimum speed for the computed duration, phase 2 moves forward likewise.
"""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger


class LateralPidNode(Node):
    """Trigger-based open-loop two-phase approach controller."""

    def __init__(self) -> None:
        super().__init__('lateral_pid')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('chassis_topic', '/t0x0101_')
        self.declare_parameter('status_topic', '/lateral_pid/status')

        self.declare_parameter('minimum_speed_mps', 0.03)
        self.declare_parameter('maximum_speed_mps', 0.1)
        self.declare_parameter('deadband_x_m', 0.005)
        self.declare_parameter('deadband_y_m', 0.1)

        self.declare_parameter('direction_sign_x', 1.0)
        self.declare_parameter('direction_sign_y', 1.0)
        self.declare_parameter('approach_timeout_s', 5.0)
        self.declare_parameter('publish_rate_hz', 100.0)

        self._state = 'idle'
        self._phase = 'align_x'
        self._target_id: int | None = None
        self._last_x_m: float | None = None
        self._last_y_m: float | None = None
        self._last_forward_mps = 0.0
        self._last_lateral_mps = 0.0
        self._phase_start_time: float | None = None
        self._phase_duration_s: float = 0.0
        self._approach_start_time: float | None = None
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
            'Open-loop two-phase ready (dx=%.4f m, dy=%.4f m, timeout=%.1f s)'
            % (
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
                'phase': self._phase,
                'target_id': self._target_id,
                'current_x_m': self._last_x_m,
                'current_y_m': self._last_y_m,
                'forward_mps': self._last_forward_mps,
                'lateral_mps': self._last_lateral_mps,
                'deadband_x_m': float(
                    self.get_parameter('deadband_x_m').value
                ),
                'deadband_y_m': float(
                    self.get_parameter('deadband_y_m').value
                ),
                'phase_elapsed_s': (
                    time.monotonic() - self._phase_start_time
                    if self._phase_start_time is not None
                    else 0.0
                ),
                'phase_duration_s': self._phase_duration_s,
                'total_elapsed_s': (
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
        if self._state != 'approaching':
            msg.data = [0.0, 0.0, 0.0]
        else:
            sx = float(self.get_parameter('direction_sign_x').value)
            sy = float(self.get_parameter('direction_sign_y').value)
            fwd = self._last_forward_mps if self._phase == 'approach_y' else 0.0
            lat = self._last_lateral_mps if self._phase == 'align_x' else 0.0
            msg.data = [sy * fwd, sx * lat, 0.0]
        self._chassis_pub.publish(msg)

    def _reset(self) -> None:
        self._state = 'idle'
        self._phase = 'align_x'
        self._last_forward_mps = 0.0
        self._last_lateral_mps = 0.0
        self._target_id = None
        self._phase_start_time = None
        self._phase_duration_s = 0.0
        self._approach_start_time = None

    def _recognition_callback(self, message: String) -> None:
        try:
            self._latest_recognition = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            pass

    def _start_phase(self, phase: str, error_m: float) -> None:
        min_s = float(self.get_parameter('minimum_speed_mps').value)
        self._phase = phase
        self._phase_start_time = time.monotonic()
        self._phase_duration_s = abs(error_m) / min_s
        if phase == 'align_x':
            self._last_lateral_mps = min_s if error_m > 0 else -min_s
            self._last_forward_mps = 0.0
        else:
            self._last_lateral_mps = 0.0
            self._last_forward_mps = min_s if error_m > 0 else -min_s
        self.get_logger().info(
            'Phase %s: error=%.4f m speed=%.4f m/s duration=%.3f s'
            % (phase, error_m, min_s, self._phase_duration_s)
        )

    def _trigger_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state != 'idle':
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

        db_x = float(self.get_parameter('deadband_x_m').value)
        db_y = float(self.get_parameter('deadband_y_m').value)

        self._last_x_m = dx_m
        self._last_y_m = dy_m

        if abs(dx_m) <= db_x and abs(dy_m) <= db_y:
            self._reset()
            self.get_logger().info(
                'Target %d already in position (x=%.4f, |y|=%.4f)'
                % (self._target_id, dx_m, abs(dy_m))
            )
            response.success = True
            response.message = 'already in position'
            return response

        self._state = 'approaching'
        self._approach_start_time = time.monotonic()

        if abs(dx_m) > db_x:
            self._start_phase('align_x', 0.0 - dx_m)
        else:
            self.get_logger().info('X already aligned, skipping to Y')
            self._start_phase('approach_y', 0.0 - dy_m)

        response.success = True
        response.message = (
            'phase %s target %d (x=%.4f, |y|=%.4f)'
            % (self._phase, self._target_id, dx_m, abs(dy_m))
        )
        return response

    def _publish(self) -> None:
        now = time.monotonic()

        if self._state == 'approaching':
            total_timeout = float(self.get_parameter('approach_timeout_s').value)
            if (
                self._approach_start_time is not None
                and now - self._approach_start_time >= total_timeout
            ):
                self.get_logger().info(
                    'Total timeout (%.1f s), stopping'
                    % (now - self._approach_start_time)
                )
                self._reset()

            elif (
                self._phase_start_time is not None
                and now - self._phase_start_time >= self._phase_duration_s
            ):
                if self._phase == 'align_x':
                    self._last_lateral_mps = 0.0
                    dy = self._last_y_m
                    error_y = 0.0 - (dy if dy is not None else 0.0)
                    db_y = float(self.get_parameter('deadband_y_m').value)
                    if abs(error_y) > db_y:
                        self._start_phase('approach_y', error_y)
                    else:
                        self.get_logger().info('Y already in deadband, done')
                        self._state = 'success'
                        self._last_forward_mps = 0.0
                else:
                    self._last_forward_mps = 0.0
                    self.get_logger().info('Both phases completed')
                    self._state = 'success'

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
