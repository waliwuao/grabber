"""Trigger-based connector approach node.

Two independent triggers:
  ~/trigger_prepare: align X via ares_tool_node/tool_action {prepare}
  ~/trigger_forward: open-loop chassis forward via /t0x0101_
"""

import json
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger


class ConnectorApproachNode(Node):
    """Trigger-based prepare alignment and forward approach controller."""

    def __init__(self) -> None:
        super().__init__('connector_approach_node')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('tool_service', '/ares_tool_node/tool_action')
        self.declare_parameter('chassis_topic', '/t0x0101_')
        self.declare_parameter('status_topic', '/connector_approach/status')

        self.declare_parameter('prepare_offset_m', 0.0)
        self.declare_parameter('direction_sign_x', 1.0)
        self.declare_parameter('direction_sign_y', 1.0)

        self.declare_parameter('deadband_x_m', 0.005)
        self.declare_parameter('deadband_y_m', 0.03)

        self.declare_parameter('minimum_forward_speed_mps', 0.03)
        self.declare_parameter('forward_timeout_s', 10.0)
        self.declare_parameter('prepare_timeout_ms', 20000)
        self.declare_parameter('publish_rate_hz', 100.0)

        self._state = 'idle'
        self._target_id: int | None = None
        self._last_x_m: float | None = None
        self._last_y_m: float | None = None
        self._forward_mps = 0.0
        self._forward_start_time: float | None = None
        self._forward_duration_s: float = 0.0
        self._latest_recognition: dict | None = None
        self._tool_available: bool = False

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
        self._trigger_prepare_srv = self.create_service(
            Trigger,
            '~/trigger_prepare',
            self._trigger_prepare_callback,
        )
        self._trigger_forward_srv = self.create_service(
            Trigger,
            '~/trigger_forward',
            self._trigger_forward_callback,
        )

        self._tool_client = None
        self._init_tool_client()
        self._init_prepare_done: bool = False
        self._init_timer = self.create_timer(3.0, self._try_init_prepare)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            'Connector approach starting (dx=%.4f m, dy=%.4f m); init prepare pending'
            % (
                float(self.get_parameter('deadband_x_m').value),
                float(self.get_parameter('deadband_y_m').value),
            )
        )

    def _init_tool_client(self) -> None:
        try:
            from ares_tool_interfaces.srv import ToolAction
            self._tool_client = self.create_client(
                ToolAction,
                str(self.get_parameter('tool_service').value),
            )
            self.get_logger().info('ToolAction client created')
        except Exception as exc:
            self.get_logger().warn(
                'ares_tool_interfaces not available (%s); prepare disabled'
                % exc
            )

    def _ensure_tool_available(self) -> bool:
        if self._tool_client is None:
            return False
        if self._tool_available:
            return True
        if self._tool_client.wait_for_service(timeout_sec=0.5):
            self._tool_available = True
            self.get_logger().info('Tool action service connected')
        return self._tool_available

    def _try_init_prepare(self) -> None:
        self.destroy_timer(self._init_timer)
        self._init_prepare_done = True

        if self._tool_client is None:
            self.get_logger().warn('Tool client unavailable; skip init prepare')
            return

        if not self._tool_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Tool service not reachable; skip init prepare')
            return

        self._tool_available = True
        offset = float(self.get_parameter('prepare_offset_m').value)
        self.get_logger().info(
            'Init prepare: moving connector to length=%.4f m' % offset
        )

        from ares_tool_interfaces.srv import ToolAction
        req = ToolAction.Request()
        req.action = 'prepare'
        req.args = [offset, 0.0, 0.0, 0.0]

        future = self._tool_client.call_async(req)

        def _done(fut) -> None:
            if fut.result() is not None:
                r = fut.result()
                if r.success:
                    self.get_logger().info('Init prepare completed')
                else:
                    self.get_logger().warn(
                        'Init prepare failed: ret=%d msg="%s"'
                        % (r.ret, r.message)
                    )
            else:
                self.get_logger().error('Init prepare: future resolved with no result')

        future.add_done_callback(_done)

    def _recognition_callback(self, message: String) -> None:
        try:
            self._latest_recognition = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            pass

    def _pick_target(self) -> tuple:
        targets = self._latest_recognition.get('targets', [])
        best = min(targets, key=lambda t: abs(float(t.get('x_m', 0.0))))
        tid = int(best.get('id', -1))
        x_m = float(best['x_m'])
        y_m = float(best['y_m'])
        return tid, x_m, y_m

    def _trigger_prepare_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state != 'idle':
            response.success = False
            response.message = 'busy: %s' % self._state
            return response

        if not self._latest_recognition:
            response.success = False
            response.message = 'no recognition data'
            return response

        if self._latest_recognition.get('status') != 'recognized':
            response.success = False
            response.message = 'status: %s' % self._latest_recognition.get(
                'status'
            )
            return response

        tid, x_m, y_m = self._pick_target()
        self._last_x_m = x_m
        self._last_y_m = y_m
        self._target_id = tid

        db_x = float(self.get_parameter('deadband_x_m').value)
        x_error = 0.0 - x_m  # error = desired(0) - current

        if abs(x_error) <= db_x:
            response.success = True
            response.message = (
                'X already within deadband (x=%.4f m, error=%.4f m)'
                % (x_m, x_error)
            )
            return response

        sign = float(self.get_parameter('direction_sign_x').value)
        offset = float(self.get_parameter('prepare_offset_m').value)
        length = sign * x_error + offset

        if not self._ensure_tool_available():
            response.success = False
            response.message = (
                'Tool service not available; X error=%.4f m (length=%.4f m)'
                % (x_error, length)
            )
            return response

        self._state = 'preparing'
        self.get_logger().info(
            'Trigger prepare: target=%d x=%.4f error=%.4f length=%.4f'
            % (tid, x_m, x_error, length)
        )

        from ares_tool_interfaces.srv import ToolAction
        req = ToolAction.Request()
        req.action = 'prepare'
        req.args = [float(length), 0.0, 0.0, 0.0]

        timeout_s = (
            float(self.get_parameter('prepare_timeout_ms').value) / 1000.0
        )
        future = self._tool_client.call_async(req)

        done_event = threading.Event()
        result_success = False
        result_message = ''

        def _on_done(fut) -> None:
            nonlocal result_success, result_message
            if fut.result() is not None:
                r = fut.result()
                result_success = r.success
                result_message = r.message
                if r.success:
                    self.get_logger().info('Prepare completed')
                else:
                    self.get_logger().warn(
                        'Prepare failed: ret=%d msg="%s"'
                        % (r.ret, r.message)
                    )
            else:
                result_message = 'prepare future resolved with no result'
                self.get_logger().error(result_message)
            done_event.set()

        future.add_done_callback(_on_done)

        if done_event.wait(timeout=timeout_s):
            self._state = 'idle'
            response.success = result_success
            response.message = result_message
        else:
            self._state = 'idle'
            response.success = False
            response.message = 'prepare timed out (%.1f s)' % timeout_s
            self.get_logger().error(response.message)

        return response

    def _trigger_forward_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state != 'idle':
            response.success = False
            response.message = 'busy: %s' % self._state
            return response

        if not self._latest_recognition:
            response.success = False
            response.message = 'no recognition data'
            return response

        if self._latest_recognition.get('status') != 'recognized':
            response.success = False
            response.message = 'status: %s' % self._latest_recognition.get(
                'status'
            )
            return response

        tid, x_m, y_m = self._pick_target()
        self._last_x_m = x_m
        self._last_y_m = y_m
        self._target_id = tid

        db_y = float(self.get_parameter('deadband_y_m').value)
        y_error = 0.0 - y_m  # error = desired(0) - current; Y is negative in work area

        if abs(y_error) <= db_y:
            response.success = True
            response.message = (
                'Y already within deadband (y=%.4f m, error=%.4f m)'
                % (y_m, y_error)
            )
            return response

        sign = float(self.get_parameter('direction_sign_y').value)
        min_s = float(self.get_parameter('minimum_forward_speed_mps').value)
        self._forward_duration_s = abs(y_error) / min_s
        self._forward_mps = sign * min_s if y_error > 0 else -sign * min_s
        self._forward_start_time = time.monotonic()
        self._state = 'forwarding'

        self.get_logger().info(
            'Trigger forward: target=%d y=%.4f error=%.4f speed=%.4f m/s duration=%.3f s'
            % (tid, y_m, y_error, self._forward_mps, self._forward_duration_s)
        )

        response.success = True
        response.message = 'forwarding for %.2f s' % self._forward_duration_s
        return response

    def _publish_chassis(self) -> None:
        msg = Float32MultiArray()
        if self._state == 'forwarding':
            msg.data = [self._forward_mps, 0.0, 0.0]
        else:
            msg.data = [0.0, 0.0, 0.0]
        self._chassis_pub.publish(msg)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'state': self._state,
                'target_id': self._target_id,
                'current_x_m': self._last_x_m,
                'current_y_m': self._last_y_m,
                'forward_mps': self._forward_mps,
                'forward_elapsed_s': (
                    time.monotonic() - self._forward_start_time
                    if self._forward_start_time is not None
                    and self._state == 'forwarding'
                    else 0.0
                ),
                'forward_duration_s': self._forward_duration_s,
                'deadband_x_m': float(
                    self.get_parameter('deadband_x_m').value
                ),
                'deadband_y_m': float(
                    self.get_parameter('deadband_y_m').value
                ),
                'tool_available': self._tool_available,
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)

    def _publish(self) -> None:
        now = time.monotonic()

        if self._state == 'forwarding':
            timeout_s = float(
                self.get_parameter('forward_timeout_s').value
            )
            if (
                self._forward_start_time is not None
                and now - self._forward_start_time >= self._forward_duration_s
            ):
                self.get_logger().info('Forward phase completed (duration)')
                self._forward_mps = 0.0
                self._state = 'idle'
            elif (
                self._forward_start_time is not None
                and now - self._forward_start_time >= timeout_s
            ):
                self.get_logger().warn(
                    'Forward timeout (%.1f s)' % timeout_s
                )
                self._forward_mps = 0.0
                self._state = 'idle'

        self._publish_chassis()
        self._publish_status()

    def _reset(self) -> None:
        self._state = 'idle'
        self._forward_mps = 0.0
        self._target_id = None
        self._forward_start_time = None
        self._forward_duration_s = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConnectorApproachNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        if rclpy.ok():
            node._reset()
            node._publish_chassis()
    except ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
