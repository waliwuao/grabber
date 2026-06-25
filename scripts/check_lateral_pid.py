#!/usr/bin/env python3
"""Check recognition data and preview the lateral PID pulse without executing it.

Subscribes to the recognition topic, waits for a valid result, prints each
target position, picks the one closest to X=0, and shows what velocity
would be sent on a trigger.
"""

import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class CheckNode(Node):
    def __init__(self) -> None:
        super().__init__('check_lateral_pid')
        self._result = None

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self._sub = self.create_subscription(
            String,
            str(self.get_parameter('result_topic').value),
            self._callback,
            10,
        )

        self.declare_parameter('kp', 0.2)
        self.declare_parameter('maximum_speed_mps', 0.1)
        self.declare_parameter('minimum_speed_mps', 0.02)
        self.declare_parameter('deadband_m', 0.005)
        self.declare_parameter('direction_sign', 1.0)
        self.declare_parameter('command_hold_s', 0.5)

    def _callback(self, msg: String) -> None:
        try:
            self._result = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            pass

    def compute_speed(self, error_m: float) -> float:
        deadband = float(self.get_parameter('deadband_m').value)
        if abs(error_m) <= deadband:
            return 0.0
        kp = float(self.get_parameter('kp').value)
        min_s = float(self.get_parameter('minimum_speed_mps').value)
        max_s = float(self.get_parameter('maximum_speed_mps').value)
        raw = kp * error_m
        output = clamp(raw, -max_s, max_s)
        if output != 0.0 and abs(output) < min_s:
            output = min_s if output > 0.0 else -min_s
        return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CheckNode()

    timeout_s = 5.0
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if node._result is not None:
            payload = node._result
            status = payload.get('status', 'unknown')
            if status != 'recognized':
                print(
                    'ERROR: recognition status is "%s" (expected "recognized")'
                    % status,
                    file=sys.stderr,
                )
                sys.exit(1)

            targets = payload.get('targets', [])
            if not targets:
                print('ERROR: no targets in recognition result', file=sys.stderr)
                sys.exit(1)

            deadband = float(node.get_parameter('deadband_m').value)
            hold_s = float(node.get_parameter('command_hold_s').value)
            sign = float(node.get_parameter('direction_sign').value)

            targets_sorted = sorted(
                targets, key=lambda t: abs(float(t.get('x_m', 0.0)))
            )

            print('=== Recognition Targets ===')
            print('%-4s %10s %10s %10s' % ('ID', 'X (m)', 'Y (m)', '|X| (m)'))
            for t in payload['targets']:
                tid = int(t.get('id', -1))
                x = float(t.get('x_m', 0.0))
                y = float(t.get('y_m', 0.0))
                closest = int(targets_sorted[0].get('id', -1))
                marker = ' <--' if tid == closest else ''
                print('%-4d %10.4f %10.4f %10.4f%s' % (tid, x, y, abs(x), marker))

            best = targets_sorted[0]
            target_id = int(best.get('id', -1))
            target_x = float(best['x_m'])
            error_m = -target_x

            print()
            print('=== Simulated Trigger ===')
            print('target_id       : %d' % target_id)
            print('target_x_m      : %.4f' % target_x)
            print('error_x_m       : %.4f' % error_m)
            print('deadband_m      : %.4f' % deadband)
            print('command_hold_s  : %.2f' % hold_s)

            if abs(target_x) <= deadband:
                print()
                print('RESULT: already centred (within deadband), no pulse needed')
            else:
                kp = float(node.get_parameter('kp').value)
                min_s = float(node.get_parameter('minimum_speed_mps').value)
                max_s = float(node.get_parameter('maximum_speed_mps').value)
                raw = node.compute_speed(error_m)
                lateral = sign * raw
                dist = lateral * hold_s
                print('pid_kp*error    : %.4f m/s' % (kp * error_m))
                print('pid_clamped     : %.4f m/s' % raw)
                print('lateral_cmd     : %.4f m/s' % lateral)
                print('limits          : [%.4f, %.4f] m/s' % (min_s, max_s))
                print()
                print(
                    'RESULT: would send [0.0, %.4f, 0.0] on /t0x0101_ for %.2f s (≈ %.1f mm)'
                    % (lateral, hold_s, abs(dist) * 1000.0)
                )
            sys.exit(0)

        if node.get_clock().now() > deadline:
            print(
                'ERROR: no recognition data received within %.0f s' % timeout_s,
                file=sys.stderr,
            )
            sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
