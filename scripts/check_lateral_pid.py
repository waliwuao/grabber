#!/usr/bin/env python3
"""Check recognition data and preview the approach pulse without executing it.

Subscribes to the recognition topic, waits for a valid result, prints each
target position, picks the one closest to X=0, and shows what forward/lateral
velocity would be sent on a trigger.
"""

import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def compute_speed(
    error_m: float,
    kp: float,
    deadband: float,
    min_s: float,
    max_s: float,
) -> float:
    if abs(error_m) <= deadband:
        return 0.0
    raw = kp * error_m
    output = clamp(raw, -max_s, max_s)
    if output != 0.0 and abs(output) < min_s:
        output = min_s if output > 0.0 else -min_s
    return output


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
        self.declare_parameter('minimum_speed_mps', 0.03)
        self.declare_parameter('deadband_x_m', 0.005)
        self.declare_parameter('deadband_y_m', 0.1)
        self.declare_parameter('direction_sign_x', 1.0)
        self.declare_parameter('direction_sign_y', 1.0)
        self.declare_parameter('approach_timeout_s', 3.0)

    def _callback(self, msg: String) -> None:
        try:
            self._result = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            pass


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

            db_x = float(node.get_parameter('deadband_x_m').value)
            db_y = float(node.get_parameter('deadband_y_m').value)
            kp = float(node.get_parameter('kp').value)
            min_s = float(node.get_parameter('minimum_speed_mps').value)
            max_s = float(node.get_parameter('maximum_speed_mps').value)
            sx = float(node.get_parameter('direction_sign_x').value)
            sy = float(node.get_parameter('direction_sign_y').value)
            timeout_s_val = float(node.get_parameter('approach_timeout_s').value)

            targets_sorted = sorted(
                targets, key=lambda t: abs(float(t.get('x_m', 0.0)))
            )

            print('=== Recognition Targets ===')
            print('%-4s %10s %10s %10s %10s' % ('ID', 'X (m)', 'Y (m)', '|X| (m)', '|Y| (m)'))
            for t in payload['targets']:
                tid = int(t.get('id', -1))
                x = float(t.get('x_m', 0.0))
                y = float(t.get('y_m', 0.0))
                closest = int(targets_sorted[0].get('id', -1))
                marker = ' <--' if tid == closest else ''
                print(
                    '%-4d %10.4f %10.4f %10.4f %10.4f%s'
                    % (tid, x, y, abs(x), abs(y), marker)
                )

            best = targets_sorted[0]
            target_id = int(best.get('id', -1))
            dx = float(best['x_m'])
            dy = float(best['y_m'])
            error_x = -dx
            error_y = -dy

            print()
            print('=== Simulated Trigger ===')
            print('target_id          : %d' % target_id)
            print('target_x_m         : %.4f' % dx)
            print('target_y_m         : %.4f' % dy)
            print('|target_y|_m       : %.4f' % abs(dy))
            print('deadband_x_m       : %.4f' % db_x)
            print('deadband_y_m       : %.4f' % db_y)
            print('approach_timeout_s : %.1f' % timeout_s_val)
            print()

            raw_x = compute_speed(error_x, kp, db_x, min_s, max_s)
            raw_y = compute_speed(error_y, kp, db_y, min_s, max_s)
            lateral = sx * raw_x
            forward = sy * raw_y

            print('pid_kp*error_x     : %.4f m/s' % (kp * error_x))
            print('pid_kp*error_y     : %.4f m/s' % (kp * error_y))
            print('forward_mps        : %.4f' % forward)
            print('lateral_mps        : %.4f' % lateral)
            print('limits             : [%.4f, %.4f] m/s' % (min_s, max_s))
            print()

            if raw_x == 0.0 and raw_y == 0.0:
                print('RESULT: already in position, no movement needed')
            else:
                dist_fwd = abs(forward) * timeout_s_val
                dist_lat = abs(lateral) * timeout_s_val
                print(
                    'RESULT: would send [%.4f, %.4f, 0.0] on /t0x0101_ for up to %.1f s'
                    % (forward, lateral, timeout_s_val)
                )
                print(
                    '        max forward displacement: ≈ %.1f mm, max lateral: ≈ %.1f mm'
                    % (dist_fwd * 1000.0, dist_lat * 1000.0)
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
