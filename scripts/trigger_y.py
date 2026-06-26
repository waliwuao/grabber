#!/usr/bin/env python3
"""Trigger Y approach (forward only)."""

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('trigger_y_client')
    cli = node.create_client(Trigger, '/lateral_pid/trigger_y')

    if not cli.wait_for_service(timeout_sec=3.0):
        print('ERROR: trigger_y service not available', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    future = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        resp = future.result()
        print('success=%s  message="%s"' % (resp.success, resp.message))
        if not resp.success:
            sys.exit(1)
    else:
        print('ERROR: service call failed', file=sys.stderr)
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
