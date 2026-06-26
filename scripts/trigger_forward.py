#!/usr/bin/env python3
"""Trigger chassis forward approach."""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('trigger_forward_client')
    cli = node.create_client(
        Trigger, '/connector_approach_node/trigger_forward'
    )

    if not cli.wait_for_service(timeout_sec=3.0):
        print('ERROR: trigger_forward service not available', file=sys.stderr)
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
