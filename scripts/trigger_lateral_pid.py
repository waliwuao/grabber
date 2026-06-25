#!/usr/bin/env python3
"""Send a single trigger to the lateral_pid node and print the response."""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('trigger_lateral_pid_client')
    cli = node.create_client(Trigger, '/lateral_pid/trigger')

    if not cli.wait_for_service(timeout_sec=3.0):
        node.get_logger().error('Lateral PID service not available')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    req = Trigger.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        resp = future.result()
        node.get_logger().info(
            'success=%s  message="%s"' % (resp.success, resp.message)
        )
        if not resp.success:
            sys.exit(1)
    else:
        node.get_logger().error('Service call failed')
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
