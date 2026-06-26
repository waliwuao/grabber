#!/usr/bin/env python3
"""Test gripper control: call prepare with a given length (default 0.1 m)."""

import sys
import rclpy
from rclpy.node import Node


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('test_prepare')

    from ares_tool_interfaces.srv import ToolAction
    cli = node.create_client(ToolAction, '/ares_tool_node/tool_action')

    if not cli.wait_for_service(timeout_sec=5.0):
        print('ERROR: ares_tool_node/tool_action not available', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    length = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    req = ToolAction.Request()
    req.action = 'prepare'
    req.args = [length, 0.0, 0.0, 0.0]

    print('Sending prepare(length=%.3f m) ...' % length)
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

    if future.done() and future.result() is not None:
        r = future.result()
        print('success=%s  ret=%d  message="%s"' % (r.success, r.ret, r.message))
    else:
        print('ERROR: timed out after 30s', file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
