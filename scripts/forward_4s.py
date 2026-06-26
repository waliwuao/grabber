#!/usr/bin/env python3
"""Send a fixed forward pulse: 0.1 m/s for 4s on /t0x0101_."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('forward_4s')
    pub = node.create_publisher(Float32MultiArray, '/t0x0101_', 10)
    time.sleep(0.1)

    speed = 0.1
    duration = 4.0
    print('Forward: %.1f m/s for %.0f s' % (speed, duration))

    msg = Float32MultiArray()
    msg.data = [speed, 0.0, 0.0]

    start = time.monotonic()
    while time.monotonic() - start < duration:
        pub.publish(msg)
        time.sleep(0.01)

    msg.data = [0.0, 0.0, 0.0]
    pub.publish(msg)
    print('Done')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
