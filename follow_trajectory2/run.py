#!/usr/bin/env python
# run.py
"""ROS2 Entrypoint for 'follow_trajectory2'.
"""

import rclpy
import sys

from follow_trajectory2.module._run import RunNode


def main(args = None):
    """Starts a ROS node, registers the callbacks."""

    if args is None:
        args = sys.argv

    rclpy.init(args = args)

    node = RunNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
