#!/usr/bin/env python
# run.py
"""Entrypoint for 'follow_trajectory2'.

Shared for both ROS versions.
"""

from autopsy.core import Core
import sys

from follow_trajectory2._run import RunNode


def main(args = None):
    """Start a ROS node, register the callbacks."""
    if args is None:
        args = sys.argv

    Core.init(args = args)

    node = RunNode()

    Core.spin(node)
    Core.shutdown()


if __name__ == '__main__':
    main()
