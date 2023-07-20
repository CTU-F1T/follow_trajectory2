#!/usr/bin/env python
# controllerabc.py
"""Controller interface for implementing trajectory tracking methods.
"""
######################
# Imports & Globals
######################

import math

from follow_trajectory2.module._utils import *


# Messages
from geometry_msgs.msg import Point


######################
# ControllerABC class
######################

class ControllerABC(object):

    def __init__(self, vehicle, *args, **kwargs):
        super(ControllerABC, self).__init__()

        self.Vehicle = vehicle


    def process_trajectory(self, controller, trajectory):
        """Prepare additional data for the controller.

        Called upon receiving a new trajectory.

        Arguments:
        controller -- controller instance, call with self
        trajectory -- trajectory data, Trajectory

        Returns:
        -
        """
        pass


    def select_point(self, controller, trajectory):
        """Select control point from the trajectory.

        Arguments:
        controller -- controller instance, call with self
        trajectory -- trajectory data, Trajectory

        Returns:
        i -- index of the closest point on the Trajectory, float/int
        point -- point on the Trajectory, TrajectoryPoint

        Note: `point` might not be in Trajectory, just approximated
        Note: Generally, when approximated, the `i` is float, describing
        the point's position.
        """
        raise NotImplementedError


    def compute(self, controller, point, trajectory, trajectory_i):
        """Compute action values using selected point.

        Arguments:
        controller -- controller instance, call with self
        point -- point of the trajectory, TrajectoryPoint
        trajectory -- trajectory data, Trajectory
        trajectory_i -- (approximate) index of the closest point to the `point`, float/int

        Returns:
        velocity -- action value for velocity, [m.s^-1], float
        steering -- action value for steering, [rad], float
        """
        raise NotImplementedError
