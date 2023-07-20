#!/usr/bin/env python
# path.py
"""Class for handling and storing the Path.
"""
######################
# Imports & Globals
######################

import math
import numpy

from ._utils import *

from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Path

from scipy.spatial.transform import Slerp, Rotation


# Lambdas
obtain = lambda t, field: numpy.asarray([ getattr(tpoint, field) for tpoint in t ])


######################
# Path class
######################

class Path(object):

    path = None

    # Fields
    _x = None
    _y = None
    _z = None

    _ox = None
    _oy = None
    _oz = None
    _ow = None

    _i = None
    _k = None

    _delta = None
    _theta = None


    def __init__(self, msg, vehicle):
        """Initialize the Path.

        Arguments:
        msg -- message containing the path, nav_msgs/Path
        vehicle -- current Vehicle instance for parameters, Vehicle
        """
        super(Path, self).__init__()

        self.header = msg.header

        _oz = numpy.asarray([ pose.pose.orientation.z for pose in msg.poses ])
        _ow = numpy.asarray([ pose.pose.orientation.w for pose in msg.poses ])
        _theta = numpy.unwrap(numpy.arctan2(_oz, _ow)*2)

        self.path = [
            PathPoint(
                pose.pose.position,
                pose.pose.orientation,
                i = i,
                theta = _theta[i],
            ) for i, pose in enumerate(msg.poses)
        ]

        if hasattr(self.get(0), "x"):
            self._x = obtain(self.path, "x")
            self._y = obtain(self.path, "y")
            self._z = obtain(self.path, "z")

        if hasattr(self.get(0), "orientation"):
            self._ox = numpy.asarray([ pose.orientation.x for tpoint in self.trajectory ])
            self._oy = numpy.asarray([ tpoint.orientation.y for tpoint in self.trajectory ])
            self._oz = numpy.asarray([ tpoint.orientation.z for tpoint in self.trajectory ])
            self._ow = numpy.asarray([ tpoint.orientation.w for tpoint in self.trajectory ])

        self._i = obtain(self.trajectory, "i")

        self._theta = obtain(self.trajectory, "theta")


    def get(self, index):
        """Obtain point of the Path.

        Arguments:
        index -- index of the point

        Returns:
        point -- path point with states, PathPoint
        """
        return self.path[index % self.length]


    @property
    def length(self):
        """Obtain the size of the path.

        Returns:
        size -- number of points in the Path, int
        """
        return len(self.path)


    def closest_point(self, position):
        """Find a closest point on the pth to the specified position.

        Arguments:
        position -- position to be closest to

        Returns:
        distance -- distance between the points
        i -- index of the closest point

        TODO: Hints. (Index of the path point that is probably close.)
        """
        _distances = numpy.hypot(self._x - position.x, self._y - position.y)
        _min_i = numpy.argmin(_distances)
        return _distances[_min_i], _min_i


######################
# PathPoint class
######################

class PathPoint(Point):

    def __init__(self, point = None, quaternion = None, **kwargs):
        super(PathPoint, self).__init__()

        if point is not None:
            self.x = point.x
            self.y = point.y
            self.z = point.z

        if quaternion is not None:
            self.orientation = quaternion

        self.i = kwargs.get("i", None)
        self.theta = kwargs.get("theta", None)

    @property
    def yaw(self):
        return quaternion_to_yaw(self.orientation) if self.orientation is not None else 0.0


    def distanceTo(self, other):
        """Distance to different point."""
        return ((other.x - self.x)**2 + (other.y - self.y)**2 + (other.z - self.z)**2)**0.5


    @staticmethod
    def interpolate(p1, p2, ratio):
        """Obtain interpolated point.

        Arguments:
        p1 -- TrajectoryPoint
        p2 -- TrajectoryPoint
        ratio -- number in (0.0, 1.0)

        Return:
        interpolated point -- instance of TrajectoryPoint
        """
        return PathPoint(
            Point(
                p1.x + (p2.x - p1.x) * ratio,
                p1.y + (p2.y - p1.y) * ratio,
                p1.z + (p2.z - p1.z) * ratio
            ),
            Quaternion(
                *Slerp([0, 1.0], Rotation.from_quat([[p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w], [p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w]]))([ratio]).as_quat()[0]
            ),
            i = p1.i + (p2.i - p1.i) * ratio,
            theta = p1.theta + (p2.theta - p1.theta) * ratio,
        )


    def __str__(self):
        ret = []

        ret += [ ("x", self.x), ("y", self.y), ("z", self.z) ]

        if self.orientation is not None:
            ret += [ ("ow", self.orientation.w), ("ox", self.orientation.x), ("oy", self.orientation.y), ("oz", self.orientation.z) ]

        if self.i is not None:
            ret += [ ("i", self.i) ]

        if self.theta is not None:
            ret += [ ("theta", self.theta) ]

        return str(ret)
