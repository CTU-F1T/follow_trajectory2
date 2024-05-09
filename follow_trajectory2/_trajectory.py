#!/usr/bin/env python
# trajectory.py
"""Class for handling and storing the Trajectory.

Similar to Path but requires time information.
"""
######################
# Imports & Globals
######################

import math
import numpy

from autopsy.node import ROS_VERSION

from ._utils import (
    angle_between_vectors,
    point_distance,
    quaternion_to_yaw,
)

from geometry_msgs.msg import (
    Point,
    Quaternion,
)

from scipy.spatial.transform import (
    Slerp,
    Rotation,
)


# Lambdas
obtain = lambda t, field: numpy.asarray(  # noqa: E731
    [getattr(tpoint, field) for tpoint in t] + [getattr(t[0], field)]
)


######################
# Trajectory class
######################

class Trajectory(object):
    """Object for representing a trajectory for car."""

    trajectory = None
    endpoint_time = None

    # Fields
    _x = None
    _y = None
    _z = None

    _ox = None
    _oy = None
    _oz = None
    _ow = None

    _i = None
    _t = None
    _k = None
    _v = None
    _a = None

    _delta = None
    _theta = None


    def __init__(self, msg, vehicle):
        """Initialize the Trajectory.

        Arguments:
        msg -- message containing the trajectory, autoware_auto_msgs/Trajectory
        vehicle -- current Vehicle instance for parameters, Vehicle
        """
        super(Trajectory, self).__init__()

        self.header = msg.header

        _oz = numpy.asarray([
            tpoint.pose.orientation.z for tpoint in msg.points
        ])
        _ow = numpy.asarray([
            tpoint.pose.orientation.w for tpoint in msg.points
        ])
        _theta = numpy.unwrap(numpy.arctan2(_oz, _ow) * 2)

        if ROS_VERSION == 1:
            self.trajectory = [
                TrajectoryPoint(
                    tpoint.pose.position,
                    tpoint.pose.orientation,
                    i = i,
                    t = tpoint.time_from_start.to_sec(),
                    v = math.sqrt(
                        tpoint.longitudinal_velocity_mps**2
                        + tpoint.lateral_velocity_mps**2
                    ),
                    a = tpoint.acceleration_mps2,
                    k = math.tan(tpoint.front_wheel_angle_rad) / vehicle.L,
                    delta = tpoint.front_wheel_angle_rad,
                    theta = _theta[i],
                ) for i, tpoint in enumerate(msg.points)
            ]
        else:
            self.trajectory = [
                TrajectoryPoint(
                    tpoint.pose.position,
                    tpoint.pose.orientation,
                    i = i,
                    t = tpoint.time_from_start.sec + (
                        tpoint.time_from_start.nanosec / 1.0e9
                    ),
                    v = math.sqrt(
                        tpoint.longitudinal_velocity_mps**2
                        + tpoint.lateral_velocity_mps**2
                    ),
                    a = tpoint.acceleration_mps2,
                    k = math.tan(tpoint.front_wheel_angle_rad) / vehicle.L,
                    delta = tpoint.front_wheel_angle_rad,
                    theta = _theta[i],
                ) for i, tpoint in enumerate(msg.points)
            ]

        if self.trajectory[-1].t is not None:
            self.endpoint_time = (
                self.trajectory[-1].t
                + point_distance(self.trajectory[-1], self.trajectory[0]) * 2
                / (self.trajectory[0].v + self.trajectory[-1].v)
            )

        if hasattr(self.get(0), "x"):
            self._x = obtain(self.trajectory, "x")
            self._y = obtain(self.trajectory, "y")
            self._z = obtain(self.trajectory, "z")

        if hasattr(self.get(0), "orientation"):
            self._ox = numpy.asarray(
                [tpoint.orientation.x for tpoint in self.trajectory]
                + [self.get(0).orientation.x]
            )
            self._oy = numpy.asarray(
                [tpoint.orientation.y for tpoint in self.trajectory]
                + [self.get(0).orientation.y]
            )
            self._oz = numpy.asarray(
                [tpoint.orientation.z for tpoint in self.trajectory]
                + [self.get(0).orientation.z]
            )
            self._ow = numpy.asarray(
                [tpoint.orientation.w for tpoint in self.trajectory]
                + [self.get(0).orientation.w]
            )

        self._i = obtain(self.trajectory, "i")
        self._t = obtain(self.trajectory, "t")
        if self._t[0] is not None:
            # In fact, this is the repeated first point. This replication
            # is done by 'obtain' lambda function.
            self._t[-1] = self.endpoint_time
        self._k = obtain(self.trajectory, "k")
        self._v = obtain(self.trajectory, "v")
        self._a = obtain(self.trajectory, "a")

        self._delta = obtain(self.trajectory, "delta")
        self._theta = obtain(self.trajectory, "theta")


    def get(self, index):
        """Obtain point of the Trajectory.

        Arguments:
        index -- index of the point

        Returns:
        point -- path point with states, TrajectoryPoint
        """
        return self.trajectory[index % self.length]


    @property
    def length(self):
        """Obtain the size of the trajectory.

        Returns:
        size -- number of points in the Trajectory, int
        """
        return len(self.trajectory)


    def lateral_distance(self, p):
        """Compute distance of 'p' to the trajectory.

        Arguments:
        p -- point for distance computation, Point

        Returns:
        dist -- distance to the trajectory, float
        """
        _, tp = self.get_closest_point(p, interpolate = True)
        return tp.distanceTo(p)


    def time_distance(self, i, j, bidirectional = False):
        """Compute the time distance between two point of the trajectory.

        Arguments:
        i -- index of the first point, int
        j -- index of the second point, int

        Returns:
        tdist -- time between the points, float
        """
        tdist = 0
        _tp1 = self.get(i)
        _tp2 = self.get(j)

        if not bidirectional:
            if _tp1.i > _tp2.i:
                tdist += self.get(-1).t

        tdist += _tp2.t - _tp1.t

        return tdist


    def closest_point(self, position):
        """Find a closest point on the trajectory to the specified position.

        Arguments:
        position -- position to be closest to

        Returns:
        distance -- distance between the points
        i -- index of the closest point

        TODO: Hints. (Index of the trajectory point that is probably close.)
        """
        _distances = numpy.hypot(self._x - position.x, self._y - position.y)
        _min_i = numpy.argmin(_distances)
        return _distances[_min_i], _min_i


    def closest_point_time(self, time):
        """Find a closest point on the trajectory to the specified time.

        Arguments:
        time -- time to be closest to

        Returns:
        time_distance -- time delta between the points
        i -- index of the closest point

        TODO: Solve the endpoints as they are not properly used.
        """
        if self._t[0] is None:
            raise ValueError(
                "Unable to find time-closest point as trajectory lacks "
                "timestamps."
            )

        _distances = numpy.sqrt(
            numpy.power(self._t - (time % max(self._t)), 2)
        )
        _min_i = numpy.argmin(_distances)
        return _distances[_min_i], _min_i


    def get_closest_point(self, position, interpolate = False):
        """Obtain the closest point on the trajectory to the specified position.

        Arguments:
        position -- position to be closest to
        interpolate -- when True, an interpolated point is returned instead

        Returns:
        i -- index of the closest point on the trajectory, int/float
        point -- closest point on the trajectory

        When `interpolate = True`, index is returned as float to indicate
        the "position" on the trajectory.
        """
        _distances = numpy.hypot(self._x - position.x, self._y - position.y)
        _min_i = numpy.argmin(_distances)

        if not interpolate or _distances[_min_i] == 0:
            return _min_i, self.get(_min_i)

        # Find orientation to the point
        # Inspired by 'trajectoryClosestIndex' from ng_trajectory.
        # Since the points can be isochronal and not equidistant,
        #  we cannot rely on having the second point 'second closest'.
        alpha = angle_between_vectors(
            self.get(_min_i), self.get(_min_i + 1), self.get(_min_i), position
        )
        beta = angle_between_vectors(
            self.get(_min_i), self.get(_min_i - 1), self.get(_min_i), position
        )

        if alpha < (math.pi / 2) and alpha < beta:
            """
                                  (position)
                                    /
                                   /
                           beta   /  alpha
                                 /
            (_min_i-1)--------(_min_i)-------------(_min_i+1)
            """  # noqa: W605
            ref = self.get(_min_i + 1)
            ratio = (
                (_distances[_min_i] * math.cos(alpha))
                / (self.get(_min_i).distanceTo(self.get(_min_i + 1)))
            )
            index = _min_i + ratio
        elif beta < (math.pi / 2) and beta < alpha:
            """
                         (position)
                              \
                               \
                         beta   \   alpha
                                 \
            (_min_i-1)--------(_min_i)-------------(_min_i+1)
            """  # noqa: W605
            ref = self.get(_min_i - 1)
            ratio = (
                (_distances[_min_i] * math.cos(beta))
                / (self.get(_min_i).distanceTo(self.get(_min_i - 1)))
            )
            index = _min_i - ratio
        else:
            """
                      (_min_i+1)
                           \
                            \
                             \     alpha
                              \
            (_min_i-1)-----(_min_i)
                                   ----
                             beta      ----
                                           ---
                                              --(position)
            """  # noqa: W605
            return _min_i, self.get(_min_i)

        if ratio > 1.0:
            # I think that this is not possible.
            print (position, _min_i, self.get(_min_i), alpha, beta, ref, ratio)
            raise ValueError(
                "Ratio exceeded expected value: %f !<= 1.0" % ratio
            )

        if ratio < 0.0:
            # This might happen on the first point of the trajectory
            index = ref.i + (1 - ratio)

        return index, TrajectoryPoint.interpolate(self.get(_min_i), ref, ratio)


    def time_reinterpolate(self, dt):
        """Reinterpolate the trajectory to be isochronal.

        Arguments:
        dt -- time delta for the trajectory

        Note: It is necessary to use TrajectoryA with time.
        """
        if self.get(0).t is None:
            raise ValueError(
                "Unable to reinterpolate the trajectory "
                "as the time is missing."
            )


        t_interp = numpy.arange(0, self._t[-1], dt)
        # Note: Might need to add another time point to properly connect
        #       last point of the trajectory to the first one.

        self._x = numpy.interp(t_interp, self._t, self._x)
        self._y = numpy.interp(t_interp, self._t, self._y)
        self._z = numpy.interp(t_interp, self._t, self._z)

        # Interpolate Quaternions using Slerp
        _o = Slerp(
            self._t,
            Rotation.from_quat(
                numpy.vstack((self._ox, self._oy, self._oz, self._ow)).T
            )
        )(t_interp)

        (self._ox, self._oy, self._oz, self._ow) = (
            numpy.hsplit(_o.as_quat(), 4)
        )

        self._i = numpy.interp(t_interp, self._t, self._i)
        self._k = numpy.interp(t_interp, self._t, self._k)
        self._v = numpy.interp(t_interp, self._t, self._v)
        self._a = numpy.interp(t_interp, self._t, self._a)

        #  self._theta = numpy.interp(t_interp, self._t, self._theta)
        # Consider: self._o.as_euler("xyz")
        self._theta = numpy.asarray(
            [
                quaternion_to_yaw(
                    Quaternion(
                        *_q.as_quat()
                    )
                ) for _q in _o
            ]
        )
        self._delta = numpy.interp(t_interp, self._t, self._delta)

        self._t = t_interp

        # Create TrajectoryPoints
        self.trajectory = [
            TrajectoryPoint(
                Point(float(_x), float(_y), float(_z)),
                Quaternion(float(_ox), float(_oy), float(_oz), float(_ow)),
                i = _i,
                t = _t,
                k = _k,
                v = _v,
                a = _a,
                theta = _theta,
                delta = _delta,
            ) for (
                _x, _y, _z,
                _ox, _oy, _oz, _ow,
                _i, _t, _k,
                _v, _a, _theta, _delta
            ) in zip(
                self._x, self._y, self._z,
                self._ox, self._oy, self._oz, self._ow,
                self._i, self._t, self._k,
                self._v, self._a, self._theta, self._delta
            )
        ]


######################
# TrajectoryPoint class
######################

class TrajectoryPoint(Point):
    """Object representing a point in a Trajectory."""

    def __init__(self, point = None, quaternion = None, **kwargs):
        """Initialize the TrajectoryPoint.

        Arguments:
        point -- location of a point, geometry_msgs.msg/Point
        quaternion -- rotation in the point, geometry_msgs.msg/Quaternion

        Keyword optional arguments:
        i -- index of the point, int
        v -- [m.s^-1] velocity in the point, float
        a -- [m.s^-2] acceleration in the point, float
        k -- [m^-1] curvature of the path in the point, float
        t -- [s] time when the car should be at this point, float
        delta -- [rad] car steering, float
        theta -- [rad] car orientation, float
        """
        super(TrajectoryPoint, self).__init__()

        if point is not None:
            self.x = point.x
            self.y = point.y
            self.z = point.z

        if quaternion is not None:
            self.orientation = quaternion

        self.i = kwargs.get("i", None)
        self.v = kwargs.get("v", 0)
        self.a = kwargs.get("a", 0)
        self.k = kwargs.get("k", 0)
        self.t = kwargs.get("t", None)
        self.delta = kwargs.get("delta", None)
        self.theta = kwargs.get("theta", None)

    @property
    def yaw(self):
        """Obtain car orientation from the quaternion."""
        return (
            quaternion_to_yaw(self.orientation)
            if self.orientation is not None else 0.0
        )


    def distanceTo(self, other):
        """Distance to different point."""
        return (
            (other.x - self.x)**2
            + (other.y - self.y)**2
            + (other.z - self.z)**2
        )**0.5


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
        _q = Slerp(
            [0, 1.0],
            Rotation.from_quat([
                [
                    p1.orientation.x, p1.orientation.y,
                    p1.orientation.z, p1.orientation.w
                ],
                [
                    p2.orientation.x, p2.orientation.y,
                    p2.orientation.z, p2.orientation.w
                ]
            ])
        )([ratio]).as_quat()[0]

        return TrajectoryPoint(
            Point(
                x = p1.x + (p2.x - p1.x) * ratio,
                y = p1.y + (p2.y - p1.y) * ratio,
                z = p1.z + (p2.z - p1.z) * ratio
            ),
            Quaternion(
                x = _q[0],
                y = _q[1],
                z = _q[2],
                w = _q[3]
            ),
            i = p1.i + (p2.i - p1.i) * ratio,
            t = p1.t + (p2.t - p1.t) * ratio,
            k = p1.k + (p2.k - p1.k) * ratio,
            v = p1.v + (p2.v - p1.v) * ratio,
            a = p1.a + (p2.a - p1.a) * ratio,
            theta = p1.theta + (p2.theta - p1.theta) * ratio,
            delta = p1.delta + (p2.delta - p1.delta) * ratio,
        )


    def __str__(self):
        """Represent the point as a string."""
        ret = []

        ret += [("x", self.x), ("y", self.y), ("z", self.z)]

        if self.orientation is not None:
            ret += [
                ("ow", self.orientation.w), ("ox", self.orientation.x),
                ("oy", self.orientation.y), ("oz", self.orientation.z)
            ]

        if self.i is not None:
            ret += [("i", self.i)]

        ret += [("v", self.v), ("a", self.a), ("k", self.k)]

        if self.t is not None:
            ret += [("t", self.t)]

        if self.delta is not None:
            ret += [("delta", self.delta)]

        if self.theta is not None:
            ret += [("theta", self.theta)]

        return str(ret)
