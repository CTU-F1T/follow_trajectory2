#!/usr/bin/env python
# _utils.py
"""Utilities for 'follow_trajectory2'.
"""

import math

from geometry_msgs.msg import Point


def point_distance(a, b):
    """Computes distance between two points in 2D.

    Arguments:
    a -- first point, class with '.x': float and '.y': float,
         e.g., geometry_msgs.msg/Point
    b -- second point, class with '.x': float and '.y': float,
         e.g., geometry_msgs.msg/Point

    Returns:
    d -- distance between points, float
    """
    return math.sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2))


def determine_side(a, b, p):
    """ Determines, if car is on right side of trajectory or on left side
    Arguments:
         a - point of trajectory, which is nearest to the car, geometry_msgs.msg/Point
         b - next trajectory point, geometry_msgs.msg/Point
         p - actual position of car, geometry_msgs.msg/Point

    Returns:
         -1 if car is on left side of trajectory
         1 if car is on right side of trajectory
         0 if car is on trajectory
    """
    side = (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x)
    if side > 0:
        return 1
    elif side < 0:
        return -1
    else:
        return 0


def angle_between_vectors(p1, p2, p3, p4):
    """Finds angle between two vectors each defined by two points
    Args:
        p1 - first point of first vector, geometry_msgs.msg/Point
        p2 - second point of first vector, geometry_msgs.msg/Point
        p3 - first point of second vector, geometry_msgs.msg/Point
        p4 - second point of second vector, geometry_msgs.msg/Point

    Returns:
        ret - angle between two vectors, radians, float
    """
    w1 = [p2.x - p1.x, p2.y - p1.y]
    w2 = [p4.x - p3.x, p4.y - p3.y]
    temp = (w1[0] * w2[0] + w1[1] * w2[1]) / (
            (math.sqrt(math.pow(w1[0], 2) + math.pow(w1[1], 2))) *
            (math.sqrt(math.pow(w2[0], 2) + math.pow(w2[1], 2))))
    try:
        ret = math.acos(temp)
    except:
        print (p1, p2, p3, p4)
        raise

    return ret


def calc_rotation(q):
    """Rotate a vector using Quaternion.

    Arguments:
    q -- Quaternion containing the coordinate rotation

    Returns:
    p2 -- rotated x-axis unit vector, 3-list of floats

    Note: This equals to:
    ```python

    # Source: https://answers.ros.org/question/196149/how-to-rotate-vector-by-quaternion-in-python/
    from tf.transformations import quaternion_multiply, \
                                   quaternion_conjugate

    q1 = list(q)
    q2 = [1, 0, 0, 0]

    return list(
        quaternion_multiply(
            quaternion_multiply(
                q1, q2
            ),
            quaternion_conjugate(
                q1
            )
        )
    )[:3]

    ```
    """
    p1x = 1
    p1y = 0
    p1z = 0

    p2x = q.w * q.w * p1x + 2 * q.y * q.w * p1z - 2 * q.z * q.w * p1y + q.x * q.x * p1x + 2 * q.y * q.x * p1y + 2 * q.z * q.x * p1z - q.z * q.z * p1x - q.y * q.y * p1x
    p2y = 2 * q.x * q.y * p1x + q.y * q.y * p1y + 2 * q.z * q.y * p1z + 2 * q.w * q.z * p1x - q.z * q.z * p1y + q.w * q.w * p1y - 2 * q.x * q.w * p1z - q.x * q.x * p1y
    p2z = 2 * q.x * q.z * p1x + 2 * q.y * q.z * p1y + q.z * q.z * p1z - 2 * q.w * q.y * p1x - q.y * q.y * p1z + 2 * q.w * q.x * p1y - q.x * q.x * p1z + q.w * q.w * p1z

    return [p2x, p2y, p2z]


def quaternion_to_yaw(q):
    """Obtain a yaw angle from a Quaternion.

    Arguments:
    q -- Quaternion

    Returns:
    yaw -- Euler angle yaw from Quaternion, float

    Note:
    This is same as:
        np.arctan2(trajectory_file["orientation.z"],trajectory_file["orientation.w"])*2
    But this version works in general.

    I think that this could be also done using tf.transformations.euler_from_quaternion().
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def constrain(val, min_val, max_val):
    """Constrain the value in variable to be within the bounds."""
    return min(max_val, max(min_val, val))
