#!/usr/bin/env python
# vehicle.py
"""Class for Vehicle and its states.
"""
######################
# Imports & Globals
######################

from ._utils import calc_rotation, quaternion_to_yaw

# Messages
from geometry_msgs.msg import Point, Quaternion


######################
# Vehicle class
######################

class Vehicle(object):

    # Predefine attributes
    #__slots__ = ["x", "y", "z", "v", "d", "yaw"]


    def __init__(self):
        super(Vehicle, self).__init__()

        # Position and orientation
        self.position = Point()
        self.orientation = Quaternion()

        # States
        self.v = 0
        self.d = 0

        # Others
        self.dt = 0

        # Vehicle parameters
        self.l_f = 0.16
        self.l_r = 0.16
        self.L = self.l_f + self.l_r


    @property
    def front_axle(self):
        _rotation = calc_rotation(self.orientation)

        return Point(
            x = self.x + self.l_f * _rotation[0],
            y = self.y + self.l_f * _rotation[1],
            z = self.z + self.l_f * _rotation[2]
        )

    @property
    def rear_axle(self):
        _rotation = calc_rotation(self.orientation)

        return Point(
            x = self.x - self.l_r * _rotation[0],
            y = self.y - self.l_r * _rotation[1],
            z = self.z - self.l_r * _rotation[2]
        )

    ## Position
    @property
    def position(self):
        return self.__position

    @position.setter
    def position(self, position):
        self.__position = position

    @property
    def x(self):
        return self.__position.x

    @x.setter
    def x(self, x):
        self.__position.x = x

    @property
    def y(self):
        return self.__position.y

    @y.setter
    def y(self, y):
        self.__position.y = y

    @property
    def z(self):
        return self.__position.z

    @z.setter
    def z(self, z):
        self.__position.z = z

    ## Orientation
    @property
    def orientation(self):
        return self.__orientation

    @orientation.setter
    def orientation(self, orientation):
        self.__orientation = orientation

    @property
    def yaw(self):
        return quaternion_to_yaw(self.orientation)


    ## States
    @property
    def v(self):
        return self.__velocity

    @v.setter
    def v(self, velocity):
        self.__velocity = velocity

    @property
    def d(self):
        return self.__steering

    @d.setter
    def d(self, steering):
        self.__steering = steering


    ## Others
    @property
    def dt(self):
        return self.__dt

    @dt.setter
    def dt(self, delta):
        self.__dt = delta


    # Functions
    def stop(self):
        """Stops the cars -- sets all states to zeros."""
        self.v = 0
        self.d = 0
