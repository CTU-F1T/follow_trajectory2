#!/usr/bin/env python
# pure_pursuit.py
"""Pure Pursuit implementation for trajectory tracking.
"""
######################
# Imports & Globals
######################

#import rospy

from .controllerabc import *

from autopsy.reconfigure import ParameterServer

from autopsy.node import ROS_VERSION

if ROS_VERSION == 1:
    import rospy

    def update_parameters(p):
        if rospy.has_param("~"):
            p.update(rospy.get_param("~"), only_existing = True)
else:
    def update_parameters(p):
        pass


# Messages
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped


# Parameters
name = "PurePursuit"
value = 1

# Reconfigurable parameters
PARAMETERS = [
    # Gain of the lookahead distance
    ("k_const", {"default": 0.7, "min": 0.0, "max": 1.2, "description": "(PP) Gain of the lookahead distance [-]", "ns": "pp"}),

    # Limits for the lookahead distance
    ("max_lookfwd_dist", {"default": 3.0, "min": 0.0, "max": 5.0, "description": "Maximum lookahead distance [m]", "ns": "default"}),
    ("min_lookfwd_dist", {"default": 0.35, "min": 0.0, "max": 5.0, "description": "Minimum lookahead distance [m]", "ns": "default"}),
]


######################
# PurePursuit class
######################

class Controller(ControllerABC):

    # ParameterServer
    P2 = None

    def __init__(self, *args, **kwargs):
        super(Controller, self).__init__(*args, **kwargs)

        self.P2 = ParameterServer()
        self.P2.update(PARAMETERS)

        update_parameters(self.P2)

        self.P2.reconfigure(namespace = "pure_pursuit", node = self.node)

        #self.pub_la_point = rospy.Publisher("trajectory/lookahead_point", PointStamped, queue_size = 1)


    def select_point(self, controller, trajectory):
        distance, i = trajectory.closest_point(self.Vehicle.rear_axle)

        return i, trajectory.get(i)


    def compute(self, controller, tpoint, trajectory, trajectory_i):

        trajectory_i = int(trajectory_i)
        act_velocity = self.Vehicle.v
        distance = point_distance(tpoint, self.Vehicle)


        ## Compute the velocity ##
        # Failsafe in case that we are too close to the point
        # FIXME: Maybe this causes the 'wavy' behaviour on slightly waved sections?
        if distance < 0.4:  # failsave
            desired_velocity = tpoint.v
        else:
            # FIXME: What are those numbers?
            # It reduces the velocity up to distance 8.7m ~ 55.5%.
            # 0.4-1.8m ~ 90%
            # TODO: Do we need this?
            desired_velocity = tpoint.v / max(
                min(1 + 0.1 * (distance - 0.6), 1.8), 1.1)


        ## Compute the acceleration required to reach the velocity ##
        # FIXME: Consider how to use acceleration.
        v_diff = desired_velocity - act_velocity
        acc = constrain(
            v_diff / self.Vehicle.dt,  # + acceleration
            controller.P.max_brk_dcc.value,
            controller.P.max_fwd_acc.value
        )


        ## Estimated the velocity ##
        # FIXME: Shouldn't we do this before changing the 'acc'?
        #        I mean if would make much MORE sense.
        act_velocity += acc * self.Vehicle.dt


        ## Compute the LA ##
        look_ahead_dist = constrain(
            self.P2.k_const * act_velocity,
            self.P2.min_lookfwd_dist.value,
            self.P2.max_lookfwd_dist.value
        )


        ## Select a TrajectoryPoint based on LA ##
        # Note: trajectory index is used to speed this up.
        goal_point_id = 0
        for i in range(trajectory.length):
            next_to_try = (trajectory_i + i) % trajectory.length
            distance_temp = point_distance(self.Vehicle.rear_axle, trajectory.get(next_to_try))

            if distance_temp > look_ahead_dist:
                goal_point_id = next_to_try
                break

        #self.pub_la_point.publish(PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="map"), point=trajectory.get(goal_point_id)))


        ## Compute the required steering angle ##
        # Calculation of pure pursuit
        alpha = angle_between_vectors(self.Vehicle.rear_axle,
                                      trajectory.get(goal_point_id),
                                      self.Vehicle.rear_axle,
                                      self.Vehicle.front_axle)

        angle_direction = determine_side(self.Vehicle.rear_axle,
                                         trajectory.get(goal_point_id),
                                         self.Vehicle.front_axle)

        alpha *= angle_direction  # for oriented angle

        steer_angle = math.atan((2 * self.Vehicle.L * math.sin(alpha)) / look_ahead_dist)

        #self.Vehicle.v = act_velocity

        return act_velocity, steer_angle
