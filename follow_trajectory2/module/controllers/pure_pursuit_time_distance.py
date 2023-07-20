#!/usr/bin/env python
# pure_pursuit_p.py
"""Pure Pursuit+ implementation for trajectory tracking.

This version requires a Trajectory with time information included.

The 'classic' PurePursuit then turns into a different version that
does not utilize current speed (estimated) to compute LA point, but
it is selected statically by timeshift.
"""
######################
# Imports & Globals
######################

#import rospy

from controllerabc import *

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
from std_msgs.msg import Header, String
from geometry_msgs.msg import PointStamped


# Parameters
name = "TimePurePursuit"
value = 4

# Reconfigurable parameters
PARAMETERS = [
    # Gain of the lookahead distance
    ("k_const", {"default": 0.7, "min": 0.0, "max": 1.2, "description": "Lookahead time-distance [s]"}),

    # Time-based lookahead
    ("k_speed", {"default": 0.07, "min": -1.2, "max": 1.2, "description": "Lookahead time-distance for speed [s]"}),

    # Limits for the lookahead distance
    ("min_lookfwd_dist", {"default": 0.3, "min": 0.0, "max": 5.0, "description": "Minimum lookahead distance [m]", "ns": "default"}),
    ("max_lookfwd_dist", {"default": 1.1, "min": 0.0, "max": 5.0, "description": "Maximum lookahead distance [m]", "ns": "default"}),

    # Experimental correction
    ("k_corr", {"default": -0.06, "min": -0.2, "max": 0.2, "description": "Steer correction per curvature"}),
    ("k_corr2", {"default": 0.0, "min": -0.5, "max": 0.5, "description": "LA steer correction per curvature"}),
    ("v_thresh", {"default": 3.7, "min": 0.5, "max": 7.0, "description": "Speed threshold for using different limits"}),
    ("max_lookfwd_dist2", {"default": 1.4, "min": 0.0, "max": 5.0, "description": "Maximum LA [m]"}),
    ("k_accel", {"default": 0.0, "min": -1.0, "max": 1.0, "description": "Coefficient to correct speed"}),
    ("k_time", {"default": 3.5, "min": 0.0, "max": 10.0, "description": "Coefficient to correct speed from time difference"}),
    ("time_tracking", False),
    ("k_ori", {"default": 0.05, "min": -1.0, "max": 1.0, "description": "Coefficient to correct orientation from difference."}),
    ("k_ori2", {"default": -0.001, "min": -1.0, "max": 1.0, "description": "LA Coefficient to correct orientation from difference."}),
    ("time_interpolation", False),
    ("dt", {"default": 0.02, "min": 0.001, "max": 1.0, "description": "Delta t for reinterpolating the trajectory to make the points isochronal."}),
    ("k_alpha", {"default": 1.0, "min": 0.0, "max": 2.0, "description": "Weight for alpha steering"}),
    ("k_delta", {"default": 0.0, "min": -1.0, "max": 1.0, "description": "Weight for delta difference"}),
    ("k_delta2", {"default": 0.0, "min": -1.0, "max": 1.0, "description": "LA Weight for delta difference"}),
]


######################
# PurePursuit class
######################

class Controller(ControllerABC):

    # ParameterServer
    P2 = None

    # Timetracking
    _time = None
    time_last = None

    def __init__(self, *args, **kwargs):
        super(Controller, self).__init__(*args, **kwargs)

        self.P2 = ParameterServer()
        self.P2.update(PARAMETERS)
        self.P2.link(self.P2.min_lookfwd_dist, self.P2.max_lookfwd_dist)

        update_parameters(self.P2)

        self.P2.reconfigure(namespace = "follow_trajectory/pure_pursuit_time_dist")

        #self.pub_la_point = rospy.Publisher("trajectory/lookahead_point", PointStamped, queue_size = 1)
        #self.pub_sp_point = rospy.Publisher("trajectory/speed_point", PointStamped, queue_size = 1)
        #self.pub_sp_point_str = rospy.Publisher("trajectory/speed_point/string", String, queue_size = 1)


    def process_trajectory(self, controller, trajectory):
        if self.P2.time_interpolation:
            trajectory.time_reinterpolate(self.P2.dt.value)


    def select_point(self, controller, trajectory):
        # Switch to time-tracking after %f seconds.
        if controller.time_elapsed > 10.0 and self.P2.time_tracking.value:
            self._time += controller.time_last - self.time_last
            distance, i = trajectory.closest_point_time(self._time)

            # When the points are too far from each other, stop the car.
            #if point_distance(trajectory.get(i), trajectory.get(i2)) > 0.3:
            #    self.Vehicle.stop()
            #    controller.publish_action(0, 0)
            #    raise Exception("Tracked point is too far away from the current location.")
        else:
            distance, i = trajectory.closest_point(self.Vehicle.rear_axle)
            self._time = trajectory.get(i).t

        self.time_last = controller.time_last
        return i, trajectory.get(i)


    def compute(self, controller, tpoint, trajectory, trajectory_i):

        # Fail to continue when we have no time information
        if tpoint.t is None:
            raise ValueError("Unable to do the time lookahead when the trajectory lacks timeshifts.")

        trajectory_i = int(trajectory_i)
        act_velocity = self.Vehicle.v

        if self.P2.time_tracking.value:
            _, i2 = trajectory.closest_point(self.Vehicle.rear_axle)
            #t_dist = trajectory.time_distance(trajectory_i, i2)
            t_dist = trajectory.get(trajectory_i).t - trajectory.get(i2).t

            if abs(t_dist) * 2 > max(trajectory._t):
                t_dist = t_dist % max(trajectory._t)

            #print ("Time diff: %02.6f  Pos diff: %02.6f  Vel: %02.6f" % (t_dist, point_distance(trajectory.get(trajectory_i), trajectory.get(i2)), t_dist * self.P2.k_time))
        else:
            t_dist = 0



        ### 1) Compute velocity ###
        distance = point_distance(tpoint, self.Vehicle)


        ## Select next point to compute velocity
        if tpoint.t is not None and self.P2.k_speed != 0:

            if self.P2.k_speed > 0:
                for i in range(trajectory.length):
                    t_diff = trajectory.get(trajectory_i + i).t - tpoint.t

                    if t_diff < 0:
                        t_diff += trajectory.get(-1).t

                    if t_diff > self.P2.k_speed:
                        tpoint = trajectory.get(trajectory_i + i)
                        break
            else:
                for i in reversed(range(trajectory.length)):
                    t_diff = trajectory.get(trajectory_i - i).t - tpoint.t

                    if t_diff > 0:
                        t_diff -= trajectory.get(-1).t

                    if t_diff < self.P2.k_speed:
                        tpoint = trajectory.get(trajectory_i - i)
                        break

        #self.pub_sp_point.publish(PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="map"), point=tpoint))
        #self.pub_sp_point_str.publish(String(str(tpoint)))

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
        #act_velocity += acc * self.Vehicle.dt
        act_velocity = desired_velocity + acc * self.Vehicle.dt * self.P2.k_accel + t_dist * self.P2.k_time


        ### 2) Compute steering ###
        ## Select a TrajectoryPoint based on LA ##
        # Note: trajectory index is used to speed this up.
        goal_point_id = 0
        look_ahead_dist = 0

        for i in range(trajectory.length):
            time_distance = trajectory.time_distance(trajectory_i, trajectory_i + i)
            look_ahead_dist = point_distance(self.Vehicle, trajectory.get(trajectory_i + i))

            # Note: We make k_const as a time distance parameter.
            # It does not make any sense to use speed in here, as the PP approach computes:
            # P * v = s ~ meaning that P ~ time.
            if (time_distance > self.P2.k_const and look_ahead_dist > self.P2.min_lookfwd_dist) or look_ahead_dist > (self.P2.max_lookfwd_dist2 if act_velocity > self.P2.v_thresh else self.P2.max_lookfwd_dist):
                # Note: This is now constrained by min/max la distance.
                # Sidenote: The limits are not hard, i.e., next point after breaking the limit is returned. However,
                #           this should not be an issue.
                goal_point_id = i + trajectory_i
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

        steer_angle = math.atan((2 * self.Vehicle.L * math.sin(alpha)) / look_ahead_dist) * self.P2.k_alpha
        steer_angle += self.P2.k_corr * trajectory.get(trajectory_i).k + self.P2.k_corr2 * trajectory.get(goal_point_id).k

        steer_angle += self.P2.k_ori * math.atan2(math.sin(trajectory.get(trajectory_i).yaw - self.Vehicle.yaw), math.cos(trajectory.get(trajectory_i).yaw - self.Vehicle.yaw))
        steer_angle += self.P2.k_ori2 * math.atan2(math.sin(trajectory.get(goal_point_id).yaw - self.Vehicle.yaw), math.cos(trajectory.get(goal_point_id).yaw - self.Vehicle.yaw))

        steer_angle += self.P2.k_delta * trajectory.get(trajectory_i).delta + self.P2.k_delta2 * trajectory.get(goal_point_id).delta


        ### 3) Finish ###
        #self.Vehicle.v = act_velocity

        return act_velocity, steer_angle
