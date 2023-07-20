#!/usr/bin/env python
# _run.py
"""Node for following a planned trajectory of the car.
"""
######################
# Imports & Globals
######################

from autopsy.reconfigure import ParameterServer
from autopsy.node import Node, ROS_VERSION

from ._path import Path
from ._trajectory import Trajectory
from ._utils import *
from ._vehicle import Vehicle


if ROS_VERSION == 1:
    import rospy

    def update_parameters(p):
        if rospy.has_param("~"):
            p.update(rospy.get_param("~"), only_existing = True)
else:
    import rclpy
    from rclpy.qos import *

    def update_parameters(p):
        pass


# Dynamic module list of Controllers
from enum import Enum

import follow_trajectory2.module.controllers as controllers

control_methods = {}

class ControlMethod(Enum):
    pass

for module in controllers.__all__:
    _ctrl = __import__("follow_trajectory2.module.controllers." + module, fromlist=["Controller", "name"])
    _d = {cm.name: cm.value for cm in ControlMethod}
    _d.update({_ctrl.name: _ctrl.value})
    ControlMethod = Enum("ControlMethod", _d)
    control_methods[_ctrl.value] = _ctrl.Controller


# Message Types
from command_msgs.msg import CommandArrayStamped, Command, CommandParameter
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Header
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from autoware_auto_msgs.msg import Trajectory as TrajectoryA
from vesc_msgs.msg import VescStateStamped


# Global parameters
PARAMETERS = [
    ("method", ControlMethod),

    ## Common variables
    # Acceleration limits
    ("max_fwd_acc", {"default": 6.0, "min": 0.0, "max": 10.0, "description": "Maximum forward acceleration [m.s^-2]", "ns": "default"}),
    ("max_brk_dcc", {"default": -6.0, "min": -10.0, "max": 0.0, "description": "Maximum backward deceleration [m.s^-2]", "ns": "default"}),
]


######################
# RunNode
######################

class RunNode(Node):

    def __init__(self):
        super(Node, self).__init__("follow_trajectory2")


        # Internal variables
        self.Vehicle = Vehicle()
        self.running = False
        self.time_last = self.get_time()
        self.time_elapsed = 0.0

        # Intialize controllers
        for key, ctrl in control_methods.items():
            control_methods[key] = ctrl(vehicle = self.Vehicle, node = self)


        self.P = ParameterServer()

        self.P.update(PARAMETERS)

        update_parameters(self.P)

        self.P.reconfigure(node = self)


        self._controller = control_methods.get(self.P.method.value)
        self.loginfo("Using controller '%d': %s" % (self.P.method.value, ControlMethod(self.P.method.value).name))
        self.P.method.callback = self.change_controller


        # Publishers
        self.pub_command = self.Publisher("/command", CommandArrayStamped, queue_size = 1)
        self.traj_point = self.Publisher("/trajectory/current_point", PointStamped, queue_size = 1)
        self.back_axle = self.Publisher("/trajectory/back_axle", PointStamped, queue_size = 1)
        self.traj_point_str = self.Publisher("/trajectory/current_point/string", String, queue_size = 1)


        # Subscribers
        # Odometry uses some tweaks available only in ROS1.
        if ROS_VERSION == 1:
            rospy.Subscriber("/odom", Odometry, self.callback_odom, queue_size = 1, buff_size = 1400, tcp_nodelay = False)
        else: # ROS_VERSION == 2
            self.create_subscription(Odometry, "/odom", self.callback_odom, qos_profile = QoSProfile(depth = 1, durability = DurabilityPolicy.VOLATILE, reliability = ReliabilityPolicy.BEST_EFFORT))

        self.Subscriber("/path", Path, self.callback_path)
        self.Subscriber("/trajectory", TrajectoryA, self.callback_trajectory)
        self.Subscriber("/eStop", Bool, self.callback_estop)

        if ROS_VERSION == 1:
            self.Subscriber("/sensors/core", VescStateStamped, self.callback_vesc)
        else: # ROS_VERSION == 2
            self.create_subscription(VescStateStamped, "/sensors/core", self.callback_vesc, qos_profile = QoSProfile(depth = 1, durability = DurabilityPolicy.VOLATILE, reliability = ReliabilityPolicy.BEST_EFFORT))


    ## Utils ##
    def get_time(self):
        """Obtain the current ROS time as float."""
        if ROS_VERSION == 1:
            return rospy.get_time()
        else: # ROS_VERSION == 2
            # https://github.com/ros2/rclpy/issues/293
            # 1e9 is float in Python 3, that leads to floating point error on division.
            return self.get_clock().now().nanoseconds / (10 ** 9)

    def get_time_now(self):
        """Obtain the current ROS timestamp."""
        if ROS_VERSION == 1:
            return rospy.Time.now()
        else: # ROS_VERSION == 2
            return self.get_clock().now().to_msg()


    ## Callbacks ##
    def callback_path(self, data):
        """Callback on the Path message."""
        self.saved_trajectory = Path(data, self.Vehicle)
        self.loginfo("Received path.")


    def callback_trajectory(self, data):
        """Store a Trajectory.

        Arguments:
        data -- autoware_auto_msgs/Trajectory
        """
        self.saved_trajectory = Trajectory(data, self.Vehicle)
        self.loginfo("Received trajectory.")
        self._controller.process_trajectory(self, self.saved_trajectory)


    def callback_estop(self, data):
        """Callback on `auto start/stop`. Reset the Vehicle states.

        Arguments:
        data -- std_msgs/Bool
        """
        self.loginfo("Using controller '%d': %s" % (self.P.method.value, ControlMethod(self.P.method.value).name))
        self.Vehicle.stop()
        self.running = not data.data

        if self.running:
            self.time_elapsed = 0.0


    def callback_vesc(self, data):
        """Callback on VESC to obtain current speed of the vehicle.

        Arguments:
        data -- vesc_msgs/VescStateStamped
        """
        self.Vehicle.v = data.state.speed / 4105.324277107


    def callback_odom(self, data):
        """Callback on the Odometry message.

        Arguments:
        data -- nav_msgs/Odometry
        """

        if not self.running:
            return


        # Update states
        self.Vehicle.position = data.pose.pose.position
        self.Vehicle.orientation = data.pose.pose.orientation
        self.Vehicle.dt = self.get_time() - self.time_last


        self.time_elapsed += self.Vehicle.dt

        #
        if self.saved_trajectory is not None:

            ## ... and now we continue in case that everything is OK.
            nearest_point_id, point = self._controller.select_point(self, self.saved_trajectory)

            ## Visualize the trajectory point + back axle position.
            self.traj_point.publish(
                PointStamped(
                    header = Header(frame_id="map"),
                    point = Point(
                        x = point.x,
                        y = point.y,
                        z = 0.0
                    )
                )
            )

            self.back_axle.publish(
                PointStamped(
                    header = Header(frame_id="odom"),
                    point = self.Vehicle.rear_axle
                )
            )

            self.traj_point_str.publish(
                String(data = str(point))
            )

            ## Obtain action values
            _velocity, _steer = self._controller.compute(self, point, self.saved_trajectory, nearest_point_id)

            self.publish_action(_velocity, _steer)


    ## Publishers ##
    def publish_action(self, velocity, steering):
        """Publish the action values to the Drive-API.

        Arguments:
        velocity -- desired velocity of the car, m.s^-1, float
        steering -- desired steering value, rad, float
        """
        self.pub_command.publish(CommandArrayStamped(
            header = Header(stamp = self.get_time_now()), commands = [
                Command(
                    command = "speed",
                    parameters = [
                        CommandParameter(parameter = "metric", value = velocity)
                    ]
                ),
                Command(
                    command = "steer",
                    parameters = [
                        CommandParameter(parameter = "rad", value = steering)
                    ]
                )
            ]
        ))


    ## Reconfigure ##
    def change_controller(self, controller_id):
        if controller_id in control_methods:
            self._controller = control_methods.get(controller_id)
            self.loginfo("Using controller '%d': %s" % (controller_id, ControlMethod(controller_id).name))
            return controller_id
        else:
            return self.P.method.value
