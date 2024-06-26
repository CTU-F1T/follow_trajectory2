#!/usr/bin/env python
# _run.py
"""Node for following a planned trajectory of the car.

Creates a ROS node with all required callbacks and publishers.
"""
######################
# Imports & Globals
######################

from autopsy.reconfigure import ParameterServer
from autopsy.node import Node, ROS_VERSION
from autopsy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from ._path import Path as PathO
from ._trajectory import Trajectory
from ._utils import (
    determine_side,
)
from ._vehicle import Vehicle

import logging

if ROS_VERSION == 1:
    import rospy

    def update_parameters(p):
        """Update the node parameters from the Parameter server."""
        if rospy.has_param("~"):
            p.update(rospy.get_param("~"), only_existing = True)
else:
    def update_parameters(p):
        """Fake the update of the parameters."""
        pass


# Message Types
from autoware_auto_msgs.msg import Trajectory as TrajectoryA
from command_msgs.msg import (
    CommandArrayStamped, Command, CommandParameter
)
from geometry_msgs.msg import (
    Point,
    PointStamped,
)
from nav_msgs.msg import (
    Odometry,
    Path,
)
from std_msgs.msg import (
    Bool,
    Header,
    String,
)


try:
    from vesc_msgs.msg import VescStateStamped
    USE_VESC = True
except ImportError:
    print ("Unable to import 'vesc_msgs.msg'. VESC callback will be disabled.")
    USE_VESC = False


# Dynamic module list of Controllers
from enum import Enum

import follow_trajectory2.controllers as controllers

control_methods = {}


class ControlMethod(Enum):
    """Enum to store the available control methods."""  # noqa: D204
    pass


for module in controllers.__all__:
    _ctrl = __import__(
        "follow_trajectory2.controllers." + module,
        fromlist=["Controller", "name"]
    )
    _d = {cm.name: cm.value for cm in ControlMethod}
    _d.update({_ctrl.name: _ctrl.value})
    ControlMethod = Enum("ControlMethod", _d)
    control_methods[_ctrl.value] = _ctrl.Controller


# Global parameters
PARAMETERS = [
    ("method", ControlMethod),

    # # Common variables
    # Acceleration limits
    ("max_fwd_acc", {
        "default": 6.0, "min": 0.0, "max": 10.0,
        "description": "Maximum forward acceleration [m.s^-2]",
        "ns": "default"
    }),
    ("max_brk_dcc", {
        "default": -6.0, "min": -10.0, "max": 0.0,
        "description": "Maximum backward deceleration [m.s^-2]",
        "ns": "default"
    }),

    # # Others
    ("use_odom_speed", {
        "default": False,
        "description": "When True, speed from the Odometry message "
                       "is used for the Vehicle speed."
    }),
]


######################
# RunNode
######################

class RunNode(Node):
    """Class for creating a ROS node."""

    def __init__(self):
        """Initialize the ROS node."""
        super(Node, self).__init__("follow_trajectory2")

        logging.basicConfig(
            filename="follow_trajectory2.log",
            filemode='a',
            format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
            datefmt='%H:%M:%S',
            level=logging.DEBUG
        )

        logging.info("FollowTrajectory2")

        self.logger = logging.getLogger('followtrajectory')


        # Internal variables
        self.saved_trajectory = None
        self.Vehicle = Vehicle()
        self.running = False
        self.time_last = self.get_time()
        self.time_elapsed = 0.0
        self.passed_half = False
        self.pub_track = 0

        # Intialize controllers
        for key, ctrl in control_methods.items():
            control_methods[key] = ctrl(vehicle = self.Vehicle, node = self)


        self.P = ParameterServer()

        self.P.update(PARAMETERS)

        update_parameters(self.P)

        self.P.reconfigure(node = self)


        self._controller = control_methods.get(self.P.method.value)
        self.loginfo(
            "Using controller '%d': %s"
            % (self.P.method.value, ControlMethod(self.P.method.value).name)
        )
        self.P.method.callback = self.change_controller


        # Publishers
        self.pub_command = self.Publisher(
            "/command", CommandArrayStamped, queue_size = 1
        )
        self.traj_point = self.Publisher(
            "/trajectory/current_point", PointStamped, queue_size = 1
        )
        self.back_axle = self.Publisher(
            "/trajectory/back_axle", PointStamped, queue_size = 1
        )
        self.traj_point_str = self.Publisher(
            "/trajectory/current_point/string", String, queue_size = 1
        )
        self.pub_track_error = self.Publisher(
            "/track_error", String, queue_size = 1
        )


        # Subscribers
        # Odometry uses some tweaks available only in ROS1.
        if ROS_VERSION == 1:
            rospy.Subscriber(
                "/odom", Odometry, self.callback_odom,
                queue_size = 1, buff_size = 1400, tcp_nodelay = False
            )
        else:  # ROS_VERSION == 2
            self.create_subscription(
                Odometry, "/odom", self.callback_odom,
                qos_profile = QoSProfile(
                    depth = 1,
                    durability = DurabilityPolicy.VOLATILE,
                    reliability = ReliabilityPolicy.BEST_EFFORT
                )
            )

        self.Subscriber("/path", Path, self.callback_path)
        self.create_subscription(
            TrajectoryA, "/trajectory", self.callback_trajectory,
            qos_profile = QoSProfile(
                depth = 1, durability = DurabilityPolicy.TRANSIENT_LOCAL
            )
        )
        self.create_subscription(
            Bool, "/eStop", self.callback_estop,
            qos_profile = QoSProfile(
                depth = 1, reliability = ReliabilityPolicy.BEST_EFFORT
            )
        )

        if USE_VESC:
            if ROS_VERSION == 1:
                self.Subscriber(
                    "/sensors/core", VescStateStamped, self.callback_vesc
                )
            else:  # ROS_VERSION == 2
                self.create_subscription(
                    VescStateStamped, "/sensors/core", self.callback_vesc,
                    qos_profile = QoSProfile(
                        depth = 1,
                        durability = DurabilityPolicy.VOLATILE,
                        reliability = ReliabilityPolicy.BEST_EFFORT
                    )
                )

    #
    # # Utils # #
    def get_time(self):
        """Obtain the current ROS time as float."""
        if ROS_VERSION == 1:
            return rospy.get_time()
        else:  # ROS_VERSION == 2
            # https://github.com/ros2/rclpy/issues/293
            # 1e9 is float in Python 3, that leads to floating point error
            # on division.
            return self.get_clock().now().nanoseconds / (10 ** 9)

    def get_time_now(self):
        """Obtain the current ROS timestamp."""
        if ROS_VERSION == 1:
            return rospy.Time.now()
        else:  # ROS_VERSION == 2
            return self.get_clock().now().to_msg()

    #
    # # Callbacks # #
    def callback_path(self, data):
        """Handle the Path message."""
        self.saved_trajectory = PathO(data, self.Vehicle)
        self.loginfo("Received path.")


    def callback_trajectory(self, data):
        """Store a Trajectory.

        Arguments:
        data -- autoware_auto_msgs/Trajectory
        """
        self.saved_trajectory = Trajectory(data, self.Vehicle)
        self.saved_errors = [[] for _ in range(self.saved_trajectory.length)]
        self.loginfo("Received trajectory.")
        self._controller.process_trajectory(self, self.saved_trajectory)


    def callback_estop(self, data):
        """Handle `auto start/stop`. Reset the Vehicle states.

        Arguments:
        data -- std_msgs/Bool
        """
        self.loginfo(
            "Using controller '%d': %s"
            % (self.P.method.value, ControlMethod(self.P.method.value).name)
        )
        self.Vehicle.stop()
        self.time_last = self.get_time()
        self.running = not data.data

        if self.running:
            self.time_elapsed = 0.0


    def callback_vesc(self, data):
        """Obtain current speed of the vehicle from VESC.

        Arguments:
        data -- vesc_msgs/VescStateStamped
        """
        self.Vehicle.v = data.state.speed / 4105.324277107


    def callback_odom(self, data):
        """Handle the Odometry message.

        Arguments:
        data -- nav_msgs/Odometry
        """
        if not self.running:
            return


        # Update states
        self.Vehicle.position = data.pose.pose.position
        self.Vehicle.orientation = data.pose.pose.orientation
        self.Vehicle.dt = self.get_time() - self.time_last

        if self.P.use_odom_speed:
            self.Vehicle.v = data.twist.twist.linear.x

        self.time_elapsed += self.Vehicle.dt
        self.time_last = self.get_time()

        #
        if self.saved_trajectory is not None:

            # # ... and now we continue in case that everything is OK.
            nearest_point_id, point = self._controller.select_point(
                self, self.saved_trajectory
            )

            if self.passed_half and nearest_point_id < 50:
                self.passed_half = False

                # Publish the track error only when there are not many nans
                if (
                    sum([1 for el in self.saved_errors if len(el) == 0])
                    < (self.saved_trajectory.length * 0.05)
                ):

                    # # New version, publish minimum of errors
                    mins = []

                    for i in range(self.saved_trajectory.length):
                        # Minimum is the one closest to zero (from both sides!)
                        if len(self.saved_errors[i]) < 1:
                            mins.append("0.0")
                        else:
                            mins.append(
                                "%s" % min(self.saved_errors[i], key = abs)
                            )

                    self.logger.info(",".join(mins))

                    # Skip the first loop
                    if self.pub_track > 1:
                        self.pub_track_error.publish(
                            String(data = ",".join(mins))
                        )

                    self.pub_track += 1
                    self.saved_errors = [
                        [] for _ in range(self.saved_trajectory.length)
                    ]

            if not self.passed_half and nearest_point_id > 200:
                self.passed_half = True

            # # Visualize the trajectory point + back axle position.
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

            self.saved_errors[nearest_point_id].append(
                self.saved_trajectory.lateral_distance(self.Vehicle)
                * determine_side(
                    self.saved_trajectory.get(nearest_point_id),
                    self.saved_trajectory.get(nearest_point_id + 1),
                    self.Vehicle
                )
            )

            # # Obtain action values
            _velocity, _steer = self._controller.compute(
                self, point, self.saved_trajectory, nearest_point_id
            )

            self.publish_action(_velocity, _steer)

    #
    # # Publishers # #
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
                        CommandParameter(
                            parameter = "metric", value = velocity
                        )
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

    #
    # # Reconfigure # #
    def change_controller(self, controller_id):
        """Handle the request to change the controller."""
        if controller_id in control_methods:
            self._controller = control_methods.get(controller_id)
            self.loginfo(
                "Using controller '%d': %s"
                % (controller_id, ControlMethod(controller_id).name)
            )
            return controller_id
        else:
            return self.P.method.value
