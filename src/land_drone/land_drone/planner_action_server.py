#!/usr/bin/env python3

import math
import rclpy  # type: ignore
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # type: ignore
from rclpy.callback_groups import ReentrantCallbackGroup  # type: ignore
from rclpy.executors import MultiThreadedExecutor  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from threading import Event, Lock

from aiv_interfaces.action import PositionAction
from land_drone.utils.or_event import OrEvent
from land_drone.GPSDrv import GPSDrv
from geometry_msgs.msg import Point  # type: ignore
from std_msgs.msg import String  # type: ignore

import numpy as np


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two points on the Earth.

    Parameters:
    lat1 (float): Latitude of the first point in degrees.
    lon1 (float): Longitude of the first point in degrees.
    lat2 (float): Latitude of the second point in degrees.
    lon2 (float): Longitude of the second point in degrees.

    Returns:
    float: Distance between the two points in meters.
    """

    R = 6371000  # Rayon de la Terre en mètres
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def gps_to_cartesian(lat, lon, alt):
    """
    Convert GPS coordinates to a Cartesian coordinate system.

    Parameters:
    lat (float): Latitude in degrees.
    lon (float): Longitude in degrees.
    alt (float): Altitude in meters.

    Returns:
    tuple[float, float, float]: Cartesian coordinates (x, y, z) in meters.
    """
    R = 6371000  # Rayon de la Terre en mètres
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    x = R * math.cos(lat_rad) * math.cos(lon_rad)
    y = R * math.cos(lat_rad) * math.sin(lon_rad)
    z = alt
    return x, y, z


class PlannerActionServer(Node):

    def __init__(self, robot_name):
        """
        Initialise the Terrestrial Drone Planner Action Server.

        This class represents the planner action server of the terrestrial drone.
        It is responsible for receiving and executing goals (i.e. target positions)
        sent by the action client.

        Parameters:
        robot_name (str): The name of the robot (used as a prefix for the node name).

        Attributes:
        _goalreached_event (Event): Event set when the goal has been reached.
        _abort_event (Event): Event set when the goal has been aborted.
        _lock (Lock): Lock used to protect access to the goal point.
        goal_point (tuple[float, float, float]): The target position (latitude, longitude,
            altitude) in degrees and meters.
        _tolerance (float): The tolerance (in meters) for considering a goal as reached.
        gps (GPSDrv): The GPS driver used to get the current position.
        current_lat (float): The current latitude in degrees.
        current_lon (float): The current longitude in degrees.
        tocontrol_publisher (Publisher[Point]): The publisher used to send the goal to the
            controller.
        action_server (ActionServer[PositionAction]): The action server used to receive and
            execute goals.
        """
        super().__init__(robot_name + "_planner_action_server")

        self._goalreached_event = Event()
        self._abort_event = Event()
        self._lock = Lock()
        self.goal_point = None
        self._tolerance = 0.5

        self.gps = GPSDrv()
        self.current_lat = None
        self.current_lon = None

        qos = QoSProfile(depth=10)
        self.tocontrol_publisher = self.create_publisher(
            Point, robot_name + "/goal", qos
        )

        self.action_server = ActionServer(
            self,
            PositionAction,
            robot_name + "/positionaction",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            "Terrestrial drone planner action server has been initialised."
        )

    def destroy(self):
        """
        Destroy the action server and the node.

        This method is used to properly shut down the action server and the node.
        It is recommended to call this method when the node is shutting down.
        """
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """
        This method is called when a goal is sent to the action server.

        The goal is stored in the `goal` attribute and the method returns a
        `GoalResponse.ACCEPT` message to indicate that the goal has been accepted.

        Parameters:
        goal_request (PositionAction.Goal): The goal request received from the action
            client.

        Returns:
        GoalResponse.ACCEPT: The goal has been accepted.
        """
        self.get_logger().info("Received goal request :)")
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        This method is called when the action client sends a cancel request.

        The method logs a message to indicate that the cancel request has been
        received and returns a `CancelResponse.ACCEPT` message to indicate that
        the cancel request has been accepted.
        """
        self.get_logger().info("Received cancel request :(")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        This method is called when a goal is received by the action server.

        The goal is stored in the `goal` attribute and the method executes the goal
        by sending the goal position to the controller. The method then waits until
        the robot has reached the goal position or the goal has been aborted.

        Parameters:
        goal_handle (GoalHandle[PositionAction]): The goal handle received from the
            action client.

        Returns:
        PositionAction.Result: The result of the goal execution.
        """
        self.feedback_msg = PositionAction.Feedback()

        if self.goal_point is not None:
            with self._lock:
                self._abort_event.set()
                self._abort_event = Event()

        self.goal_handle = goal_handle
        action_goal = goal_handle.request
        self.goal_lat = action_goal.goal_position[0]
        self.goal_lon = action_goal.goal_position[1]
        self.get_logger().info(
            f"Moving robot to goal position: Latitude {self.goal_lat}, Longitude {self.goal_lon}"
        )

        self.update_current_gps()

        goal_x, goal_y, goal_z = gps_to_cartesian(self.goal_lat, self.goal_lon, 0)
        current_x, current_y, current_z = gps_to_cartesian(
            self.current_lat, self.current_lon, 0
        )

        distance = math.sqrt(
            (goal_x - current_x) ** 2
            + (goal_y - current_y) ** 2
            + (goal_z - current_z) ** 2
        )
        self.get_logger().info(
            f"Current position: x = {current_x:.2f}, y = {current_y:.2f}, z = {current_z:.2f}"
        )
        self.get_logger().info(
            f"Converted goal Cartesian: x = {goal_x:.2f}, y = {goal_y:.2f}, z = {goal_z:.2f}"
        )
        self.get_logger().info(f"Distance to goal: {distance:.2f} meters")

        if distance <= self._tolerance:
            self._goalreached_event.set()
        else:
            self.get_logger().info("Distance to goal exceeds tolerance.")

        with self._lock:
            abort_event = self._abort_event
        OrEvent(self._goalreached_event, abort_event).wait()

        if self._goalreached_event.is_set():
            self.goal_point = None
            self._goalreached_event.clear()
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = PositionAction.Result()
        result.final_position = [float(self.current_lat), float(self.current_lon), 0.0]
        self.get_logger().info(
            f"Finished at position: Latitude {result.final_position[0]}, Longitude {result.final_position[1]}"
        )

        return result

    def update_current_gps(self):
        """
        Updates the current GPS position.

        Gets the current GPS coordinates from the GPS driver, and if successful,
        updates the current latitude and longitude. Logs an info message with the
        updated position if successful, or a warning message if not.
        """
        coordinates = self.gps.get_coordinates()
        if coordinates:
            lat_str, lon_str, _ = coordinates.split(",")
            self.current_lat = float(lat_str.split(":")[1].strip())
            self.current_lon = float(lon_str.split(":")[1].strip())
            self.get_logger().info(
                f"Updated GPS position: Latitude {self.current_lat}, Longitude {self.current_lon}"
            )
        else:
            self.get_logger().warning("Failed to get current GPS position.")


def main(args=None):
    """
    Main entry point for the Terrestrial Drone Planner Action Server.

    This function initializes the ROS client library, creates a PlannerActionServer
    node, and spins it using a MultiThreadedExecutor. The node is destroyed when
    the main thread exits.

    Parameters:
    args (list): Command line arguments passed to the node. If not provided, it
        defaults to sys.argv.

    Returns:
    None
    """
    rclpy.init(args=args)

    # node = Node(
    #     "planner_action_server",
    #     allow_undeclared_parameters=True,
    #     automatically_declare_parameters_from_overrides=True,
    # )
    # robot_name = node.get_parameter("robot_name").value

    robot_name = "robot1"  # Change to your robot name
    planner_action_server = PlannerActionServer(robot_name)

    executor = MultiThreadedExecutor()
    rclpy.spin(planner_action_server, executor)

    planner_action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
