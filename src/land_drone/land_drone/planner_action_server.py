
#!/usr/bin/env python3

import math
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Event, Lock

from aiv_interfaces.action import PositionAction
from land_drone.utils.or_event import OrEvent
from geometry_msgs.msg import Point
from std_msgs.msg import String
from land_drone.GPSDrv import GPSDrv

import numpy as np
from rclpy.executors import MultiThreadedExecutor

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Rayon de la Terre en mètres
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def gps_to_cartesian(lat, lon, alt):
    R = 6371000  # Rayon de la Terre en mètres
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    x = R * math.cos(lat_rad) * math.cos(lon_rad)
    y = R * math.cos(lat_rad) * math.sin(lon_rad)
    z = alt
    return x, y, z

class PlannerActionServer(Node):

    def __init__(self, robot_name):
        super().__init__(robot_name + '_planner_action_server')

        self._goalreached_event = Event()
        self._abort_event = Event()
        self._lock = Lock()
        self.goal_point = None
        self._tolerance = 0.5

        self.gps = GPSDrv()
        self.current_lat = None
        self.current_lon = None

        qos = QoSProfile(depth=10)
        self.tocontrol_publisher = self.create_publisher(Point, robot_name + '/goal', qos)

        self.action_server = ActionServer(
            self,
            PositionAction, robot_name + '/positionaction',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Terrestrial drone planner action server has been initialised.")

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.feedback_msg = PositionAction.Feedback()

        if self.goal_point is not None:
            with self._lock:
                self._abort_event.set()
                self._abort_event = Event()
        
        self.goal_handle = goal_handle
        action_goal = goal_handle.request
        self.goal_lat = action_goal.goal_position[0]
        self.goal_lon = action_goal.goal_position[1]
        self.get_logger().info(f'Moving robot to goal position: Latitude {self.goal_lat}, Longitude {self.goal_lon}')

        self.update_current_gps()

        goal_x, goal_y, goal_z = gps_to_cartesian(self.goal_lat, self.goal_lon, 0)
        current_x, current_y, current_z = gps_to_cartesian(self.current_lat, self.current_lon, 0)

        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2 + (goal_z - current_z) ** 2)
        self.get_logger().info(f'Current position: x = {current_x:.2f}, y = {current_y:.2f}, z = {current_z:.2f}')
        self.get_logger().info(f'Converted goal Cartesian: x = {goal_x:.2f}, y = {goal_y:.2f}, z = {goal_z:.2f}')
        self.get_logger().info(f'Distance to goal: {distance:.2f} meters')

        if distance <= self._tolerance:
            self._goalreached_event.set()
        else:
            self.get_logger().info('Distance to goal exceeds tolerance.')

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
        self.get_logger().info(f'Finished at position: Latitude {result.final_position[0]}, Longitude {result.final_position[1]}')

        return result

    def update_current_gps(self):
        coordinates = self.gps.get_coordinates()
        if coordinates:
            lat_str, lon_str, _ = coordinates.split(',')
            self.current_lat = float(lat_str.split(':')[1].strip())
            self.current_lon = float(lon_str.split(':')[1].strip())
            self.get_logger().info(f'Updated GPS position: Latitude {self.current_lat}, Longitude {self.current_lon}')
        else:
            self.get_logger().warning('Failed to get current GPS position.')


def main(args=None):
    rclpy.init(args=args)

    node = Node('planner_action_server', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    robot_name = node.get_parameter('robot_name').value

    robot_name = "robot1"  # Change to your robot name
    planner_action_server = PlannerActionServer(robot_name)

    executor = MultiThreadedExecutor()
    rclpy.spin(planner_action_server, executor)

    planner_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
