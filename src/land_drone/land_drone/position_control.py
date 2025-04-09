import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from land_drone.GPSDrv import GPSDrv
from land_drone.Position3D import Position3D
from aiv_interfaces.msg import Vector2Goal, CAM, CPM
from rclpy.executors import MultiThreadedExecutor

ROTATE_FORWARD = 2000.

class PositionControl(Node):

    def __init__(self):
        super().__init__('position_control')

        self.robot_name = robot_name
        qos = QoSProfile(depth=10)

        # Initialise variables
        self.init_gps_state = False
        self.current_position = None
        self.goal_position = None

        # Initialise ROS Subscribers

        luMsgType = CAM
        lsTopicName = self.robot_name + '/CAM'
        self.create_subscription(luMsgType, lsTopicName, self.cam_response_callback, qos)

        luMsgType = CPM
        lsTopicName = self.robot_name + '/CPM'
        self.create_subscription(luMsgType, lsTopicName, self.cpm_response_callback, qos)

        luMsgType = Vector2Goal
        lsTopicName = self.robot_name + '/Vector2Goal'
        self.create_subscription(luMsgType, lsTopicName, self.Vector2Goal_response_callback, qos)

        # Initialise ROS2 Publishers

        luMsgType = Twist
        lsTopicName = self.robot_name + '/motor_cmd'
        self.muPubMotorCmd = self.create_publisher(luMsgType, lsTopicName, qos)

        self.get_logger().info("Terrestrial drone position control node has been initialised.")

        # Callback functions and relevant functions

    def Vector2Goal_response_callback(self, msg):
        lfDistanceToGo = msg.cur_pos.x - msg.goal_pos.x
        if lfDistanceToGo != 0:
            twist_msg = Twist()
            twist_msg.linear.x = ROTATE_FORWARD
            self.muPubMotorCmd.publish(twist_msg)
        self.get_logger().info(f"Received goal position: {msg.goal_pos}")

    def cam_response_callback(self, msg):
        self.cam_response = msg.cam_response
        self.get_logger().info("CAM message : {0} reeceived.".format(self.cam_response))

    def cpm_response_callback(self, msg):
        self.cpm_response = msg.cpm_response
        self.get_logger().info("CPM message : {0} reeceived.".format(self.cpm_response))

    def get_current_position(self):

        coordinates = self.gps_drv.get_coordinates()
        
        # GPS to Cartesian conversion
        latitude, longitude, altitude = map(float, coordinates.split(':')[1].split(','))
        return Position3D.fromGPScoordinates(latitude, longitude, altitude)

    def calculate_distance(self, curr_pos, goal_pos):
        dx = goal_pos.x - curr_pos.x
        dy = goal_pos.y - curr_pos.y
        dz = goal_pos.z - curr_pos.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def update_callback(self):
        if self.init_gps_state is True:
            self.control_loop()
            
        value, type = self.detect_obstacle()
        if type == "cpm": 
            self.send_CPM()


    def control_loop(self):
        if self.current_position is None or self.goal_position is None:
            return

        self.current_position = self.get_current_position()

        # Calculate the remaining distance
        distance = self.calculate_distance(self.current_position, self.goal_position)

        # Motor control based on remaining distance
        twist_msg = Twist()
        if distance > 1.0:  # Threshold to advance
            twist_msg.linear.x = 1.0  # Speed ​​to move forward
            self.motor_cmd_publisher.publish(twist_msg)
            self.get_logger().info(f"Moving forward. Distance to goal: {distance:.2f} meters")
        else:
            twist_msg.linear.x = 0.0  # Arrêt
            self.motor_cmd_publisher.publish(twist_msg)
            self.get_logger().info("Goal reached or too close, stopping.")


def main(args=None):
    rclpy.init(args=args)
    node = Node('position_control', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    robot_name = node.get_parameter('robot_name').value

    position_control = PositionControl(robot_name, wall_segments)


    executor = MultiThreadedExecutor()
    rclpy.spin(position_control, executor)


    position_control.get_logger().info('Destroy node {0}'.format(node))

    position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()