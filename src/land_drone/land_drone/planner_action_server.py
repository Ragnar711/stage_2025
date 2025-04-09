import time
import rclpy
import math

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from aiv_interfaces.action import PositionAction
from aiv_interfaces.msg import Vector2Goal
from geometry_msgs.msg import Point
from land_drone.GPSDrv import GPSDrv
from land_drone.Position3D import Position3D
from rclpy.executors import MultiThreadedExecutor

class PlannerActionServer(Node):

    def __init__(self, robot_name):
        super().__init__(robot_name + '_planner_action_server')

        luMsgType = Vector2Goal
        self.msTopicName = f'{robot_name}/vector_to_goal' 
        luQos = QoSProfile(depth=10)

        self.muPubGoalPos = self.create_publisher(
            luMsgType, 
            self.msTopicName, 
            luQos
        )

        self.action_server = ActionServer(
            self,
            PositionAction,
            robot_name + '/positionaction',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Terrestrial drone planner action server has been initialized.")

        self.luGPSDrv = GPSDrv()

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

    def extract_coordinates(self, gps_coordinates):
        parts = gps_coordinates.split(', ')
        latitude = None
        longitude = None
        altitude = None
        
        for part in parts:
            if part.startswith('Latitude:'):
                latitude_str = part.split(': ')[1]
                latitude = float(latitude_str)
            elif part.startswith('Longitude:'):
                longitude_str = part.split(': ')[1]
                longitude = float(longitude_str)
            elif part.startswith('Altitude:'):
                altitude_str = part.split(': ')[1].split(' ')[0]
                altitude = float(altitude_str)
        
        if latitude is None or longitude is None or altitude is None:
            raise ValueError('Failed to extract valid coordinates')

        return latitude, longitude, altitude

    def calculate_distance(self, CurPos, GoalPos):
        dx = GoalPos.x - CurPos.x
        dy = GoalPos.y - CurPos.y
        distance = math.sqrt(dx * dx + dy * dy)
        distance = distance - 9.0
        return distance

    async def execute_callback(self, goal_handle):
        goal_position = goal_handle.request.goal_position
        luCartesianGoalPos = Position3D(goal_position[0], goal_position[1], goal_position[2])
        self.get_logger().info('Executing goal to move until: {:.3f}, {:.3f}, {:.3f}'.format(*goal_position))

        feedback_msg = PositionAction.Feedback()

        while True:

            lsGPSCoordinates = self.luGPSDrv.get_coordinates()  # Assuming this gets the current position
            lfLatitude, lfLongitude, lfAltitude = self.extract_coordinates(lsGPSCoordinates)

            # # Simulated GPS coordinates for current position
            # lfLatitude = 48.047
            # lfLongitude = -1.744
            # lfAltitude = 27.617

            luCartesianCurPos = Position3D.fromGPScoordinates(lfLatitude, lfLongitude, lfAltitude)
            
            # Definition of the feedback msg 
            luMsg = Vector2Goal()
            luMsg.cur_pos.x = luCartesianCurPos.x
            luMsg.cur_pos.y = luCartesianCurPos.y
            luMsg.cur_pos.z = luCartesianCurPos.z
            luMsg.goal_pos.x = luCartesianGoalPos.x
            luMsg.goal_pos.y = luCartesianGoalPos.y
            luMsg.goal_pos.z = luCartesianGoalPos.z
      
            # Publication of the feedback msg 
            self.muPubGoalPos.publish(luMsg)
            self.get_logger().info(f"Cartesian current position : {luCartesianCurPos.toString()}")

            # Calculate the distances to the goal for latitude, longitude, and altitude
            distance = self.calculate_distance(luCartesianCurPos,luCartesianGoalPos)

            # Send feedback to the client
            feedback_msg.current_position = [luCartesianCurPos.x, luCartesianCurPos.y, luCartesianCurPos.z]
            feedback_msg.distance = distance

            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Feedback sent to client: Current Position: latitude = {feedback_msg.current_position[0]:.3f}, longitude = {feedback_msg.current_position[1]:.3f}, altitude = {feedback_msg.current_position[2]:.3f}')
            self.get_logger().info(f'Distance to goal: {distance:.2f} meters')
            
            time.sleep(0.5)  # Simulate work being done

            # Check if the goal is reached or if a cancel request was made
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return PositionAction.Result()

            # Here you might want to check if the distance is small enough to consider the goal reached
            if distance < 0.1 :   # You can adjust this threshold as needed
                goal_handle.succeed()
                result_msg = PositionAction.Result()
                result_msg.final_position = [luCartesianGoalPos.x, luCartesianGoalPos.y, luCartesianGoalPos.z]
                self.get_logger().info(f'Goal succeeded! Final position : [{result_msg.final_position[0]:.3f}, {result_msg.final_position[1]:.3f}, {result_msg.final_position[2]:.3f}]')
                return result_msg

def main(args=None):
    rclpy.init(args=args)

    robot_name = "robot1" 
    planner_action_server = PlannerActionServer(robot_name)

    executor = MultiThreadedExecutor()
    rclpy.spin(planner_action_server, executor)

    planner_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
