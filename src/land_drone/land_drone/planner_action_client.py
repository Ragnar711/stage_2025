import sys
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from aiv_interfaces.action import PositionAction
from ament_index_python.packages import get_package_share_directory
from .utils import manageFiles

class PlannerActionClient(Node):
    def __init__(self, robot_name):
        super().__init__(robot_name + '_planner_action_client')
        
        # Initialise ROS clients
        luActionType = PositionAction
        lsActionName = robot_name + '/positionaction'
        self.action_client = ActionClient(self, luActionType, lsActionName)
        self.get_logger().info("Terrestrial drone planner action server has been initialized.")
        
        self.goal_positions = None
        self.current_position = None
        self.goal_distance = None
        self.goal_displayed = False  # Indicator to print goal only once

    def send_goal(self, afGoalPos):
        self.mfGoalPos = afGoalPos
        self.status = GoalStatus.STATUS_EXECUTING
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        positionAction_msg = PositionAction.Goal()
        positionAction_msg.goal_position = afGoalPos

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.action_client.send_goal_async(
            positionAction_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_handle):
        feedback_msg = feedback_handle.feedback
        current_position = feedback_msg.current_position
        distance = feedback_msg.distance 

        # Print goal position once
        if not self.goal_displayed:
            goal_position_str = 'Goal Position: [{:.3f}, {:.3f}, {:.3f}]'.format(self.mfGoalPos[0], self.mfGoalPos[1], self.mfGoalPos[2])
            self.get_logger().info(goal_position_str)
            self.goal_displayed = True

        # Print current position (latitude, longitude, altitude) and calculated distances
        current_position = 'Received feedback: Current Position: latitude = {:.3f}, longitude = {:.3f}, altitude = {:.3f}'.format(
            current_position[0], current_position[1], current_position[2]
        )
        self.get_logger().info(current_position)
        self.get_logger().info(f'Received feedback : Distance to goal : {distance:.2f} meters')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Final position: {0}'.format(result.final_position))
            self.status = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))


    def getStatus(self):
        return self.status

def main(args=None):
    rclpy.init(args=args)
    land_drone_directory = get_package_share_directory('land_drone')

    robot_name = "robot1"

    config_file = os.path.join(land_drone_directory, 'config.json')
    config = manageFiles.read_config_file(config_file)
    goals = config["experiences"]["missions"][robot_name]["goal_positions"]


    planner_action_client = PlannerActionClient(robot_name)

    for i in goals :
        planner_action_client.send_goal(goals[i])
        while planner_action_client.getStatus() != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(planner_action_client)

    planner_action_client.get_logger().info('All missions are done!')

    planner_action_client.destroy_node()


if __name__ == '__main__':
    main()
