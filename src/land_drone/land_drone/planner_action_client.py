import sys
import termios
import rclpy
import os

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from aiv_interfaces.action import PositionAction
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from land_drone.utils.manageFiles import read_config_file
from .utils import manageFiles

terminal_msg = """
Terretrial Drone Planner
------------------------------------------------------
goal_position: float64[]
------------------------------------------------------
"""

class PlannerActionClient(Node):

    def __init__(self, robot_name):
        super().__init__(robot_name + '_planner_action_client')

        self.action_client = ActionClient(self, PositionAction, robot_name + '/positionaction')

        self.get_logger().info("Terrestrial drone planner action client node has been initialised.")

    def send_goal(self, goal_position):
        self.status = GoalStatus.STATUS_EXECUTING
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        positionAction_msg = PositionAction.Goal()
        positionAction_msg.goal_position = goal_position

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.action_client.send_goal_async(
            positionAction_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(
            'Time left until the robot stops: {0}'.format(feedback.feedback.left_time))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.final_position))
            self.status = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # rclpy.shutdown()

    def getStatus(self):
        return self.status

def main(args=None):
    rclpy.init(args=args)
    land_drone_directory = get_package_share_directory('land_drone')

    node = Node('planner_action_client', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    robot_name = node.get_parameter('robot_name').value

    config_file = os.path.join(land_drone_directory, 'config.json')
    
    if os.path.exists(config_file):
        config = read_config_file(config_file)
        if config:
            robot_name = "robot1"  # Default to robot 1
                
            if robot_name in config["experiences"]["missions"]:
                goals = config["experiences"]["missions"][robot_name].get("goal_positions", {})
                    
    #             if "1" in goals:
    #                 first_goal = goals["1"]
    #                 print(f"{robot_name}: Goal 1: {first_goal}")
    #             else:
    #                 print(f"Goal 1 n'existe pas pour le robot {robot_name}.")
    #         else:
    #             print(f"Le robot {robot_name} n'existe pas dans la configuration.")
    #     else:
    #         print(f"La configuration dans {config_file} n'a pas été chargée correctement.")
    # else:
    #     print(f"Le fichier {config_file} n'existe pas.")

    planner_action_client = PlannerActionClient(robot_name)

    for i in goals :
        planner_action_client.send_goal(goals[i])
        while planner_action_client.getStatus() != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(planner_action_client)

    planner_action_client.get_logger().info('All missions are done!')
    
    planner_action_client.destroy_node()


if __name__ == '__main__':
    main()
