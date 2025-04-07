from launch import LaunchService
from launch_ros.actions import Node
from launch import LaunchDescription

from land_drone.utils.manageFiles import get_name_robot

def generate_launch_description():
    # Creation of launch actions for each drone and its neighbours
    actions = []

    drone_name, neighbors = get_name_robot()

    #  Configuration du Node Sonar
    sonar_node = Node(
        package='land_drone',
        executable='sonar',
        parameters=[{'hostname': drone_name}]
    )
    actions.append(sonar_node)

    # Configuration du Node Motor 
    motor_node = Node(
        package='land_drone',
        executable='motor',
        parameters=[{'hostname': drone_name}]
    )
    actions.append(motor_node)

    """
    # Configuration du Node Gyro 
    gyro_node = Node(
        package='land_drone',
        executable='gyro',
        parameters=[{'hostname': drone_name}]
    )
    actions.append(gyro_node)

    # Configuration du Node GPS
    gps_node = Node(
        package='land_drone',
        executable='gps',
        parameters=[{'hostname': drone_name}]
    )
    actions.append(gps_node)
    """

    return LaunchDescription(actions)
