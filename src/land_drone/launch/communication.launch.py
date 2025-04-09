# Fichier communication.launch.py
import os
import json
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from land_drone.utils.manageFiles import get_name_robot


def generate_launch_description():
    # Creation of launch actions for each drone and its neighbours
    actions = []

    drone_name, neighbors = get_name_robot()
    
    # Creation of the launch action for the ExchangeMessages node
    actions.append(Node(
            package='land_drone',
            executable='exchangeMessages',
            parameters=[{'hostname': drone_name, 'neighbors': neighbors}]
        ))
    print(drone_name)
    print(neighbors)
    return LaunchDescription(actions)