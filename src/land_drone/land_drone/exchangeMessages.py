import re
import math
import json                         
import rclpy                        
from rclpy.node import Node         
from aiv_interfaces.msg import CAM, CPM, PerceiveObjectContainer 
import socket
from std_msgs.msg import Float64, String, Float64MultiArray

# Define a class for exchanging messages
class ExchangeMessages(Node):
    def __init__(self, node_name, hostname, neighbors):
        super().__init__(node_name)

        # Initialize the ExchangeMessages node with a specified name and neighbors
        self.hostname = hostname
        self.neighbors = neighbors
        
        # Test values by default
        self.current_gps_position = [0.0, 0.0, 0.0]
        self.current_gyro_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_sonar_distance = 100.0

        # Create a publisher for sending CAM messages
        self.CAM_publisher = self.create_publisher(CAM, f"/{self.hostname}/CAM", 10)
        
        # Create a publisher for sending CPM messages
        self.CPM_publisher = self.create_publisher(CPM, f"/{self.hostname}/CPM", 10)

        # Create a timer to periodically publish messages
        self.timer = self.create_timer(1.0, self.publish_message)

        # Create a list to store subscription objects
        self.subscription_ = []

        # Subscribe to the CAM messages published by the neighbors
        # self.subscribe_to_neighbor()

        self.subscription_sonar = self.create_subscription(
            Float64,
            f'{self.hostname}/sonar_data',
            self.sonar_data_callback,
            10
        )

        self.subscription_gps = self.create_subscription(
            String,
            f'{self.hostname}/gps_data',
            self.gps_data_callback,
            10
        )

        self.subscription_gyro = self.create_subscription(
            Float64MultiArray,
            f'{self.hostname}/gyro_data',
            self.gyro_data_callback,
            10
        )

    def subscribe_to_neighbor(self):
        # Create a subscription for each neighbor's CAM topic
        for neighbor in self.neighbors:
            if neighbor != self.hostname:
                # Subscribe to the CAM messages published by the current neighbor
                subscription = self.create_subscription(
                    CAM,
                    f"/{neighbor}/CAM",
                    self.receive_callback,
                    10
                )
                # Add the subscription object to the list
                self.subscription_.append(subscription)


    def publish_message(self):
        # Create a new CAM message
        CAM_msg = CAM()
        CAM_msg.current_position = self.current_gps_position

        # Publish the CAM message
        self.CAM_publisher.publish(CAM_msg)
        
        # Create a new CPM message
        CPM_msg = CPM()
        CPM_msg.perceive_object = PerceiveObjectContainer()
        CPM_msg.perceive_object.distance = self.current_sonar_distance

        # Publish the CPM message
        self.CPM_publisher.publish(CPM_msg)    # def publish_message(self):


    def receive_callback(self, msg):
        # Process the received CAM message
        self.get_logger().info(f"Received message from {self.neighbors[0]}/CAM: {msg.current_position}")

    def sonar_data_callback(self,msg):
        distance = msg.data
        self.get_logger().info('Received sonar data: {} cm'.format(distance))
        self.current_sonar_distance = distance

    def gps_data_callback(self,msg):
        coordonne_gps = msg.data 
        lsGPSCoordinates = str(coordonne_gps)
        lfLatitude, lfLongitude, lfAltitude = self.extract_coordinates(lsGPSCoordinates)
        self.get_logger().info(f"Received gps data : Latitude: {lfLatitude}, Longitude: {lfLongitude}, Altitude: {lfAltitude} meters")
        self.current_gps_position[0] = lfLatitude
        self.current_gps_position[1] = lfLongitude
        self.current_gps_position[2] = lfAltitude
        
    
    def gyro_data_callback(self,msg):
        coordonne_gyro = msg.data
        self.get_logger().info('Received gyro data: {}'.format(coordonne_gyro))
        if len(coordonne_gyro) == 6:
            x, y, z, ax, ay, az = coordonne_gyro
            self.current_gyro_position = [x, y, z, ax, ay, az]
        else:
            self.get_logger().warn('Received invalid gyro data: {}'.format(coordonne_gyro))


def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    node = Node('exchange_messages', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    
    hostname = node.get_parameter('hostname').value
    neighbors = node.get_parameter('neighbors').value

    hostname = "robot1"
    node_name = f'{hostname}_exchange_messages'

    node_exchange_messages = ExchangeMessages(node_name,hostname, neighbors)
    rclpy.spin(node_exchange_messages)
    rclpy.shutdown(node_exchange_messages )
    
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
