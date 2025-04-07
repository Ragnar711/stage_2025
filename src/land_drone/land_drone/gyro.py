import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from land_drone.gyroDrv import GyroDrv
import json
import socket  

from land_drone.utils.manageFiles import get_name_robot

class Gyro(Node):
    def __init__(self, node_name, hostname, auGyroDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.topic_name = f'{self.hostname}/gyro_data'
        self.publisher_ = self.create_publisher(Float64MultiArray, self.topic_name, 10) 
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.muGyroDrv = auGyroDrv
        self.get_logger().info('Nœud gyro initialisé avec le topic : {}'.format(self.topic_name))

    def publish_message(self):
        x, y, z = self.muGyroDrv.get_gyro_data()
        ax, ay, az = self.muGyroDrv.get_accel_data()
        self.publish_gyro_message(x, y, z, ax, ay, az)
        
    def publish_gyro_message(self, x, y, z, ax, ay, az):
        data_dict = {
            "Gyro X": x,
            "Gyro Y": y,
            "Gyro Z": z,
            "Accel X": ax,
            "Accel Y": ay,
            "Accel Z": az
        }
        gyro_msg = Float64MultiArray()
        gyro_msg.data = [float(value) for value in json.dumps(data_dict).encode('utf-8')]
        self.publisher_.publish(gyro_msg)
        self.get_logger().info('Message publié sur le topic {} : {}'.format(self.topic_name, data_dict))


def main(args=None):
    rclpy.init(args=args)
    node = Node('gyro', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    
    hostname = node.get_parameter('hostname').value # get hostname from the launcher
    if hostname == None:
        hostname, neighbors = get_name_robot() # get hostname from the function, because the node is running independently without the launcher

    luGyroDrv = GyroDrv()
    gyro_node = Gyro(f'{hostname}_gyro', hostname, luGyroDrv)
    rclpy.spin(gyro_node)
    gyro_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
