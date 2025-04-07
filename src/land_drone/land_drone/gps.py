import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from land_drone.GPSDrv import GPSDrv
from land_drone.utils.manageFiles import get_name_robot

class GPS(Node):
    def __init__(self, node_name, hostname, auGPSDrv):
        super().__init__(node_name)

        self.hostname = hostname
        self.topic_name = f'{self.hostname}/gps_data'
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.muGPSDrv = auGPSDrv
        self.get_logger().info(f'GPS node initialised with topic: {self.topic_name}')

    def publish_message(self):
        coordinates = self.muGPSDrv.get_coordinates()
        self.get_logger().info(f"Coordinates: {coordinates}")
        self.publish_gps_message(coordinates)
    
    def publish_gps_message(self, coordinates):
        msg = self.create_string_message(coordinates)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Message published on the topic {self.topic_name}: {coordinates}')

    def create_string_message(self, coordinates):
        msg = String()
        msg.data = coordinates
        return msg

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('gps', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

    hostname = node.get_parameter('hostname').get_parameter_value().string_value
    if hostname is None:
        hostname, _ = get_name_robot()  # get hostname from the function

    hostname = "robot1"
    luGPSDrv = GPSDrv()
    gps_node = GPS(f'{hostname}_gps', hostname, luGPSDrv)
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
