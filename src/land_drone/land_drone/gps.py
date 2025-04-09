import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64  # type: ignore
from std_msgs.msg import String  # type: ignore
from land_drone.GPSDrv import GPSDrv
import time

from land_drone.utils.manageFiles import get_name_robot


class GPS(Node):
    def __init__(self, node_name, hostname, auGPSDrv):
        super().__init__(node_name)

        self.hostname = hostname
        self.topic_name = f"{self.hostname}/gps_data"
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.muGPSDrv = auGPSDrv
        self.get_logger().info(
            "GPS node initialised with topic: {}".format(self.topic_name)
        )

    def publish_message(self):
        coordinates = self.muGPSDrv.get_coordinates()
        print("coordinates: {}".format(coordinates))
        self.publish_gps_message(coordinates)
        time.sleep(2)

    def publish_gps_message(self, coordinates):
        self.get_logger().info("Coordinates: {}".format(coordinates))
        msg = self.create_string_message(coordinates)
        self.publisher_.publish(msg)
        self.get_logger().info(
            "Message published on the topic {}: {}".format(self.topic_name, coordinates)
        )

    def create_string_message(self, coordinates):
        msg = String()
        msg.data = str(coordinates)
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = Node(
        "gps",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    hostname = node.get_parameter("hostname").value  # get hostname from the launcher
    if hostname == None:
        hostname, _ = (
            get_name_robot()
        )  # get hostname from the function, because the node is running independently without the launcher

    hostname = "robot1"
    luGPSDrv = GPSDrv()
    gps_node = GPS(f"{hostname}_gps", hostname, luGPSDrv)
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
