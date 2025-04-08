import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64  # type: ignore
from land_drone.sonarDrv import SonarDrv
from land_drone.utils.manageFiles import get_name_robot

THRESHOLD_OBSTACLE_DISTANCE = 10  # cm


class Sonar(Node):
    def __init__(self, node_name, hostname, auSonarDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.topic_name = f"{self.hostname}/sonar_data"
        self.publisher_ = self.create_publisher(Float64, self.topic_name, 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.muSonarDrv = auSonarDrv
        self.detected_obstacle = False
        self.get_logger().info(
            "Sonar node initialised with the topic: {}".format(self.topic_name)
        )

    def publish_message(self):
        distance = self.muSonarDrv.get_distance()
        self.update_obstacle_detection(distance)
        self.publish_distance_message(distance)

    def update_obstacle_detection(self, distance):
        if distance < THRESHOLD_OBSTACLE_DISTANCE:
            self.detected_obstacle = True
        else:
            self.detected_obstacle = False

    def publish_distance_message(self, distance):
        self.get_logger().info("Distance: {} cm".format(distance))
        msg = self.create_float_message(distance)
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Message published on the topic {}: "{}"'.format(self.topic_name, distance)
        )

    def create_float_message(self, distance):
        msg = Float64()
        msg.data = distance
        return msg

    def is_detected_obstacle(self):
        return self.detected_obstacle


def main(args=None):
    rclpy.init(args=args)

    node = Node(
        "sonar",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    hostname = node.get_parameter("hostname").value  # get hostname from the launcher
    if hostname == None:
        hostname, _ = (
            get_name_robot()
        )  # get hostname from the function, because the node is running independently without the launcher

    luSonarDrv = SonarDrv(SonarDrv.TRIG_PIN, SonarDrv.ECHO_PIN)

    sonar_node = Sonar(f"{hostname}_sonar", hostname, luSonarDrv)
    rclpy.spin(sonar_node)
    sonar_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
