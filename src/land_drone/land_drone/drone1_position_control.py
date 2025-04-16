import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String, Float64  # type: ignore # This imports the Float64 message type

# from aiv_interfaces.msg import CPM  # Commented out CPM import
from land_drone.utils.manageFiles import get_name_robot

SEUIL_DISTANCE_OBSTACLE = 50


class DronePositionControl(Node):
    def __init__(self, hostname):
        super().__init__("drone1_position_control")

        self.hostname = hostname
        self.sonar_topic = f"{self.hostname}/sonar_data"

        self.subscription = self.create_subscription(
            Float64, self.sonar_topic, self.sonar_callback, 10
        )

        self.publisher = self.create_publisher(String, f"{self.hostname}/motor_cmd", 10)

        # self.publisher = self.create_publisher(CPM, 'cpm', 10)  # Commented out CPM publisher
        self.state = "Stop"
        self.distance = 0.0

        self.get_logger().info("Nœud drone1_position_control initialisé")

    def sonar_callback(self, msg):
        distance = msg.data
        self.distance = distance

        self.get_logger().info("Received sonar data: {} cm".format(distance))

        if distance > SEUIL_DISTANCE_OBSTACLE:
            self.state = "Forward"
        else:
            self.state = "Stop"

        self.publish_sonar_data(self.state)

        # cpm_msg = self.create_cpm_message(distance)  # Commented out CPM message creation
        # self.publish_cpm_message(cpm_msg)  # Commented out CPM message publishing

    def publish_sonar_data(self, state):
        msg = String()
        msg.data = state

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Published sonar data: {msg.data}")

    # def run_sonar_node(self):
    #     rclpy.spin(self.sonar_node)


def main(args=None):
    rclpy.init(args=args)

    hostname, _ = get_name_robot()

    drone_position_control = DronePositionControl(hostname)

    rclpy.spin(drone_position_control)

    drone_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
