import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64  # type: ignore # This imports the Float64 message type

# from aiv_interfaces.msg import CPM  # Commented out CPM import
from land_drone.utils.manageFiles import get_name_robot

SEUIL_DISTANCE_OBSTACLE = 10


class DronePositionControl(Node):
    def __init__(self, hostname):
        super().__init__("drone1_position_control")

        self.hostname = hostname
        self.sonar_topic = f"{self.hostname}/sonar_data"

        self.subscription = self.create_subscription(
            Float64, self.sonar_topic, self.sonar_callback, 10
        )

        self.publisher = self.create_publisher(Float64, "motor_cmd", 10)

        # self.publisher = self.create_publisher(CPM, 'cpm', 10)  # Commented out CPM publisher

        self.distance = 0.0

        self.get_logger().info("Nœud drone1_position_control initialisé")

    def sonar_callback(self, msg):
        distance = msg.data
        self.distance = distance

        self.get_logger().info(f"distance: {distance}")

        self.publish_sonar_data(distance)

        # cpm_msg = self.create_cpm_message(distance)  # Commented out CPM message creation
        # self.publish_cpm_message(cpm_msg)  # Commented out CPM message publishing

    def publish_sonar_data(self, distance):
        # Create a Float64 message and set its data field to the distance value
        distance_msg = Float64()
        distance_msg.data = distance

        # Publish the message
        self.publisher.publish(distance_msg)
        self.get_logger().info(f"Published sonar data: {distance}")

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
