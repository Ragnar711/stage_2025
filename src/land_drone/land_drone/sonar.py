import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64  # type: ignore
from land_drone.sonarDrv import SonarDrv
from land_drone.utils.manageFiles import get_name_robot


class Sonar(Node):
    def __init__(self, node_name, hostname, auSonarDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.topic_name = f"{self.hostname}/sonar_data"
        self.publisher_ = self.create_publisher(Float64, self.topic_name, 10)

        # Timer controls how frequently sonar distance is published
        timer_period = 0.1  # seconds (publish 10 times per second for responsiveness)
        self.timer_ = self.create_timer(timer_period, self.publish_message)

        self.muSonarDrv = auSonarDrv
        self.get_logger().info(
            "Sonar node initialised with the topic: {} (Publishing every {}s)".format(
                self.topic_name, timer_period
            )
        )

    def publish_message(self):
        """Called by the timer to read distance and publish it."""
        distance = self.muSonarDrv.get_distance()
        self.publish_distance_message(distance)

    def publish_distance_message(self, distance):
        """Creates and publishes a Float64 message with the distance."""
        msg = self.create_float_message(distance)
        self.publisher_.publish(msg)
        # Optional: Log published message (can be noisy)
        # self.get_logger().debug(f'Published distance: {distance} on {self.topic_name}')

    def create_float_message(self, distance):
        """Safely creates a Float64 message."""
        msg = Float64()
        try:
            msg.data = float(distance)
        except (ValueError, TypeError):
            self.get_logger().error(
                f"Invalid distance value received: {distance}. Sending error value 51.0"
            )
            msg.data = 51.0  # Default error value consistent with SonarDrv
        return msg


def main(args=None):
    rclpy.init(args=args)

    # Temporary node to get parameters safely
    param_node = Node(
        "sonar_param_getter",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    try:
        hostname = param_node.get_parameter("hostname").value
    except rclpy.exceptions.ParameterNotDeclaredException:
        param_node.get_logger().warn(
            "Hostname parameter not set via launch file, trying fallback."
        )
        hostname, _ = get_name_robot()  # Fallback using utility function
    param_node.destroy_node()  # Clean up temporary node

    if hostname is None or hostname == "":
        print("ERROR: Could not determine hostname for Sonar node.")
        rclpy.shutdown()
        return

    luSonarDrv = SonarDrv(SonarDrv.TRIG_PIN, SonarDrv.ECHO_PIN)
    sonar_node = Sonar(f"{hostname}_sonar", hostname, luSonarDrv)

    try:
        rclpy.spin(sonar_node)
    except KeyboardInterrupt:
        sonar_node.get_logger().info(
            "Keyboard interrupt received, shutting down sonar node."
        )
    finally:
        sonar_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
