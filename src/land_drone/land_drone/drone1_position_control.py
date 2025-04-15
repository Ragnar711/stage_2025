import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String, Float64  # type: ignore
from land_drone.utils.manageFiles import get_name_robot

# Define the distance (in cm) below which the drone should stop.
# Tune based on sensor accuracy, braking distance, and desired safety margin.
SEUIL_DISTANCE_OBSTACLE = 50  # Example: stop if obstacle is within 40cm


class DronePositionControl(Node):
    def __init__(self, hostname):
        # Node name includes hostname for uniqueness
        super().__init__(f"{hostname}_position_control")

        self.hostname = hostname
        # Define topic names based on hostname and standard conventions
        self.sonar_topic = f"{self.hostname}/sonar_data"
        self.motor_command_topic = (
            "motor_cmd"  # Topic to send commands to the motor node
        )

        # Subscribe to sonar distance data
        self.subscription = self.create_subscription(
            Float64, self.sonar_topic, self.sonar_callback, 10  # QoS depth
        )

        # Publish motor commands ('Forward', 'Stop', 'Backward')
        self.publisher = self.create_publisher(
            String, self.motor_command_topic, 10  # QoS depth
        )

        # Track the last command sent to avoid redundant publications
        self._last_command_sent = None

        self.get_logger().info(
            f"Node '{self.get_name()}' initialized. Listening to '{self.sonar_topic}', publishing to '{self.motor_command_topic}'. Obstacle threshold: {SEUIL_DISTANCE_OBSTACLE} cm."
        )

    def sonar_callback(self, msg):
        """
        Processes incoming sonar distance data and decides the motor command.
        """
        distance = msg.data
        # self.get_logger().debug(f"Received sonar data: {distance:.2f} cm") # Optional debug log

        # Determine desired motor state based on distance
        desired_state = "Stop"  # Default to Stop for safety
        if distance > SEUIL_DISTANCE_OBSTACLE:
            desired_state = "Forward"
        # Handle threshold
        elif distance <= SEUIL_DISTANCE_OBSTACLE:
            desired_state = "Stop"

        # Only publish the command if it's different from the last one sent
        if desired_state != self._last_command_sent:
            self.publish_motor_command(desired_state)
            self._last_command_sent = desired_state
        # else:
        #     self.get_logger().debug(f"Desired state '{desired_state}' is same as last sent. No publish.")

    def publish_motor_command(self, state):
        """Publishes the given state ('Forward', 'Stop', etc.) to the motor command topic."""
        msg = String()
        msg.data = state
        self.publisher.publish(msg)
        self.get_logger().info(f"Published motor command: {state}")


def main(args=None):
    rclpy.init(args=args)

    hostname, _ = get_name_robot()
    if hostname is None or hostname == "":
        print("ERROR: Could not determine hostname for DronePositionControl.")
        rclpy.shutdown()
        return

    drone_position_control = DronePositionControl(hostname)

    try:
        rclpy.spin(drone_position_control)
    except KeyboardInterrupt:
        drone_position_control.get_logger().info(
            "Keyboard interrupt received, shutting down position control."
        )
    finally:
        # Cleanly destroy node upon exit
        drone_position_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
