import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String, Float64  # type: ignore
import land_drone.utils.manageFiles as manageFiles  # Import the NEW manageFiles module
import logging  # Use standard logging for early messages

SEUIL_DISTANCE_OBSTACLE = 50  # cm


class DronePositionControl(Node):
    # Node name is now set in main based on hostname
    def __init__(self, node_name, hostname):
        super().__init__(node_name)

        self.hostname = hostname
        # Construct topic names using the dynamic hostname
        self.sonar_topic = f"{self.hostname}/sonar_data"
        self.motor_cmd_topic = f"{self.hostname}/motor_cmd"

        self.subscription = self.create_subscription(
            Float64, self.sonar_topic, self.sonar_callback, 10
        )
        # Publisher uses the correctly constructed topic name
        self.publisher = self.create_publisher(String, self.motor_cmd_topic, 10)

        self.state = "Stop"  # Initial state
        self.distance = 0.0

        self.get_logger().info(
            f"Node '{node_name}' initialised. Subscribing to '{self.sonar_topic}', Publishing to '{self.motor_cmd_topic}'."
        )

    def sonar_callback(self, msg):
        # Ensure received data is treated as float
        distance = float(msg.data)
        self.distance = distance  # Store the latest distance

        # Log received distance less frequently or only if needed for debugging
        # self.get_logger().info(f"Received sonar data: {distance:.2f} cm")

        new_state = "Forward" if distance > SEUIL_DISTANCE_OBSTACLE else "Stop"

        # Publish command only if the state changes
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(
                f"State changed to '{self.state}' based on distance ({distance:.2f} cm)"
            )
            self.publish_motor_command(self.state)
        # else:
        # Optionally log that state remains unchanged if needed
        # self.get_logger().debug(f"State remains '{self.state}' (Distance: {distance:.2f} cm)")

    # Renamed for clarity: This publishes the command based on sonar processing
    def publish_motor_command(self, state):
        msg = String()
        msg.data = state
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published motor command '{msg.data}' to topic '{self.motor_cmd_topic}'"
        )


def main(args=None):
    rclpy.init(args=args)
    # Use standard logger before ROS node logger is available
    logger = logging.getLogger("dpc_main")

    hostname = None
    initial_neighbors = []
    monitor_started = False
    dpc_node = None  # Initialize to None for cleanup check

    try:
        # 1. Get hostname from environment variable via manageFiles
        hostname = manageFiles.get_robot_name()
        logger.info(f"Starting Drone Position Control for Robot Name: {hostname}")
        if not hostname or hostname.startswith("robot_"):  # Basic check
            logger.warning("ROBOT_NAME environment variable may not be set correctly.")
            # Decide behavior: exit? use default? For now, we proceed.

        # 2. Start the host monitor (registers, starts SSE/heartbeat)
        logger.info("Starting Host Monitor...")
        monitor = manageFiles.start_host_monitor()  # Ensures monitor starts/exists
        if monitor:  # Basic check that monitor instance was created
            monitor_started = True
            logger.info("Host Monitor started successfully.")
            # 3. Get initial neighbors list (optional, good for logging)
            initial_neighbors = manageFiles.get_current_neighbors()
            logger.info(f"Initial neighbors from server: {initial_neighbors}")
        else:
            logger.error(
                "Failed to get Host Monitor instance. Neighbor discovery might fail."
            )
            # Depending on requirements, you might want to exit here

        # 4. Create and spin the node
        node_name = f"{hostname}_position_control"  # Unique node name
        dpc_node = DronePositionControl(node_name, hostname)

        dpc_node.get_logger().info(f"Node '{node_name}' starting spin...")
        rclpy.spin(dpc_node)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, initiating shutdown.")
    except Exception as e:
        # Log any unexpected errors during setup or spin
        logger.error(f"An error occurred in main: {e}", exc_info=True)
    finally:
        # 5. Cleanup Sequence (runs on normal exit, Ctrl+C, or error)
        logger.info("Starting shutdown sequence...")

        if dpc_node is not None:
            logger.info(f"Destroying node '{dpc_node.get_name()}'...")
            dpc_node.destroy_node()
            logger.info("Node destroyed.")

        if monitor_started:
            logger.info("Stopping Host Monitor...")
            # Explicitly stop monitor (sends unregister, stops threads)
            manageFiles.stop_host_monitor()
            logger.info("Host Monitor stopped.")
        else:
            logger.info("Host Monitor was not started, skipping stop.")

        logger.info("Shutting down rclpy...")
        if rclpy.ok():  # Check if it's already shut down
            rclpy.shutdown()
        logger.info("rclpy shut down.")
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()
