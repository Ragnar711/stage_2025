import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64  # type: ignore
from land_drone.sonarDrv import SonarDrv

# Import the NEW manageFiles functions
import land_drone.utils.manageFiles as manageFiles
import logging  # Use standard logging for early messages

THRESHOLD_OBSTACLE_DISTANCE = 10  # cm


class Sonar(Node):
    def __init__(self, node_name, hostname, auSonarDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.topic_name = (
            f"{self.hostname}/sonar_data"  # Topic based on dynamic hostname
        )
        self.publisher_ = self.create_publisher(Float64, self.topic_name, 10)
        # Timer interval remains 0.1 seconds (10 Hz)
        self.timer_ = self.create_timer(0.1, self.publish_message)
        self.muSonarDrv = auSonarDrv
        self.detected_obstacle = False

        # Get neighbors *inside* the node if needed later
        # self.current_neighbors = manageFiles.get_current_neighbors()
        # self.get_logger().info(f"Initial neighbors seen by node: {self.current_neighbors}")

        self.get_logger().info(
            f"Sonar node '{node_name}' initialised with topic: '{self.topic_name}'"
        )

    def publish_message(self):
        distance = self.muSonarDrv.get_distance()
        self.update_obstacle_detection(distance)
        self.publish_distance_message(distance)

        # If you need to check neighbors periodically within the node:
        # latest_neighbors = manageFiles.get_current_neighbors()
        # if latest_neighbors != self.current_neighbors:
        #    self.get_logger().info(f"Neighbor list updated: {latest_neighbors}")
        #    self.current_neighbors = latest_neighbors
        #    # ... react to neighbor changes ...

    def update_obstacle_detection(self, distance):
        if distance < THRESHOLD_OBSTACLE_DISTANCE:
            if not self.detected_obstacle:  # Log only on change
                self.get_logger().warn(
                    f"Obstacle DETECTED (Distance: {distance:.2f} cm)"
                )
            self.detected_obstacle = True
        else:
            if self.detected_obstacle:  # Log only on change
                self.get_logger().info(
                    f"Obstacle CLEARED (Distance: {distance:.2f} cm)"
                )
            self.detected_obstacle = False

    def publish_distance_message(self, distance):
        # Reduce logging frequency for distance, maybe log only if changed significantly or less often
        # self.get_logger().info("Distance: {} cm".format(distance))
        msg = self.create_float_message(distance)
        self.publisher_.publish(msg)
        # Reduce logging for publishing too
        # self.get_logger().info(
        #    'Message published on the topic {}: "{}"'.format(self.topic_name, distance)
        # )

    def create_float_message(self, distance):
        msg = Float64()
        # Ensure distance is float, handle potential None from driver gracefully
        msg.data = (
            float(distance) if distance is not None else -1.0
        )  # Or some indicator value
        return msg

    def is_detected_obstacle(self):
        return self.detected_obstacle


def main(args=None):
    rclpy.init(args=args)
    # Use standard logger before ROS node logger is available
    logger = logging.getLogger("sonar_main")

    hostname = None
    initial_neighbors = []
    monitor_started = False
    sonar_node = None  # Initialize to None for cleanup check

    try:
        # 1. Get hostname from environment variable via manageFiles
        hostname = manageFiles.get_robot_name()
        logger.info(f"Starting Sonar node for Robot Name: {hostname}")
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
            # This case should ideally not happen if start_host_monitor logs errors
            logger.error(
                "Failed to get Host Monitor instance. Neighbor discovery might fail."
            )
            # Depending on requirements, you might want to exit here

        # 4. Initialize hardware driver
        # Make sure TRIG/ECHO pins are correct for your hardware
        luSonarDrv = SonarDrv(SonarDrv.TRIG_PIN, SonarDrv.ECHO_PIN)
        logger.info("Sonar hardware driver initialized.")

        # 5. Create and spin the node
        node_name = f"{hostname}_sonar"  # Unique node name using hostname
        sonar_node = Sonar(node_name, hostname, luSonarDrv)

        sonar_node.get_logger().info(f"Node '{node_name}' starting spin...")
        rclpy.spin(sonar_node)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, initiating shutdown.")
    except Exception as e:
        # Log any unexpected errors during setup or spin
        logger.error(f"An error occurred in main: {e}", exc_info=True)
    finally:
        # 6. Cleanup Sequence (runs on normal exit, Ctrl+C, or error)
        logger.info("Starting shutdown sequence...")

        if sonar_node is not None:
            logger.info(f"Destroying node '{sonar_node.get_name()}'...")
            sonar_node.destroy_node()
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
