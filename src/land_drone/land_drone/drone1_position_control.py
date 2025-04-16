import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String, Float64  # type: ignore
import land_drone.utils.manageFiles as manageFiles 
import logging  

SEUIL_DISTANCE_OBSTACLE = 50  # cm


class DronePositionControl(Node):
    def __init__(self, node_name, hostname):
        super().__init__(node_name)

        self.hostname = hostname
        self.sonar_topic = f"{self.hostname}/sonar_data"
        self.motor_cmd_topic = f"{self.hostname}/motor_cmd"

        self.subscription = self.create_subscription(
            Float64, self.sonar_topic, self.sonar_callback, 10
        )
        self.publisher = self.create_publisher(String, self.motor_cmd_topic, 10)

        self.state = "Stop"  # Initial state
        self.distance = 0.0

        self.get_logger().info(
            f"Node '{node_name}' initialised. Subscribing to '{self.sonar_topic}', Publishing to '{self.motor_cmd_topic}'."
        )

    def sonar_callback(self, msg):
        distance = float(msg.data)
        self.distance = distance  

        new_state = "Forward" if distance > SEUIL_DISTANCE_OBSTACLE else "Stop"

        # Publish command only if the state changes
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(
                f"State changed to '{self.state}' based on distance ({distance:.2f} cm)"
            )
            self.publish_motor_command(self.state)

    def publish_motor_command(self, state):
        msg = String()
        msg.data = state
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published motor command '{msg.data}' to topic '{self.motor_cmd_topic}'"
        )


def main(args=None):
    rclpy.init(args=args)
    logger = logging.getLogger("dpc_main")

    hostname = None
    initial_neighbors = []
    monitor_started = False
    dpc_node = None

    try:
        hostname = manageFiles.get_robot_name()
        logger.info(f"Starting Drone Position Control for Robot Name: {hostname}")
        if not hostname or hostname.startswith("robot_"):  
            logger.warning("ROBOT_NAME environment variable may not be set correctly.")

        logger.info("Starting Host Monitor...")
        monitor = manageFiles.start_host_monitor()  
        if monitor:  
            monitor_started = True
            logger.info("Host Monitor started successfully.")
            initial_neighbors = manageFiles.get_current_neighbors()
            logger.info(f"Initial neighbors from server: {initial_neighbors}")
        else:
            logger.error(
                "Failed to get Host Monitor instance. Neighbor discovery might fail."
            )

        node_name = f"{hostname}_position_control"  
        dpc_node = DronePositionControl(node_name, hostname)

        dpc_node.get_logger().info(f"Node '{node_name}' starting spin...")
        rclpy.spin(dpc_node)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, initiating shutdown.")
    except Exception as e:
        logger.error(f"An error occurred in main: {e}", exc_info=True)
    finally:
        logger.info("Starting shutdown sequence...")

        if dpc_node is not None:
            logger.info(f"Destroying node '{dpc_node.get_name()}'...")
            dpc_node.destroy_node()
            logger.info("Node destroyed.")

        if monitor_started:
            logger.info("Stopping Host Monitor...")
            manageFiles.stop_host_monitor()
            logger.info("Host Monitor stopped.")
        else:
            logger.info("Host Monitor was not started, skipping stop.")

        logger.info("Shutting down rclpy...")
        if rclpy.ok(): 
            rclpy.shutdown()
        logger.info("rclpy shut down.")
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()
