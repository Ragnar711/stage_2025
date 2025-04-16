import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64  # type: ignore
from land_drone.sonarDrv import SonarDrv
import land_drone.utils.manageFiles as manageFiles
import logging  

THRESHOLD_OBSTACLE_DISTANCE = 10  # cm


class Sonar(Node):
    def __init__(self, node_name, hostname, auSonarDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.topic_name = (
            f"{self.hostname}/sonar_data"  
        )
        self.publisher_ = self.create_publisher(Float64, self.topic_name, 10)
        self.timer_ = self.create_timer(0.1, self.publish_message)
        self.muSonarDrv = auSonarDrv
        self.detected_obstacle = False

        self.get_logger().info(
            f"Sonar node '{node_name}' initialised with topic: '{self.topic_name}'"
        )

    def publish_message(self):
        distance = self.muSonarDrv.get_distance()
        self.update_obstacle_detection(distance)
        self.publish_distance_message(distance)

    def update_obstacle_detection(self, distance):
        if distance < THRESHOLD_OBSTACLE_DISTANCE:
            if not self.detected_obstacle:  
                self.get_logger().warn(
                    f"Obstacle DETECTED (Distance: {distance:.2f} cm)"
                )
            self.detected_obstacle = True
        else:
            if self.detected_obstacle:
                self.get_logger().info(
                    f"Obstacle CLEARED (Distance: {distance:.2f} cm)"
                )
            self.detected_obstacle = False

    def publish_distance_message(self, distance):
        msg = self.create_float_message(distance)
        self.publisher_.publish(msg)

    def create_float_message(self, distance):
        msg = Float64()
        msg.data = (
            float(distance) if distance is not None else -1.0
        )  
        return msg

    def is_detected_obstacle(self):
        return self.detected_obstacle


def main(args=None):
    rclpy.init(args=args)
    logger = logging.getLogger("sonar_main")

    hostname = None
    initial_neighbors = []
    monitor_started = False
    sonar_node = None  

    try:
        hostname = manageFiles.get_robot_name()
        logger.info(f"Starting Sonar node for Robot Name: {hostname}")
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

        luSonarDrv = SonarDrv(SonarDrv.TRIG_PIN, SonarDrv.ECHO_PIN)
        logger.info("Sonar hardware driver initialized.")

        node_name = f"{hostname}_sonar"  
        sonar_node = Sonar(node_name, hostname, luSonarDrv)

        sonar_node.get_logger().info(f"Node '{node_name}' starting spin...")
        rclpy.spin(sonar_node)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, initiating shutdown.")
    except Exception as e:
        logger.error(f"An error occurred in main: {e}", exc_info=True)
    finally:
        logger.info("Starting shutdown sequence...")

        if sonar_node is not None:
            logger.info(f"Destroying node '{sonar_node.get_name()}'...")
            sonar_node.destroy_node()
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
