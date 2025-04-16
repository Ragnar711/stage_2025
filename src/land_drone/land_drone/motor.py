import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String  # type: ignore
from land_drone.motorDrv import MotorDrv
import land_drone.utils.manageFiles as manageFiles
import logging


class Motor(Node):
    def __init__(self, node_name, hostname, auMotorDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.muMotorDrv = auMotorDrv
        self.state = "Stop"  # Initial state

        self.motor_cmd_topic = f"{self.hostname}/motor_cmd"
        self.subscription = self.create_subscription(
            String, self.motor_cmd_topic, self.motor_cmd_callback, 10
        )
        self.get_logger().info(
            f"Subscribing to motor commands on: '{self.motor_cmd_topic}'"
        )

        self.control_timer_ = self.create_timer(0.1, self.apply_motor_state)

        self.get_logger().info(f"Motor node '{node_name}' initialised.")

    def motor_cmd_callback(self, msg: String):
        """Callback for receiving motor commands (e.g., 'Forward', 'Stop')."""
        received_state = msg.data
        if received_state != self.state:
            self.get_logger().info(
                f"Received motor command: '{received_state}', updating state."
            )
            self.state = received_state

    def apply_motor_state(self):
        """Applies the current self.state to the physical motors."""
        if self.state == "Forward":
            self.muMotorDrv.go_forward()
        elif self.state == "Backward":
            self.muMotorDrv.go_backward()
        else:
            if self.state != "Stop":
                self.get_logger().warning(
                    f"Unknown motor state '{self.state}', defaulting to Stop."
                )
                self.state = "Stop"
            self.muMotorDrv.stop()


def main(args=None):
    rclpy.init(args=args)
    logger = logging.getLogger("motor_main")

    hostname = None
    initial_neighbors = []
    monitor_started = False
    motor_node = None

    try:
        hostname = manageFiles.get_robot_name()
        logger.info(f"Starting Motor node for Robot Name: {hostname}")
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

        luMotorDrv = MotorDrv()
        logger.info("Motor hardware driver initialized.")

        node_name = f"{hostname}_motor"
        motor_node = Motor(node_name, hostname, luMotorDrv)

        motor_node.get_logger().info(f"Node '{node_name}' starting spin...")
        rclpy.spin(motor_node)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, initiating shutdown.")
    except Exception as e:
        logger.error(f"An error occurred in main: {e}", exc_info=True)
    finally:
        logger.info("Starting shutdown sequence...")

        if motor_node is not None and hasattr(motor_node, "muMotorDrv"):
            logger.info("Stopping motors...")
            try:
                motor_node.muMotorDrv.stop()
                logger.info("Motors stopped.")
            except Exception as motor_stop_e:
                logger.error(f"Error stopping motors: {motor_stop_e}", exc_info=True)

        if motor_node is not None:
            logger.info(f"Destroying node '{motor_node.get_name()}'...")
            motor_node.destroy_node()
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
