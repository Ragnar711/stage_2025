import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String  # type: ignore
from land_drone.motorDrv import MotorDrv
from land_drone.utils.manageFiles import get_name_robot


class Motor(Node):
    def __init__(self, node_name, hostname, auMotorDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.muMotorDrv = auMotorDrv
        self._current_state = "Stop"  # Internal tracking of the motor state

        # Publisher for reporting the motor's actual state (for monitoring)
        self.state_reporter_topic = f"{self.hostname}/motor_state"
        self.state_publisher = self.create_publisher(
            String, self.state_reporter_topic, 10
        )
        # Timer to periodically publish the current state
        self.state_report_timer = self.create_timer(1.0, self.publish_state_message)

        # Subscription to motor commands from the control logic
        self.command_topic = "motor_cmd"
        self.command_subscription = self.create_subscription(
            String,
            self.command_topic,
            self.motor_command_callback,  # Callback executes commands immediately
            10,  # QoS profile depth
        )

        self.get_logger().info(
            f"Motor node initialized. Subscribed to '{self.command_topic}', Publishing state to '{self.state_reporter_topic}'"
        )
        # Ensure motor is stopped on initialization for safety
        self.muMotorDrv.stop()
        self.get_logger().info("Motor stopped initially.")

    def motor_command_callback(self, msg):
        """
        Callback function for receiving motor commands ('Forward', 'Backward', 'Stop').
        Acts on the command immediately if it's different from the current state.
        """
        requested_state = msg.data
        self.get_logger().debug(f"Received motor command: {requested_state}")

        # Only execute if the requested state is new to prevent redundant driver calls
        if requested_state != self._current_state:
            self.get_logger().info(
                f"Changing motor state from '{self._current_state}' to '{requested_state}'"
            )
            if requested_state == "Forward":
                self.muMotorDrv.go_forward()
                self._current_state = "Forward"
            elif requested_state == "Backward":
                self.muMotorDrv.go_backward()
                self._current_state = "Backward"
            else:  # Includes "Stop" or any unexpected value
                if (
                    self._current_state != "Stop"
                ):  # Log only if it wasn't already stopped
                    self.get_logger().info("Stopping motor.")
                self.muMotorDrv.stop()
                self._current_state = "Stop"  # Treat unexpected commands as stop
        # else:
        #      self.get_logger().debug(f"Motor already in state: {self._current_state}")

    def publish_state_message(self):
        """Publishes the current state of the motor periodically for monitoring."""
        msg = String()
        msg.data = self._current_state
        self.state_publisher.publish(msg)
        # self.get_logger().debug(f"Published current motor state: {self._current_state}")


def main(args=None):
    rclpy.init(args=args)

    # Temporary node to get parameters safely
    param_node = Node(
        "motor_param_getter",
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
    param_node.destroy_node()

    if hostname is None or hostname == "":
        print("ERROR: Could not determine hostname for Motor node.")
        rclpy.shutdown()
        return

    # Choose MotorDrv implementation (default is 'pitop')
    # luMotorDrv = MotorDrv(driver_type="rover") # Example if using rover handler
    luMotorDrv = MotorDrv()

    motor_node = Motor(f"{hostname}_motor", hostname, luMotorDrv)
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        motor_node.get_logger().info("Keyboard interrupt received, stopping motor...")
    finally:
        # Ensure motor stops cleanly on exit
        if (
            rclpy.ok()
        ):  # Check if context is still valid before using node logger/driver
            motor_node.get_logger().info("Shutting down, ensuring motor is stopped.")
            # Ensure stop command is sent before destroying node
            if hasattr(motor_node, "muMotorDrv"):
                motor_node.muMotorDrv.stop()
            motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
