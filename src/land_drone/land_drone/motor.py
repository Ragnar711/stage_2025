import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64, String  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from land_drone.motorDrv import MotorDrv
from land_drone.utils.manageFiles import get_name_robot

SEUIL_DISTANCE_OBSTACLE = 100
# ZERO_SPEED_VALUE = 1500


class Motor(Node):
    def __init__(self, node_name, hostname, auMotorDrv):
        super().__init__(node_name)
        self.hostname = hostname
        self.distance = 0

        luMsgType = String
        self.msTopicName = f"{self.hostname}/motor_data"
        self.muPubMotorData = self.create_publisher(luMsgType, self.msTopicName, 10)

        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.muMotorDrv = auMotorDrv
        self.state = "Stop"
        # self.state = "Forward"

        self.get_logger().info(
            "Motor node initialised with the topic : {}".format(self.msTopicName)
        )

        # luMsgType = Twist
        # lsTopicName = f'{self.hostname}/motor_cmd'
        # self.muSubMotorCmd = self.create_subscription(
        #     luMsgType,
        #     lsTopicName,
        #     self.motor_cmd_callback,
        #     10
        # )
        # MDK Restoré juste pour le jeudi de la recherche du 20.03.2025
        # -------------------------------------------------------------
        self.subscription = self.create_subscription(
            Float64, "motor_cmd", self.sonar_data_callback, 10
        )
        # ---------------------------------------------------------

    def publish_message(self):
        # Publication dans le topic Odometrie (motor_data)
        self.publish_state_message(self.state)

        if self.state == "Forward":
            self.muMotorDrv.go_forward()
        elif self.state == "Backward":
            self.muMotorDrv.go_backward()
        else:
            self.muMotorDrv.stop()

    def publish_state_message(self, state):
        self.get_logger().info("state motors : {}".format(self.state))
        self.get_logger().info("distance : {}".format(self.distance))
        msg = self.create_string_message(state)
        self.muPubMotorData.publish(msg)
        self.get_logger().info(
            "Message publié sur le topic {}: {}".format(self.msTopicName, self.state)
        )

    def create_string_message(self, state):
        msg = String()
        msg.data = self.state
        return msg

    # def motor_cmd_callback(self, msg):
    #     lfSpeedValue = msg.linear.x
    #     self.get_logger().info('Received motor speed cmd: {} m/s'.format(lfSpeedValue))

    #     if lfSpeedValue > ZERO_SPEED_VALUE:
    #         self.state = "Forward"
    #     elif lfSpeedValue < ZERO_SPEED_VALUE:
    #         self.state = "Backward"
    #     else:
    #         self.state = "Stop"

    # MDK Restoré juste pour le jeudi de la recherche du 20.03.2025
    # -------------------------------------------------------------
    def sonar_data_callback(self, msg):
        distance = msg.data
        self.get_logger().info("Received sonar data: {} cm".format(distance))
        self.distance = distance
        if distance > SEUIL_DISTANCE_OBSTACLE:
            self.state = "Forward"
        else:
            self.state = "Stop"

    # ---------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    node = Node(
        "motor",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    hostname = node.get_parameter("hostname").value  # get hostname from the launcher
    if hostname == None:
        hostname, _ = (
            get_name_robot()
        )  # get hostname from the function, because the node is running independently without the launcher

    luMotorDrv = MotorDrv()

    motor_node = Motor(f"{hostname}_motor", hostname, luMotorDrv)
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass  # Permet au programme de s'arrêter proprement si l'utilisateur appuie sur Ctrl+C
    finally:
        motor_node.muMotorDrv.stop()  # Assurez-vous que le moteur s'arrête avant de détruire le nœud
        motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
