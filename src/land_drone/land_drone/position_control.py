import rclpy  # type: ignore # le module rclpy pour utiliser ROS 2
from rclpy.node import Node  # type: ignore # Importe la classe Node pour créer un nœud ROS
from std_msgs.msg import (  # type: ignore
    Float64,
)  # Importe le message Float64 depuis le package std_msgs
from aiv_interfaces.msg import (
    CPM,
)  # Importe le message CPM depuis le package aiv_interfaces
from land_drone.utils.manageFiles import get_name_robot

SEUIL_DISTANCE_OBSTACLE = 10  # Définit la distance seuil pour détecter un obstacle


class DronePositionControl(Node):
    def __init__(self, hostname):
        super().__init__("position_control")  # Initialise le nœud avec un nom unique

        self.hostname = hostname
        self.sonar_topic = f"{self.hostname}/sonar_data"

        # S'abonne au topic de données du sonar
        self.subscription = self.create_subscription(
            Float64, self.sonar_topic, self.sonar_callback, 10
        )

        # Crée un éditeur pour publier des messages CPM
        self.publisher = self.create_publisher(CPM, f"{self.hostname}/cpm", 10)

        # Stocke la dernière distance mesurée
        self.last_distance = 0.0

        # Affiche des informations sur l'initialisation du nœud
        self.get_logger().info(
            f"Nœud position_control initialisé, écoute sur {self.sonar_topic}"
        )

    def sonar_callback(self, msg):
        # Le rappel appelé lorsque de nouvelles données du sonar sont reçues
        distance = msg.data  # Récupère la distance du sonar depuis le message reçu
        self.last_distance = distance  # Stocke la valeur pour utilisation future

        # Affiche la distance reçue
        self.get_logger().info(f"Distance reçue du sonar: {distance} cm")

        # Crée et publie un message CPM
        cpm_msg = self.create_cpm_message(distance)
        self.publish_cpm_message(cpm_msg)

    def create_cpm_message(self, distance):
        # Crée un message CPM (Capteur Perception Message) en fonction de la distance du sonar
        cpm_msg = CPM()

        # Pour un système réel, utilisez le temps actuel
        now = self.get_clock().now()
        cpm_msg.its_header.generation_time = now.to_msg()

        # Position actuelle (à remplir avec les coordonnées réelles dans un système complet)
        cpm_msg.current_position = []

        # Déterminer s'il y a un obstacle
        if distance < SEUIL_DISTANCE_OBSTACLE:
            cpm_msg.sensor_information = "Obstacle détecté"
            self.get_logger().warn(f"ATTENTION: Obstacle détecté à {distance} cm!")
        else:
            cpm_msg.sensor_information = "Aucun obstacle"

        # Stocke la distance mesurée
        cpm_msg.perceive_object = distance

        return cpm_msg

    def publish_cpm_message(self, cpm_msg):
        # Publie le message CPM
        self.publisher.publish(cpm_msg)
        self.get_logger().info(
            f"Message CPM publié: {cpm_msg.sensor_information}, distance: {cpm_msg.perceive_object} cm"
        )


def main(args=None):
    rclpy.init(args=args)  # Initialise le framework ROS 2

    # Obtient le hostname du robot
    hostname, _ = get_name_robot()

    # Crée une instance du nœud DronePositionControl
    drone_position_control = DronePositionControl(hostname)

    # Boucle principale pour faire fonctionner le nœud
    rclpy.spin(drone_position_control)

    # Nettoyage à la fin
    drone_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
