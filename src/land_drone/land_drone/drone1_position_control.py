import rclpy  #le module rclpy pour utiliser ROS 2
from rclpy.node import Node  # Importe la classe Node pour créer un nœud ROS
from std_msgs.msg import Float64  # Importe le message Float64 depuis le package std_msgs
from aiv_interfaces.msg import CPM  # Importe le message CPM depuis le package aiv_interfaces
from land_drone.sonar import Sonar, SonarDrv



SEUIL_DISTANCE_OBSTACLE = 10  # Définit la distance seuil pour détecter un obstacle

class DronePositionControl(Node):
    def __init__(self):
        super().__init__('drone1_position_control')  # Initialise le nœud avec un nom unique
        self.subscription = self.create_subscription(
            Float64,  
            'sonar_data', 
            self.sonar_callback,  
            10 
        )
        self.publisher = self.create_publisher(CPM, 'cpm', 10) 
        self.get_logger().info('Nœud drone1_position_control initialisé')

        # Ajoutez ces lignes pour créer un nœud Sonar et un objet SonarDrv
        self.sonar_drv = SonarDrv(SonarDrv.TRIG_PIN, SonarDrv.ECHO_PIN)
        self.sonar_node = Sonar(self.sonar_drv)

    def sonar_callback(self, msg): # Le rappel (callback) appelé lorsque de nouvelles données du sonar sont reçues   
        distance = msg.data  # Récupère la distance du sonar depuis le message reçu
        cpm_msg = self.create_cpm_message(distance)  # Crée un message CPM en fonction de la distance
        self.publish_cpm_message(cpm_msg)  # Publie le message CPM

    def create_cpm_message(self, distance):
        # Crée un message CPM (Capteur Perception Message) en fonction de la distance du sonar
        cpm_msg = CPM()
        cpm_msg.its_header.generation_time = 0  # Temps de génération du message (à remplacer par la valeur réelle)
        cpm_msg.current_position = []  # Position actuelle du drone (à remplir avec les coordonnées réelles)

        if distance < SEUIL_DISTANCE_OBSTACLE:
            cpm_msg.sensor_information = "Obstacle détecté"  # Message indiquant la détection d'un obstacle
        else:
            cpm_msg.sensor_information = "Aucun obstacle"  # Message indiquant l'absence d'obstacle

        cpm_msg.perceive_object = self.sonar_drv.get_distance()  # Récupère la distance depuis le SonarDrv

        return cpm_msg

    def publish_cpm_message(self, cpm_msg):
        # Publie le message CPM
        self.publisher.publish(cpm_msg)  # Publie le message sur le topic 'cpm'
        self.get_logger().info('Message CPM publié')

    # Ajoutez cette méthode pour exécuter le nœud Sonar
    def run_sonar_node(self):
        rclpy.spin(self.sonar_node)

def main(args=None):
    rclpy.init(args=args)  # Initialise le framework ROS 2
    drone_position_control = DronePositionControl()  # Crée une instance du nœud DronePositionControl
    drone_position_control.run_sonar_node()  # Exécute le nœud Sonar
    rclpy.spin(drone_position_control)  # Boucle principale pour faire fonctionner le nœud
    drone_position_control.destroy_node()  # Détruit le nœud DronePositionControl
    rclpy.shutdown()  # Arrête le framework ROS 2
    
if __name__ == '__main__':
    main()

