import time  # Importe le module time pour les opérations liées au temps
import RPi.GPIO as GPIO  # type: ignore # Importe le module RPi.GPIO pour accéder aux broches GPIO du Raspberry Pi


class SonarDrv:
    TRIG_PIN = 6
    ECHO_PIN = 5

    def __init__(self, anTrigPin, anEchoPin):
        self.mnTrigPin = anTrigPin  # Définit le numéro de broche GPIO pour la broche de déclenchement du capteur sonar
        self.mnEchoPin = anEchoPin  # Définit le numéro de broche GPIO pour la broche d'écho du capteur sonar
        GPIO.setmode(GPIO.BCM)  # Utilisation du mode BCM pour les numéros GPIO (plus flexible)
        GPIO.setup(self.mnTrigPin, GPIO.OUT)  # Configure la broche de déclenchement en tant que broche de sortie
        GPIO.setup(self.mnEchoPin, GPIO.IN)  # Configure la broche d'écho en tant que broche d'entrée


    def get_distance(self):
        GPIO.output(
            self.mnTrigPin, False
        )  # Définit la broche de déclenchement à basse tension (0V) initialement
        print("En attente de stabilisation du capteur")
        time.sleep(
            2
        )  # Attend pendant 2 secondes pour permettre au capteur de se stabiliser

        GPIO.output(
            self.mnTrigPin, True
        )  # Définit la broche de déclenchement à haute tension (3,3V)
        time.sleep(0.00001)  # Attend pendant une très courte durée
        GPIO.output(
            self.mnTrigPin, False
        )  # Remet la broche de déclenchement à basse tension (0V)

        while (
            GPIO.input(self.mnEchoPin) == 0
        ):  # Attend que la broche d'écho devienne haute (3,3V)
            pulse_start = time.time()  # Enregistre le moment de début de l'impulsion

        while (
            GPIO.input(self.mnEchoPin) == 1
        ):  # Attend que la broche d'écho redevienne basse (0V)
            pulse_end = time.time()  # Enregistre le moment de fin de l'impulsion

        pulse_duration = pulse_end - pulse_start  # Calcule la durée de l'impulsion
        distance = (
            pulse_duration * 17150
        )  # Calcule la distance en fonction de la durée de l'impulsion et de la vitesse du son
        distance = round(distance, 2)  # Arrondit la distance à 2 décimales
        print(
            "Distance :", distance, "cm"
        )  # Affiche la distance mesurée en centimètres

        return distance  # Renvoie la distance mesurée
