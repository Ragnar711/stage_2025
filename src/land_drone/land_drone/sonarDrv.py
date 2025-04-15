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
        GPIO.output(self.mnTrigPin, False)  # Set the TRIG pin to LOW
        print("Waiting for sensor to stabilize")
        time.sleep(2)  # Wait for the sensor to stabilize

        GPIO.output(self.mnTrigPin, True)  # Send a pulse to trigger the sensor
        time.sleep(0.00001)  # Wait for a very short time
        GPIO.output(self.mnTrigPin, False)  # Set the TRIG pin back to LOW

        pulse_start = pulse_end = 0  # Initialize variables

        while GPIO.input(self.mnEchoPin) == 0:  # Wait for the echo pin to go HIGH
            pulse_start = time.time()  # Record the start time if it transitions to HIGH

        while GPIO.input(self.mnEchoPin) == 1:  # Wait for the echo pin to go LOW
            pulse_end = time.time()  # Record the end time when it goes LOW

        if pulse_start and pulse_end:  # Ensure both pulse_start and pulse_end have been set
            pulse_duration = pulse_end - pulse_start  # Calculate pulse duration
            distance = pulse_duration * 17150  # Calculate distance
            distance = round(distance, 2)  # Round the distance to 2 decimal places
            print(f"Distance: {distance} cm")  # Print the measured distance
            return distance
        else:
            print("Error: Echo signal not received properly.")
            return 51.0  # Return 51 if the echo signal was not detected properly

