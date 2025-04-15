import time
import RPi.GPIO as GPIO  # type: ignore


class SonarDrv:
    # Define GPIO pins for Sonar sensor
    TRIG_PIN = 6
    ECHO_PIN = 5

    def __init__(self, anTrigPin, anEchoPin):
        self.mnTrigPin = anTrigPin  # GPIO pin for trigger pulse
        self.mnEchoPin = anEchoPin  # GPIO pin for echo pulse receive
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setup(self.mnTrigPin, GPIO.OUT)
        GPIO.setup(self.mnEchoPin, GPIO.IN)

        # Allow sensor to stabilize once during initialization
        print("Waiting for sensor to stabilize (initialization)...")
        GPIO.output(self.mnTrigPin, False)
        # Reduced stabilization time - adjust if necessary
        time.sleep(0.5)
        print("Sensor stabilized.")

    def get_distance(self):
        """
        Triggers the sonar sensor and reads the echo to calculate distance.
        Returns distance in cm or 51.0 on error/timeout.
        """
        # Ensure TRIG is low before starting pulse
        GPIO.output(self.mnTrigPin, False)
        time.sleep(0.01)

        # Send 10 microsecond trigger pulse
        GPIO.output(self.mnTrigPin, True)
        time.sleep(0.00001)
        GPIO.output(self.mnTrigPin, False)

        pulse_start = time.time()
        pulse_end = pulse_start
        timeout_start = time.time()

        # Wait for the echo pin to go HIGH (with timeout)
        while GPIO.input(self.mnEchoPin) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > 0.1:  # Timeout after 0.1 seconds
                print("Error: Timeout waiting for echo pulse start.")
                return 51.0  # Return error/max distance

        timeout_start = time.time()  # Reset timeout timer
        # Wait for the echo pin to go LOW (with timeout)
        while GPIO.input(self.mnEchoPin) == 1:
            pulse_end = time.time()
            # Timeout if pulse is unreasonably long (e.g., > 0.1s implies > 17m distance)
            if pulse_end - pulse_start > 0.1:
                print("Error: Timeout waiting for echo pulse end (pulse too long).")
                return 51.0  # Return error/max distance
            # General timeout waiting for pin to go low
            if pulse_end - timeout_start > 0.1:
                print("Error: Timeout waiting for echo pulse end.")
                return 51.0  # Return error/max distance

        # Calculate duration and distance
        pulse_duration = pulse_end - pulse_start
        # Speed of sound is approx 34300 cm/s.
        # Distance = (duration * speed_of_sound) / 2 (round trip)
        distance = (pulse_duration * 34300) / 2
        distance = round(distance, 2)

        # Basic bounds check for typical HC-SR04 sensor range
        if distance < 2:  # Min reliable distance ~2cm
            return 2.0
        elif distance > 400:  # Max reliable distance ~400cm
            return 51.0  # Use the error value for out of range

        return distance
