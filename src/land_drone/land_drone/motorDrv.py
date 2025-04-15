class MotorDrv:
    def __init__(self, driver_type="pitop"):
        # Select which implementation to use based on driver_type
        if driver_type == "rover":
            from .utils.motor_handlers.rover_handler import RoverHandler

            self.handler = RoverHandler()
        else:  # Default to pitop
            from .utils.motor_handlers.pitop_handler import PitopHandler

            self.handler = PitopHandler()

    def go_forward(self):
        return self.handler.go_forward()

    def go_backward(self):
        return self.handler.go_backward()

    def stop(self):
        return self.handler.stop()
