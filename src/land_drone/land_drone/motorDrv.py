class MotorDrv:
    def __init__(self, driver_type="pitop"):
        # Select which implementation to use based on driver_type
        if driver_type == "rover":
            from .motorDrvRover import MotorDrvRover

            self.handler = MotorDrvRover()
        else:  # Default to pitop
            from .motorDrvPitop import MotorDrvPitop

            self.handler = MotorDrvPitop()

    def go_forward(self):
        return self.handler.go_forward()

    def go_backward(self):
        return self.handler.go_backward()

    def stop(self):
        return self.handler.stop()
