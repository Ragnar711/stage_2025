from .utils.motor_lib.encoder_motor import EncoderMotor
from .utils.motor_lib.parameters import Direction


class MotorDrvPitop:
    def __init__(self, rpm_speed=100):
        self.motor1 = EncoderMotor("M0", Direction.BACK)
        self.motor2 = EncoderMotor("M3", Direction.BACK)
        self.rpm_speed = rpm_speed

    def go_forward(self):
        self.motor1.set_target_rpm(self.rpm_speed)
        self.motor2.set_target_rpm(-self.rpm_speed)
        state = "Forward"
        return state

    def go_backward(self):
        self.motor1.set_target_rpm(-self.rpm_speed)
        self.motor2.set_target_rpm(self.rpm_speed)
        state = "Backward"
        return state

    def stop(self):
        self.motor1.stop()
        self.motor2.stop()
        state = "Stop"
        return state
