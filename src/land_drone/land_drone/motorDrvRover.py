import busio  # type: ignore
import adafruit_mcp4725  # type: ignore
import board  # type: ignore


class MotorDrvRover:
    i2c = busio.I2C(board.SCL, board.SDA)
    motor1 = adafruit_mcp4725.MCP4725(i2c, address=0x60)
    motor2 = adafruit_mcp4725.MCP4725(i2c, address=0x61)

    def __init__(self):
        pass

    def go_forward(self):
        MotorDrvRover.motor1.value = int((4.0 / 0.33) * 4095)
        MotorDrvRover.motor2.value = int((4.0 / 0.33) * 4095)
        state = "Forward"
        return state

    def go_backward(self):
        MotorDrvRover.motor1.value = int((1.0 / 0.33) * 4095)
        MotorDrvRover.motor2.value = int((1.0 / 0.33) * 4095)
        state = "Backward"
        return state

    def stop(self):
        MotorDrvRover.motor1.value = int((2.5 / 0.33) * 4095)
        MotorDrvRover.motor2.value = int((2.5 / 0.33) * 4095)
        state = "Stop"
        return state
