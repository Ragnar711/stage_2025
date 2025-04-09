import time
from mpu6050 import mpu6050  # type: ignore


class GyroDrv:

    def __init__(self):
        self.mpu = mpu6050(0x68)

    def get_gyro_data(self):
        gyro_data = self.mpu.get_gyro_data()
        x = gyro_data["x"]
        y = gyro_data["y"]
        z = gyro_data["z"]
        return x, y, z

    def get_accel_data(self):
        accel_data = self.mpu.get_accel_data()
        ax = accel_data["x"]
        ay = accel_data["y"]
        az = accel_data["z"]
        return ax, ay, az
