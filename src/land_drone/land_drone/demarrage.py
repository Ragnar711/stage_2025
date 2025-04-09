import rclpy  # type: ignore
from land_drone.sonar import Sonar
from land_drone.sonarDrv import SonarDrv
from land_drone.gps import GPS
from land_drone.GPSDrv import GPSDrv


def test_gps():
    rclpy.init()
    try:
        luGPSDrv = GPSDrv()
        gps_test_node = GPS(luGPSDrv)
        rclpy.spin_once(gps_test_node, timeout_sec=1.0)
        gps_test_node.publish_message()
    finally:
        gps_test_node.destroy_node()
        rclpy.shutdown()


def test_sonar():
    rclpy.init()
    try:
        luSonarDrv = SonarDrv(SonarDrv.TRIG_PIN, SonarDrv.ECHO_PIN)
        sonar_test_node = Sonar(luSonarDrv)
        rclpy.spin_once(sonar_test_node, timeout_sec=1.0)
        sonar_test_node.publish_message()
    finally:
        sonar_test_node.destroy_node()
        rclpy.shutdown()


def main():
    test_sonar()
    test_gps()


if __name__ == "__main__":
    main()
