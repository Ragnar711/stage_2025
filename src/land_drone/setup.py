from setuptools import setup, find_packages
import os
from glob import glob

package_name = "land_drone"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.py")),
        (os.path.join("share", package_name), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="drone",
    maintainer_email="azizbechaib711@gmail.com",
    description="This package contains the land drone project.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "exchangeMessages = land_drone.exchangeMessages:main",
            "sonar = land_drone.sonar:main",
            "motor = land_drone.motor:main",
            "gps = land_drone.gps:main",
            "gyro = land_drone.gyro:main",
            "sensors = land_drone.sensors:main",
            "demarrage = land_drone.demarrage:main",
            "position_control = land_drone.position_control:main",
            "planner_action_client = land_drone.planner_action_client:main",
            "planner_action_server = land_drone.planner_action_server:main",
            "position_control = land_drone.position_control:main",
        ],
    },
)
