from glob import glob
from setuptools import find_packages, setup


package_name = "pm01_aruco_detection"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PM01 development team",
    maintainer_email="pm01-dev@example.com",
    description="Person-follow controller and armed cmd_vel safety bridge for PM01.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector = pm01_aruco_detection.detector_node:main",
            "aruco_generate_marker = pm01_aruco_detection.generate_marker:main",
            "aruco_detect_image = pm01_aruco_detection.detect_image:main",
            "person_follow_controller = pm01_aruco_detection.person_follow_controller:main",
            "cmd_vel_safety_bridge = pm01_aruco_detection.cmd_vel_safety_bridge:main",
            "body_vel_bridge = pm01_aruco_detection.body_vel_bridge:main",
            "pm01_gamepad_bridge = pm01_aruco_detection.pm01_gamepad_bridge:main",
        ],
    },
)
