"""Visual pipeline bringup: Realsense camera + YOLO person 3D localization.

Pulls up the camera (pm01_camera_bringup, align_depth baked in) and the
yolo_person_3d node (TensorRT GPU inference via .engine) in one command. The
yolo node waits for camera topics, so starting them together is race-free.

Run:
    ros2 launch yolo_person_3d bringup.launch.py

Safety: this publishes /person/position_* on ROS_DOMAIN_ID=69, which may drive
visual_follow_controller. Ensure the robot is stable / e-stop ready first.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("pm01_camera_bringup"),
                    "launch",
                    "camera.launch.py",
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("yolo_person_3d"),
                    "launch",
                    "person_3d.launch.py",
                ])
            ),
        ),
    ])
