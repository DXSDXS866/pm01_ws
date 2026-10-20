"""Realsense D435i bringup for the PM01 visual stack.

Wraps realsense2_camera's rs_launch.py with this machine's serial and
align_depth enabled. `align_depth.enable` defaults to false in rs_launch.py, so
without this it had to be re-set every launch via
`ros2 param set /camera align_depth.enable true` (non-persistent).

Produces node /camera with topics /camera/{color,aligned_depth_to_color,...}/...
which is what yolo_person_3d subscribes to. camera_namespace is intentionally
empty so the node sits at the root (/camera), matching the existing setup.

Run:
    ros2 launch pm01_camera_bringup camera.launch.py
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
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ])
            ),
            launch_arguments={
                "camera_namespace": "",
                "camera_name": "camera",
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "align_depth.enable": "true",
            }.items(),
        ),
    ])
