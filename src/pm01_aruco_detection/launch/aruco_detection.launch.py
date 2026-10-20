from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("dictionary", default_value="DICT_4X4_50"),
            DeclareLaunchArgument("target_id", default_value="23"),
            DeclareLaunchArgument("marker_size_m", default_value="0.074"),
            Node(
                package="pm01_aruco_detection",
                executable="aruco_detector",
                name="aruco_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "dictionary": LaunchConfiguration("dictionary"),
                        "target_id": LaunchConfiguration("target_id"),
                        "marker_size_m": LaunchConfiguration("marker_size_m"),
                    }
                ],
            ),
        ]
    )
