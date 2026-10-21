import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_model_path() -> str:
    try:
        return os.path.join(
            get_package_share_directory("yolo_person_3d"),
            "weights", "yolo26n.engine",
        )
    except Exception:
        return os.path.normpath(os.path.join(
            os.path.dirname(__file__), "..", "weights", "yolo26n.engine",
        ))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=_default_model_path(),
        ),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("camera_mount_x_m", default_value="0.0"),
        DeclareLaunchArgument("camera_mount_y_m", default_value="0.0"),
        DeclareLaunchArgument("camera_mount_z_m", default_value="0.0"),
        DeclareLaunchArgument("camera_mount_pitch_deg", default_value="16.1"),
        DeclareLaunchArgument(
            "require_lock",
            default_value="true",
            description="Publish no target until /person/set_target lock; "
            "then follow only that person.",
        ),
        DeclareLaunchArgument(
            "target_command_topic", default_value="/person/set_target"),
        DeclareLaunchArgument("lock_center_weight", default_value="1.0"),
        DeclareLaunchArgument("lock_depth_weight", default_value="0.35"),
        DeclareLaunchArgument(
            "lock_track_color_max_dist", default_value="0.55"),
        DeclareLaunchArgument(
            "lock_reacquire_color_max_dist", default_value="0.40"),
        DeclareLaunchArgument(
            "lock_reacquire_confirm_frames", default_value="2"),
        DeclareLaunchArgument("lock_gap_reset_sec", default_value="0.5"),
        Node(
            package="yolo_person_3d",
            executable="person_3d_node",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "target_frame": LaunchConfiguration("target_frame"),
                "show_window": LaunchConfiguration("show_window"),
                "device": LaunchConfiguration("device"),
                "camera_mount_x_m": LaunchConfiguration("camera_mount_x_m"),
                "camera_mount_y_m": LaunchConfiguration("camera_mount_y_m"),
                "camera_mount_z_m": LaunchConfiguration("camera_mount_z_m"),
                "camera_mount_pitch_deg": LaunchConfiguration("camera_mount_pitch_deg"),
                "require_lock": LaunchConfiguration("require_lock"),
                "target_command_topic": LaunchConfiguration(
                    "target_command_topic"),
                "lock_center_weight": LaunchConfiguration(
                    "lock_center_weight"),
                "lock_depth_weight": LaunchConfiguration("lock_depth_weight"),
                "lock_track_color_max_dist": LaunchConfiguration(
                    "lock_track_color_max_dist"),
                "lock_reacquire_color_max_dist": LaunchConfiguration(
                    "lock_reacquire_color_max_dist"),
                "lock_reacquire_confirm_frames": LaunchConfiguration(
                    "lock_reacquire_confirm_frames"),
                "lock_gap_reset_sec": LaunchConfiguration(
                    "lock_gap_reset_sec"),
            }],
        ),
    ])
