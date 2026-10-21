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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "require_lock",
            default_value="true",
            description="Publish no target until /person/set_target lock.",
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
            launch_arguments={
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
            }.items(),
        ),
    ])
