"""One-command person follow: camera + YOLO + controller + armed safety bridge.

Pipeline:
    yolo_person_3d (ros2_ws, optional via start_vision)
      -> /person/position_robot (base_link)
    person_follow_controller
      -> /vision_follow/cmd_vel (isolated, never drives the robot directly)
    cmd_vel_safety_bridge (armed + motion-state + freshness gating)
      -> /cmd_vel -> PM01 locomotion (运控主机 ros2_control_node subscribes here)

    body_vel_bridge (legacy, kept but inert): /motion/body_vel_cmd has zero
      subscribers on this SDK — verified 2026-08-31. The real velocity input
      is /cmd_vel (geometry_msgs/Twist).

The unified workspace must be sourced first:
    source /opt/ros/humble/setup.bash
    source ~/pm01_ws/install/setup.bash
    # or simply: source ~/pm01_ws/env.sh

Run (unarmed first, verify, then arm):
    ros2 launch pm01_aruco_detection person_follow.launch.py armed:=false
    ros2 launch pm01_aruco_detection person_follow.launch.py \
        armed:=true allowed_motion_state:=walk require_motion_state:=false

start_vision:=true additionally includes yolo_person_3d bringup (camera +
YOLO) for a one-command cold start; the default false expects perception to
run separately, as in the staged bring-up procedure.

require_motion_state:=true additionally gates on /motion/motion_state being
fresh and equal to allowed_motion_state, and needs interface_protocol sourced.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_vision",
                default_value="false",
                description="Also start camera + YOLO person 3D (ros2_ws). "
                "Keep false when perception already runs in another terminal "
                "(default staged bring-up); true only for one-command cold start.",
            ),
            DeclareLaunchArgument(
                "armed",
                default_value="false",
                description="Safety bridge gate. false = bridge stays BLOCKED "
                "and never publishes /cmd_vel.",
            ),
            DeclareLaunchArgument(
                "require_motion_state",
                default_value="false",
                description="Gate on /motion/motion_state being fresh and equal "
                "to allowed_motion_state (needs interface_protocol).",
            ),
            DeclareLaunchArgument("allowed_motion_state", default_value="walk"),
            DeclareLaunchArgument(
                "enable_forward",
                default_value="false",
                description="Only gates the legacy body_vel_bridge (dead topic "
                "on this SDK, kept for compatibility). It does NOT gate the "
                "real /cmd_vel chain — the safety bridge forwards forward "
                "motion whenever armed.",
            ),
            DeclareLaunchArgument(
                "voice_control",
                default_value="true",
                description="Subscribe /vision_follow/set_follow (std_msgs/Bool) "
                "so the voice stack can arm/disarm following at runtime "
                "('跟着我' / '停止跟随').",
            ),
            DeclareLaunchArgument("target_topic", default_value="/person/position_robot"),
            DeclareLaunchArgument("desired_distance_m", default_value="0.8"),
            DeclareLaunchArgument("min_distance_m", default_value="0.5"),
            # 运控侧 rl_walking_example 配置有 stop_vel_threshold: 0.3,指令低于
            # 0.3 一律当"停下"处理;限幅必须高于它,否则机器人永远不迈步。
            DeclareLaunchArgument("max_linear_mps", default_value="0.45"),
            DeclareLaunchArgument("max_angular_rps", default_value="0.6"),
            DeclareLaunchArgument("linear_gain", default_value="0.50"),
            DeclareLaunchArgument("angular_gain", default_value="1.20"),
            DeclareLaunchArgument("turn_in_place_angle_rad", default_value="0.60"),
            DeclareLaunchArgument("target_timeout_sec", default_value="0.50"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("yolo_person_3d"),
                            "launch",
                            "bringup.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_vision")),
            ),
            Node(
                package="pm01_aruco_detection",
                executable="person_follow_controller",
                name="person_follow_controller",
                output="screen",
                parameters=[
                    {
                        "target_topic": LaunchConfiguration("target_topic"),
                        "cmd_topic": "/vision_follow/cmd_vel",
                        "desired_distance_m": LaunchConfiguration("desired_distance_m"),
                        "min_distance_m": LaunchConfiguration("min_distance_m"),
                        "max_linear_mps": LaunchConfiguration("max_linear_mps"),
                        "max_angular_rps": LaunchConfiguration("max_angular_rps"),
                        "linear_gain": LaunchConfiguration("linear_gain"),
                        "angular_gain": LaunchConfiguration("angular_gain"),
                        "turn_in_place_angle_rad": LaunchConfiguration(
                            "turn_in_place_angle_rad"
                        ),
                        "target_timeout_sec": LaunchConfiguration("target_timeout_sec"),
                    }
                ],
            ),
            Node(
                package="pm01_aruco_detection",
                executable="cmd_vel_safety_bridge",
                name="pm01_cmd_vel_safety_bridge",
                output="screen",
                parameters=[
                    {
                        "armed": LaunchConfiguration("armed"),
                        "voice_control": LaunchConfiguration("voice_control"),
                        "require_motion_state": LaunchConfiguration(
                            "require_motion_state"
                        ),
                        "allowed_motion_state": LaunchConfiguration(
                            "allowed_motion_state"
                        ),
                        "max_forward_mps": LaunchConfiguration("max_linear_mps"),
                        "max_yaw_rps": LaunchConfiguration("max_angular_rps"),
                    }
                ],
            ),
            # 注意:/motion/body_vel_cmd 在本机运控 SDK 上是死话题(订阅者 0,
            # 2026-08-31 实测),真速度入口是 /cmd_vel(ros2_control_node 订阅)。
            # 本桥仅为兼容保留,不参与真机驱动链。enable_forward 只影响本桥
            # 输出,对 /cmd_vel 链路无作用。
            Node(
                package="pm01_aruco_detection",
                executable="body_vel_bridge",
                name="pm01_body_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/vision_follow/cmd_vel",
                        "armed": LaunchConfiguration("armed"),
                        "allowed_motion_state": LaunchConfiguration(
                            "allowed_motion_state"
                        ),
                        "enable_forward": LaunchConfiguration("enable_forward"),
                        "max_forward_mps": LaunchConfiguration("max_linear_mps"),
                        "max_yaw_rps": LaunchConfiguration("max_angular_rps"),
                    }
                ],
            ),
        ]
    )
