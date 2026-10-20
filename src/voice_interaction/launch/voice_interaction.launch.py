"""启动语音交互节点并加载参数。

用法：
  ros2 launch voice_interaction voice_interaction.launch.py
  ros2 launch voice_interaction voice_interaction.launch.py energy_threshold:=300
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('voice_interaction'),
        'config', 'params.yaml')

    params_file = LaunchConfiguration('params', default=default_params)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=default_params,
            description='参数文件路径'),
        Node(
            package='voice_interaction',
            executable='voice_node',
            name='voice_interaction',
            output='screen',
            parameters=[params_file],
        ),
    ])
