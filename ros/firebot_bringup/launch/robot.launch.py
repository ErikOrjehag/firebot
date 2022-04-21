
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="firebot_hw",
                executable="hw_node",
                name="hw_node",
                output="screen",
            ),
            Node(
                package="firebot_hw",
                executable="servo_node",
                name="servo_node",
                output="screen",
            ),
            Node(
                package="firebot_hw",
                executable="mic_node",
                name="mic_node",
                output="screen",
            ),
            Node(
                package="firebot_viz",
                executable="viz_node",
                name="viz_node",
                output="screen",
            ),
            Node(
                package="firebot_ai",
                executable="ai_node",
                name="ai_node",
                output="screen",
            ),
        ])
