
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
                package="firebot_sim",
                executable="input_node",
                name="input_node",
                output="screen",
            ),
            Node(
                package="firebot_sim",
                executable="sim_node",
                name="sim_node",
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
