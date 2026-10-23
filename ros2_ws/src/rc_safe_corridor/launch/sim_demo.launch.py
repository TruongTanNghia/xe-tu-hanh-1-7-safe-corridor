"""Real-time demo without hardware: simulator + Safe Corridor stack + RViz2.

    ros2 launch rc_safe_corridor sim_demo.launch.py scenario:=slalom
    scenarios: clear | right | left | slalom | curve | block | lead | mixed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("rc_safe_corridor")
    config = os.path.join(share, "config", "safe_corridor_hardware.yaml")
    rviz_config = os.path.join(share, "config", "sim_demo.rviz")
    scenario = LaunchConfiguration("scenario")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="slalom"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="rc_safe_corridor",
                executable="sim_world",
                name="sim_world",
                output="screen",
                parameters=[{"scenario": ParameterValue(scenario, value_type=str)}],
            ),
            Node(
                package="rc_safe_corridor",
                executable="obstacle_detector",
                name="obstacle_detector",
                output="screen",
                parameters=[config, {"use_tf": False}],
            ),
            Node(
                package="rc_safe_corridor",
                executable="safe_corridor_planner",
                name="safe_corridor_planner",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rc_safe_corridor",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                parameters=[config, {"use_tf": False, "enable_disarm": False}],
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="log",
            ),
        ]
    )
