import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rc_safe_corridor"),
        "config",
        "safe_corridor_hardware.yaml",
    )
    publish_markers = LaunchConfiguration("publish_markers")
    start_safety_monitor = LaunchConfiguration("start_safety_monitor")
    enable_disarm = LaunchConfiguration("enable_disarm")
    lane_gate_enabled = LaunchConfiguration("lane_gate_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_markers", default_value="true"),
            DeclareLaunchArgument("start_safety_monitor", default_value="true"),
            # Safety monitor luôn chạy; chỉ tắt lệnh disarm khi test perception
            # offline hoặc khi actuation chưa được khởi động.
            DeclareLaunchArgument("enable_disarm", default_value="true"),
            DeclareLaunchArgument("lane_gate_enabled", default_value="true"),
            Node(
                package="rc_safe_corridor",
                executable="obstacle_detector",
                name="obstacle_detector",
                output="screen",
                parameters=[
                    config,
                    {
                        "publish_markers": ParameterValue(publish_markers, value_type=bool),
                        "lane_gate_enabled": ParameterValue(lane_gate_enabled, value_type=bool),
                    },
                ],
            ),
            Node(
                package="rc_safe_corridor",
                executable="safe_corridor_planner",
                name="safe_corridor_planner",
                output="screen",
                parameters=[
                    config,
                    {"publish_markers": ParameterValue(publish_markers, value_type=bool)},
                ],
            ),
            Node(
                condition=IfCondition(start_safety_monitor),
                package="rc_safe_corridor",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                parameters=[
                    config,
                    {"enable_disarm": ParameterValue(enable_disarm, value_type=bool)},
                ],
            ),
        ]
    )
