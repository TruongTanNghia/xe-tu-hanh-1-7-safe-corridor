"""Sensors + lane + localization + safe corridor stack + NMPC (shadow by default).

    ros2 launch rc_bringup safe_corridor_full.launch.py                # shadow
    ros2 launch rc_bringup safe_corridor_full.launch.py nmpc_shadow:=false

Actuation/micro-ROS is NOT started here (same rule as shadow_full.launch.py);
use actuation_disarmed.launch.py in a separate terminal for rolling tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("rc_bringup")
    sensors_launch = os.path.join(bringup_share, "launch", "sensors_only.launch.py")
    corridor_launch = os.path.join(
        get_package_share_directory("rc_safe_corridor"), "launch", "safe_corridor.launch.py"
    )
    nmpc_config = os.path.join(
        get_package_share_directory("race_cars"), "config", "nmpc_hardware.yaml"
    )
    acados_install = os.environ.get("RC_ACADOS_INSTALL", "/home/ac/acados")

    start_camera = LaunchConfiguration("start_camera")
    start_lidar = LaunchConfiguration("start_lidar")
    start_imu = LaunchConfiguration("start_imu")
    start_nmpc = LaunchConfiguration("start_nmpc")
    nmpc_shadow = LaunchConfiguration("nmpc_shadow")
    corridor_mode = LaunchConfiguration("corridor_mode")
    publish_markers = LaunchConfiguration("publish_markers")
    enable_disarm = LaunchConfiguration("enable_disarm")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_nmpc", default_value="true"),
            # Safe default: shadow. Active only inside a recorded gate session.
            DeclareLaunchArgument("nmpc_shadow", default_value="true"),
            DeclareLaunchArgument("corridor_mode", default_value="optional"),
            DeclareLaunchArgument("publish_markers", default_value="true"),
            DeclareLaunchArgument("enable_disarm", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    "start_camera": start_camera,
                    "start_lidar": start_lidar,
                    "start_imu": start_imu,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(corridor_launch),
                launch_arguments={
                    "publish_markers": publish_markers,
                    "enable_disarm": enable_disarm,
                }.items(),
            ),
            Node(
                condition=IfCondition(start_nmpc),
                package="race_cars",
                executable="lane_reference_adapter",
                name="lane_reference_adapter",
                output="screen",
                parameters=[nmpc_config],
            ),
            Node(
                condition=IfCondition(start_nmpc),
                package="race_cars",
                executable="racecar_nmpc_node",
                name="racecar_nmpc_node",
                output="screen",
                additional_env={
                    "ACADOS_SOURCE_DIR": acados_install,
                    "LD_LIBRARY_PATH": os.path.join(acados_install, "lib")
                    + ":"
                    + os.environ.get("LD_LIBRARY_PATH", ""),
                },
                parameters=[
                    nmpc_config,
                    {
                        "shadow_mode": ParameterValue(nmpc_shadow, value_type=bool),
                        "corridor_mode": ParameterValue(corridor_mode, value_type=str),
                    },
                ],
            ),
        ]
    )
