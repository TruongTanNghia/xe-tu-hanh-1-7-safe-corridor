import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    bringup_share = get_package_share_directory("rc_bringup")
    sensors_launch = os.path.join(
        bringup_share,
        "launch",
        "sensors_only.launch.py",
    )
    nmpc_launch = os.path.join(
        get_package_share_directory("race_cars"),
        "launch",
        "nmpc_shadow.launch.py",
    )
    extrinsics_config = os.path.join(
        get_package_share_directory("rc_localization"),
        "config",
        "sensor_extrinsics.yaml",
    )
    with open(extrinsics_config, encoding="utf-8") as stream:
        extrinsics_data = yaml.safe_load(stream)

    start_camera = LaunchConfiguration("start_camera")
    start_lidar = LaunchConfiguration("start_lidar")
    start_imu = LaunchConfiguration("start_imu")
    start_nmpc = LaunchConfiguration("start_nmpc")
    lidar_port = LaunchConfiguration("lidar_port")
    imu_port = LaunchConfiguration("imu_port")
    extrinsic_names = (
        "lidar_x", "lidar_y", "lidar_z",
        "lidar_roll", "lidar_pitch", "lidar_yaw",
        "imu_x", "imu_y", "imu_z",
        "imu_roll", "imu_pitch", "imu_yaw",
    )
    extrinsic_defaults = {}
    for prefix, key in (
        ("lidar", "base_link_to_laser"),
        ("imu", "base_link_to_imu"),
    ):
        for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
            extrinsic_defaults[f"{prefix}_{axis}"] = str(
                extrinsics_data[key][axis]
            )
    extrinsics = {
        name: LaunchConfiguration(name)
        for name in extrinsic_names
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_nmpc", default_value="true"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_"
                    "Controller_0001-if00-port0"
                ),
            ),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyAMA0"),
            *[
                DeclareLaunchArgument(
                    name,
                    default_value=extrinsic_defaults[name],
                )
                for name in extrinsic_names
            ],
            LogInfo(
                msg=(
                    "Sensor extrinsics status: "
                    f"{extrinsics_data['status']}"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    "start_camera": start_camera,
                    "start_lidar": start_lidar,
                    "start_imu": start_imu,
                    "lidar_port": lidar_port,
                    "imu_port": imu_port,
                    **extrinsics,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nmpc_launch),
                condition=IfCondition(start_nmpc),
            ),
        ]
    )
