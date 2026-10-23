import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    package_share = get_package_share_directory("rc_localization")
    bno_config = os.path.join(
        package_share,
        "config",
        "bno055_hardware.yaml",
    )
    core_launch = os.path.join(
        package_share,
        "launch",
        "fusion_core.launch.py",
    )
    extrinsics_config = os.path.join(
        package_share,
        "config",
        "sensor_extrinsics.yaml",
    )
    with open(extrinsics_config, encoding="utf-8") as stream:
        extrinsics_data = yaml.safe_load(stream)

    start_lidar = LaunchConfiguration("start_lidar")
    start_imu = LaunchConfiguration("start_imu")
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
    extrinsic_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=extrinsic_defaults[name],
            description=(
                "PROVISIONAL base_link sensor extrinsic; "
                "measure before rolling tests"
            ),
        )
        for name in extrinsic_names
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_"
                    "Controller_0001-if00-port0"
                ),
            ),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyAMA0"),
            *extrinsic_arguments,
            LogInfo(
                msg=(
                    "Sensor extrinsics status: "
                    f"{extrinsics_data['status']}"
                )
            ),
            Node(
                condition=IfCondition(start_lidar),
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": lidar_port,
                        "serial_baudrate": 115200,
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                output="screen",
                arguments=[
                    "--x", extrinsics["lidar_x"],
                    "--y", extrinsics["lidar_y"],
                    "--z", extrinsics["lidar_z"],
                    "--roll", extrinsics["lidar_roll"],
                    "--pitch", extrinsics["lidar_pitch"],
                    "--yaw", extrinsics["lidar_yaw"],
                    "--frame-id", "base_link",
                    "--child-frame-id", "laser",
                ],
            ),
            Node(
                condition=IfCondition(start_imu),
                package="rc_localization",
                executable="bno055_safe_node",
                name="bno055",
                output="screen",
                parameters=[bno_config, {"uart_port": imu_port}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                output="screen",
                arguments=[
                    "--x", extrinsics["imu_x"],
                    "--y", extrinsics["imu_y"],
                    "--z", extrinsics["imu_z"],
                    "--roll", extrinsics["imu_roll"],
                    "--pitch", extrinsics["imu_pitch"],
                    "--yaw", extrinsics["imu_yaw"],
                    "--frame-id", "base_link",
                    "--child-frame-id", "imu_link",
                ],
            ),
            Node(
                condition=IfCondition(start_lidar),
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odometry/rf2o_raw",
                        "publish_tf": False,
                        "base_frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "init_pose_from_topic": "",
                        "freq": 20.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(core_launch),
            ),
        ]
    )
