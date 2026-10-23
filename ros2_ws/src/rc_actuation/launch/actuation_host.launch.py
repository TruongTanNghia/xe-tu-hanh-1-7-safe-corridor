import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rc_actuation"),
        "config",
        "actuation_hardware.yaml",
    )
    serial_device = LaunchConfiguration("serial_device")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-STMicroelectronics_STM32_STLink_"
                    "066EFF495365495067184632-if02"
                ),
            ),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=[
                    "serial",
                    "--dev",
                    serial_device,
                    "-b",
                    "115200",
                ],
            ),
            Node(
                package="rc_actuation",
                executable="actuation_manager",
                name="actuation_manager",
                output="screen",
                parameters=[config],
            ),
        ]
    )
