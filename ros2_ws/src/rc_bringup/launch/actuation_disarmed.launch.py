import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    actuation_launch = os.path.join(
        get_package_share_directory("rc_actuation"),
        "launch",
        "actuation_host.launch.py",
    )
    serial_device = LaunchConfiguration("serial_device")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_device",
                description="Stable /dev/serial/by-id path for the STM32.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(actuation_launch),
                launch_arguments={"serial_device": serial_device}.items(),
            ),
        ]
    )
