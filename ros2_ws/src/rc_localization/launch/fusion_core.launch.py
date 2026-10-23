import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rc_localization")
    conditioner_config = os.path.join(
        package_share,
        "config",
        "velocity_conditioner.yaml",
    )
    ekf_config = os.path.join(
        package_share,
        "config",
        "ekf_hardware.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="rc_localization",
                executable="velocity_conditioner",
                name="rf2o_velocity_conditioner",
                output="screen",
                parameters=[conditioner_config],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config],
            ),
        ]
    )
