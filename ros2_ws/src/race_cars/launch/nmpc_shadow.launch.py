import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("race_cars"),
        "config",
        "nmpc_hardware.yaml",
    )
    acados_install = os.environ.get("RC_ACADOS_INSTALL", "/home/ac/acados")
    current_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    library_path = os.path.join(acados_install, "lib")
    if current_library_path:
        library_path = f"{library_path}:{current_library_path}"

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "ACADOS_SOURCE_DIR",
                acados_install,
            ),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                library_path,
            ),
            Node(
                package="race_cars",
                executable="lane_reference_adapter",
                name="lane_reference_adapter",
                output="screen",
                # This launch is the explicitly safe shadow profile. Keep it
                # independent from the hardware config's active test default.
                parameters=[config, {"shadow_mode": True}],
            ),
            Node(
                package="race_cars",
                executable="racecar_nmpc_node",
                name="racecar_nmpc_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
