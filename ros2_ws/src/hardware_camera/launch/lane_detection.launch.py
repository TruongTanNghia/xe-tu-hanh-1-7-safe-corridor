import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_share = get_package_share_directory("hardware_camera")
    calibration_file = os.path.join(
        camera_share,
        "config",
        "imx708_wide_4608x2592.yaml",
    )

    manual_controls = LaunchConfiguration("manual_controls")
    exposure_time_us = LaunchConfiguration("exposure_time_us")
    analogue_gain = LaunchConfiguration("analogue_gain")
    colour_gain_r = LaunchConfiguration("colour_gain_r")
    colour_gain_b = LaunchConfiguration("colour_gain_b")
    boundary_corridor_enabled = LaunchConfiguration(
        "boundary_corridor_enabled"
    )
    boundary_margin_px = LaunchConfiguration("boundary_margin_px")
    boundary_recovery_enabled = LaunchConfiguration(
        "boundary_recovery_enabled"
    )
    boundary_recovery_max_age_ms = LaunchConfiguration(
        "boundary_recovery_max_age_ms"
    )
    boundary_recovery_min_observations = LaunchConfiguration(
        "boundary_recovery_min_observations"
    )
    boundary_search_left_ratio = LaunchConfiguration(
        "boundary_search_left_ratio"
    )
    boundary_search_right_ratio = LaunchConfiguration(
        "boundary_search_right_ratio"
    )
    scan_top_ratio = LaunchConfiguration("scan_top_ratio")
    scan_bot_ratio = LaunchConfiguration("scan_bot_ratio")
    scan_left_ratio = LaunchConfiguration("scan_left_ratio")
    scan_right_ratio = LaunchConfiguration("scan_right_ratio")

    return LaunchDescription([
        DeclareLaunchArgument("manual_controls", default_value="false"),
        DeclareLaunchArgument("exposure_time_us", default_value="8000"),
        DeclareLaunchArgument("analogue_gain", default_value="1.0"),
        DeclareLaunchArgument("colour_gain_r", default_value="1.5"),
        DeclareLaunchArgument("colour_gain_b", default_value="1.5"),
        DeclareLaunchArgument(
            "boundary_corridor_enabled", default_value="true"
        ),
        DeclareLaunchArgument("boundary_margin_px", default_value="4"),
        DeclareLaunchArgument(
            "boundary_recovery_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "boundary_recovery_max_age_ms", default_value="150"
        ),
        DeclareLaunchArgument(
            "boundary_recovery_min_observations", default_value="3"
        ),
        DeclareLaunchArgument(
            "boundary_search_left_ratio", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "boundary_search_right_ratio", default_value="1.0"
        ),
        DeclareLaunchArgument("scan_top_ratio", default_value="0.40"),
        DeclareLaunchArgument("scan_bot_ratio", default_value="0.63"),
        DeclareLaunchArgument("scan_left_ratio", default_value="0.0"),
        DeclareLaunchArgument("scan_right_ratio", default_value="1.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_camera_optical",
            arguments=[
                # Updated mechanical measurement: camera moved 13 cm forward
                # relative to the previous base_link pose.
                "--x", "0.276",
                "--y", "0",
                # Updated mechanical measurement: camera is 4.5 cm lower.
                "--z", "0.160",
                "--qx", "-0.61237265",
                "--qy", "0.61237265",
                "--qz", "-0.35355302",
                "--qw", "0.35355302",
                "--frame-id", "base_link",
                "--child-frame-id", "camera_optical_frame",
            ],
        ),
        Node(
            package="hardware_camera",
            executable="camera_node",
            name="hardware_camera",
            output="screen",
            parameters=[{
                "calibration_file": calibration_file,
                "width": 640,
                "height": 360,
                "frame_rate": 30.0,
                "sensor_width": 2304,
                "sensor_height": 1296,
                "frame_id": "camera_optical_frame",
                "manual_controls": ParameterValue(
                    manual_controls,
                    value_type=bool,
                ),
                "exposure_time_us": ParameterValue(
                    exposure_time_us,
                    value_type=int,
                ),
                "analogue_gain": ParameterValue(
                    analogue_gain,
                    value_type=float,
                ),
                "colour_gain_r": ParameterValue(
                    colour_gain_r,
                    value_type=float,
                ),
                "colour_gain_b": ParameterValue(
                    colour_gain_b,
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="image_processing",
            executable="image_processor",
            name="image_processor",
            output="screen",
            parameters=[{
                "boundary_corridor_enabled": ParameterValue(
                    boundary_corridor_enabled,
                    value_type=bool,
                ),
                "boundary_margin_px": ParameterValue(
                    boundary_margin_px,
                    value_type=int,
                ),
                "boundary_recovery_enabled": ParameterValue(
                    boundary_recovery_enabled,
                    value_type=bool,
                ),
                "boundary_recovery_max_age_ms": ParameterValue(
                    boundary_recovery_max_age_ms,
                    value_type=int,
                ),
                "boundary_recovery_min_observations": ParameterValue(
                    boundary_recovery_min_observations,
                    value_type=int,
                ),
                "boundary_search_left_ratio": ParameterValue(
                    boundary_search_left_ratio,
                    value_type=float,
                ),
                "boundary_search_right_ratio": ParameterValue(
                    boundary_search_right_ratio,
                    value_type=float,
                ),
                "scan_top_ratio": ParameterValue(
                    scan_top_ratio,
                    value_type=float,
                ),
                "scan_bot_ratio": ParameterValue(
                    scan_bot_ratio,
                    value_type=float,
                ),
                "scan_left_ratio": ParameterValue(
                    scan_left_ratio,
                    value_type=float,
                ),
                "scan_right_ratio": ParameterValue(
                    scan_right_ratio,
                    value_type=float,
                ),
            }],
        ),
    ])
