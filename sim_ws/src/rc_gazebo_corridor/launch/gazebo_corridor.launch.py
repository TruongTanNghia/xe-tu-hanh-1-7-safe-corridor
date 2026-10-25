"""Gazebo Classic 11 demo of the Safe Corridor stack on the 1/7 car.

    ros2 launch rc_gazebo_corridor gazebo_corridor.launch.py scenario:=tour
    scenarios: clear | right | left | slalom | curve | block | lead | mixed | tour

Pipeline (same nodes as on the real car, only the sensors are simulated):
  Gazebo ray sensor /scan --> obstacle_detector --> safe_corridor_planner
  ground truth -> /odometry/filtered, /lane/reference (gazebo_corridor_bridge)
  /planning/corridor --> corridor_follower (/cmd_vel) --> ackermann_cmd_bridge
  --> ros2_control steering/wheel controllers in Gazebo.
  safety_monitor runs alongside (disarm disabled: no real actuation here).
"""

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rc_gazebo_corridor.world_builder import build_world


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _robot_description(controllers_yaml):
    xacro_path = os.path.join(
        get_package_share_directory("rc_gazebo_corridor"), "urdf", "rc_1_7_lidar.urdf.xacro"
    )
    result = subprocess.run(
        ["xacro", xacro_path, "use_ros2_control:=true", f"controllers_file:={controllers_yaml}"],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr}")
    xml = re.sub(r"<\?xml[^?]*\?>\s*", "", result.stdout)
    return re.sub(r"<!--.*?-->\s*", "", xml, flags=re.DOTALL)


def _gazebo_env():
    model_paths = [
        os.path.dirname(get_package_share_directory("rc_description")),
        "/usr/share/gazebo-11/models",
    ]
    return {
        "GAZEBO_MODEL_PATH": os.pathsep.join(model_paths),
        "GAZEBO_MODEL_DATABASE_URI": "",
        "LIBGL_DRI3_DISABLE": "1",
    }


def _setup(context, *_args, **_kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    gui = _as_bool(LaunchConfiguration("gui").perform(context))
    rviz = _as_bool(LaunchConfiguration("rviz").perform(context))
    controller = LaunchConfiguration("controller").perform(context)

    corridor_share = get_package_share_directory("rc_safe_corridor")
    gazebo_share = get_package_share_directory("rc_gazebo_corridor")
    corridor_config = os.path.join(corridor_share, "config", "safe_corridor_hardware.yaml")
    controllers_yaml = os.path.join(get_package_share_directory("rc_control"), "config", "controllers.yaml")
    rviz_config = os.path.join(gazebo_share, "config", "gazebo_corridor.rviz")

    world_path = os.path.join(tempfile.gettempdir(), f"safe_corridor_{scenario}.world")
    with open(world_path, "w", encoding="utf-8") as stream:
        stream.write(build_world(scenario))

    sim_time = {"use_sim_time": True}
    env = _gazebo_env()

    gzserver = ExecuteProcess(
        cmd=["gzserver", world_path, "-slibgazebo_ros_init.so", "-slibgazebo_ros_factory.so",
             "-slibgazebo_ros_force_system.so"],
        output="screen", additional_env=env,
    )
    gzclient = ExecuteProcess(cmd=["gzclient"], output="screen", additional_env=env)

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[sim_time, {"robot_description": _robot_description(controllers_yaml)}],
    )
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-topic", "robot_description", "-entity", "rc_car",
                   "-x", "0", "-y", "0", "-z", "0.0", "-Y", "0", "-timeout", "60"],
    )

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "-c", "/controller_manager"])

    ackermann_bridge = Node(
        package="rc_control", executable="ackermann_cmd_bridge.py", name="ackermann_cmd_bridge",
        output="screen",
        parameters=[sim_time, {
            "wheelbase": 0.375, "track_width": 0.260, "wheel_radius": 0.049, "mass": 6.5,
            "max_steering_angle": 0.524, "max_speed": 1.0, "cmd_timeout": 0.5,
            "publish_rate": 50.0, "odom_topic": "/odometry/filtered",
            "nmpc_longitudinal_mode": "duty_to_speed", "duty_to_speed_gain": 0.6,
            "max_acceleration": 1.0,
        }],
    )
    bridge = Node(
        package="rc_gazebo_corridor", executable="gazebo_corridor_bridge", output="screen",
        parameters=[sim_time, {"scenario": scenario}],
    )
    detector = Node(
        package="rc_safe_corridor", executable="obstacle_detector", name="obstacle_detector",
        output="screen", parameters=[corridor_config, sim_time, {"use_tf": True}],
    )
    planner = Node(
        package="rc_safe_corridor", executable="safe_corridor_planner", name="safe_corridor_planner",
        output="screen", parameters=[corridor_config, sim_time],
    )
    safety = Node(
        package="rc_safe_corridor", executable="safety_monitor", name="safety_monitor",
        output="screen", parameters=[corridor_config, sim_time, {"use_tf": True, "enable_disarm": False}],
    )
    follower = Node(
        package="rc_safe_corridor", executable="corridor_follower", name="corridor_follower",
        output="screen", parameters=[sim_time],
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config], parameters=[sim_time],
    )

    actions = [gzserver, robot_state_publisher, TimerAction(period=3.0, actions=[spawn])]
    if gui:
        actions.append(gzclient)
    actions += [
        TimerAction(period=7.0, actions=[spawner("joint_state_broadcaster")]),
        TimerAction(period=8.0, actions=[spawner("front_steering_position_controller")]),
        TimerAction(period=8.5, actions=[spawner("rear_wheel_velocity_controller")]),
        TimerAction(period=9.0, actions=[ackermann_bridge, bridge]),
        TimerAction(period=10.0, actions=[detector, planner, safety]),
    ]
    if controller == "follower":
        actions.append(TimerAction(period=11.0, actions=[follower]))
    if rviz:
        actions.append(TimerAction(period=4.0, actions=[rviz_node]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
        SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
        DeclareLaunchArgument("scenario", default_value="tour"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        # follower: corridor_follower (no acados). nmpc: start race_cars NMPC
        # yourself (it publishes /nmpc_cmd which the ackermann bridge accepts).
        DeclareLaunchArgument("controller", default_value="follower"),
        OpaqueFunction(function=_setup),
    ])
