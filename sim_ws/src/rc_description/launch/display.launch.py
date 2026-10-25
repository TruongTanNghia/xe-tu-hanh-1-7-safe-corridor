from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = LaunchConfiguration('xacro_file')
    default_xacro_file = PathJoinSubstitution([
        FindPackageShare('rc_description'),
        'urdf',
        'rc.urdf.xacro',
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('rc_description'),
        'rviz',
        'rc_display.rviz',
    ])
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'xacro_file',
            default_value=default_xacro_file,
            description='Path to the robot xacro file'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
