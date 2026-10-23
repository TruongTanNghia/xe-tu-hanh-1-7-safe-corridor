import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processing',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='image_processing',
            executable='image_processor',
            name='image_processor',
            output='screen'
        )
    ])
