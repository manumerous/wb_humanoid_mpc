from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="remote_control",
                executable="xbox_velocity_publisher",
                name="xbox_velocity_publisher",
                output="screen",
            )
        ]
    )
