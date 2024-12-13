from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="humanoid_common_mpc_pyutils",
                executable="mpc_observation_logger",
                name="mpc_observation_logger",
                output="screen",
            )
        ]
    )
