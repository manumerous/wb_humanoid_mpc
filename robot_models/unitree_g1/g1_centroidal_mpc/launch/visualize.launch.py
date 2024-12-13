import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch_ros
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    cfg = MPCLaunchConfig(
        mpc_lib_pkg="humanoid_centroidal_mpc",
        mpc_config_pkg="g1_centroidal_mpc",
        mpc_model_pkg="g1_description",
        urdf_rel_path="/urdf/g1_29dof.urdf",
        robot_name="g1",
        solver="sqp",
        enable_debug=False,
    )

    # Add parameter
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)

    # Add nodes
    cfg.ld.add_action(cfg.rviz_node)

    return ld
