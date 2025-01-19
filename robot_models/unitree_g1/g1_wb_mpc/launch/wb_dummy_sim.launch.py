from ament_index_python.packages import get_package_share_directory

import launch
from humanoid_common_mpc_ros2.mpc_launch_config import MPCLaunchConfig
from launch_ros.actions import Node


def generate_launch_description():

    cfg = MPCLaunchConfig(
        mpc_lib_pkg="humanoid_wb_mpc",
        mpc_config_pkg="g1_wb_mpc",
        mpc_model_pkg="g1_description",
        urdf_rel_path="/urdf/g1_29dof.urdf",
        robot_name="g1",
        solver="sqp",
        enable_debug=False,
    )

    # Add parameters
    cfg.ld.add_action(cfg.declare_robot_name)
    cfg.ld.add_action(cfg.declare_config_file)
    cfg.ld.add_action(cfg.declare_target_command_file)
    cfg.ld.add_action(cfg.declare_gait_command_file)
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)

    # Add nodes
    cfg.ld.add_action(cfg.mpc_node)
    cfg.ld.add_action(cfg.dummy_sim_node)
    cfg.ld.add_action(cfg.robot_state_publisher_node)
    cfg.ld.add_action(cfg.terminal_robot_state_publisher_node)
    cfg.ld.add_action(cfg.target_robot_state_publisher_node)
    cfg.ld.add_action(cfg.rviz_node)
    cfg.ld.add_action(cfg.base_velocity_controller_gui_node)

    return cfg.ld
