from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration

import launch
from humanoid_common_mpc_ros2.mpc_launch_config import MPCLaunchConfig


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

    test_visualizer_node = launch_ros.actions.Node(
        package="g1_centroidal_mpc_ros2",
        executable="test_visualizer",
        name="test_visualizer",
        # prefix=['x-terminal-emulator -e gdb -ex run --args'],
        output="screen",
        arguments=[
            LaunchConfiguration("robot_name"),
            LaunchConfiguration("config_name"),
            LaunchConfiguration("target_command_file"),
            LaunchConfiguration("description_name"),
        ],
    )

    # Add parameters
    cfg.ld.add_action(cfg.declare_robot_name)
    cfg.ld.add_action(cfg.declare_config_file)
    cfg.ld.add_action(cfg.declare_target_command_file)
    cfg.ld.add_action(cfg.declare_gait_command_file)
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)

    # Add nodes
    cfg.ld.add_action(test_visualizer_node)
    cfg.ld.add_action(cfg.robot_state_publisher_node)
    cfg.ld.add_action(cfg.rviz_node)

    return cfg.ld
