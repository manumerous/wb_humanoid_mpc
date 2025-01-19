import os

from ament_index_python.packages import get_package_share_directory

import launch_ros
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def extract_constant_from_cpp(file_path, constant_name):
    with open(file_path, "r") as file:
        for line in file:
            if constant_name in line:
                # Assuming the constant is defined as a std::string
                parts = line.split("=")
                if len(parts) >= 2:
                    # Extracting the string between double quotes
                    string_parts = parts[1].split('"')
                    if len(string_parts) >= 2:
                        return string_parts[1].strip()  # Remove whitespace if any
    return None


class MPCLaunchConfig:

    def __init__(
        self,
        mpc_lib_pkg: str,
        mpc_config_pkg: str,
        mpc_model_pkg: str,
        urdf_rel_path: str,
        robot_name: str,
        enable_debug: bool = False,
        solver: str = "sqp",
    ):

        self.solver = solver
        self.mpc_lib_pkg: str = mpc_lib_pkg
        self.mpc_lib_pkg_ros2: str = mpc_lib_pkg + "_ros2"
        self.mpc_config_pkg_dir: str = get_package_share_directory(mpc_config_pkg)

        self.common_mpc_dir = get_package_share_directory("humanoid_common_mpc")
        self.mpc_dir = get_package_share_directory(self.mpc_lib_pkg)
        self.mpc_ros2_dir = get_package_share_directory(self.mpc_lib_pkg_ros2)

        self.urdf_path = get_package_share_directory(mpc_model_pkg) + urdf_rel_path

        ### MPC Config ###
        default_mpc_config_path = os.path.join(
            self.mpc_config_pkg_dir, "config/mpc/task.info"
        )
        default_target_command_path = os.path.join(
            self.mpc_config_pkg_dir, "config/command/reference.info"
        )
        default_gait_command_path = os.path.join(
            self.common_mpc_dir, "config/command/gait.info"
        )

        print("MPC config urdf file path: ", self.urdf_path)

        ### RVIZ Config ###
        default_rviz_config_path = os.path.join(
            get_package_share_directory("humanoid_common_mpc_ros2"),
            "rviz/humanoid.rviz",
        )

        ### Termianl Prefix ###
        if enable_debug:
            self.always_terminal_prefix = ["x-terminal-emulator -e gdb -ex run --args"]
            self.terminal_prefix = ["x-terminal-emulator -e gdb -ex run --args"]
        else:
            self.always_terminal_prefix = ["x-terminal-emulator -e"]
            self.terminal_prefix = []

        #############
        ### Nodes ###
        #############

        self.ld = launch.LaunchDescription()

        mpc_node_exec = self.mpc_lib_pkg + "_" + self.solver + "_node"
        self.mpc_node = launch_ros.actions.Node(
            package=self.mpc_lib_pkg_ros2,
            executable=mpc_node_exec,
            prefix=self.terminal_prefix,
            name=mpc_node_exec,
            arguments=[
                LaunchConfiguration("robot_name"),
                LaunchConfiguration("config_name"),
                LaunchConfiguration("target_command_file"),
                LaunchConfiguration("description_name"),
                LaunchConfiguration("target_gait_file"),
            ],
        )

        dummy_sim_node_exec = self.mpc_lib_pkg + "_dummy_sim_node"
        self.dummy_sim_node = launch_ros.actions.Node(
            package=self.mpc_lib_pkg_ros2,
            executable=dummy_sim_node_exec,
            prefix=self.terminal_prefix,
            name=dummy_sim_node_exec,
            arguments=[
                LaunchConfiguration("robot_name"),
                LaunchConfiguration("config_name"),
                LaunchConfiguration("target_command_file"),
                LaunchConfiguration("description_name"),
                LaunchConfiguration("target_gait_file"),
            ],
        )

        self.gait_keyboard_command_node = launch_ros.actions.Node(
            package="humanoid_common_mpc_ros2",
            executable="gait_keyboard_command_node",
            prefix=self.always_terminal_prefix,
            name="gait_keyboard_command_node",
            output="screen",
            arguments=[
                LaunchConfiguration("robot_name"),
                LaunchConfiguration("target_gait_file"),
            ],
        )

        self.velocity_keyboard_command_node = launch_ros.actions.Node(
            package="humanoid_common_mpc_ros2",
            executable="velocity_keyboard_command_node",
            prefix=self.always_terminal_prefix,
            name="velocity_keyboard_command_node",
            output="screen",
            arguments=[
                LaunchConfiguration("robot_name"),
                LaunchConfiguration("config_name"),
                LaunchConfiguration("target_command_file"),
                LaunchConfiguration("description_name"),
                LaunchConfiguration("target_gait_file"),
            ],
        )

        pose_command_node_exec = self.mpc_lib_pkg + "_pose_command_node"
        self.keyboard_pose_command_node = launch_ros.actions.Node(
            package=self.mpc_lib_pkg_ros2,
            executable=pose_command_node_exec,
            prefix=self.always_terminal_prefix,
            name=pose_command_node_exec,
            output="screen",
            arguments=[
                LaunchConfiguration("robot_name"),
                LaunchConfiguration("config_name"),
                LaunchConfiguration("target_command_file"),
                LaunchConfiguration("description_name"),
                LaunchConfiguration("target_gait_file"),
            ],
        )

        self.rviz_node = launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            prefix=self.terminal_prefix,
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rvizconfig")],
        )

        self.robot_state_publisher_node = launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(
                        ["xacro ", LaunchConfiguration("description_name")]
                    ),
                    "publish_frequency": 180.0,
                }
            ],
        )

        self.terminal_robot_state_publisher_node = launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="terminal_robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(
                        ["xacro ", LaunchConfiguration("description_name")]
                    ),
                    "frame_prefix": "terminal_state/",
                    "publish_frequency": 180.0,
                }
            ],
            remappings=[
                (
                    "/joint_states",
                    "/terminal_state/joint_states",
                )  # Remap the joint_states topic
            ],
        )

        self.target_robot_state_publisher_node = launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="target_robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(
                        ["xacro ", LaunchConfiguration("description_name")]
                    ),
                    "frame_prefix": "terminal_target/",
                    "publish_frequency": 180.0,
                }
            ],
            remappings=[
                (
                    "/joint_states",
                    "/terminal_target/joint_states",
                )  # Remap the joint_states topic
            ],
        )

        self.base_velocity_controller_gui_node = launch_ros.actions.Node(
            package="remote_control",
            executable="base_velocity_controller_gui",
            name="base_velocity_controller_gui",
            output="screen",
        )

        self.mpc_observation_logger_node = launch_ros.actions.Node(
            package="humanoid_common_mpc_pyutils",
            executable="mpc_observation_logger",
            name="mpc_observation_logger",
            prefix=self.always_terminal_prefix,
            output="screen",
        )

        #########################
        ### Lauinch Arguments ###
        #########################

        self.declare_robot_name = DeclareLaunchArgument(
            "robot_name",
            default_value=robot_name,
            description="Name of the robot",
        )
        self.declare_config_file = DeclareLaunchArgument(
            "config_name",
            default_value=default_mpc_config_path,
            description="Path to MPC config file",
        )
        self.declare_target_command_file = DeclareLaunchArgument(
            "target_command_file",
            default_value=default_target_command_path,
            description="Path to MPC target reference file",
        )
        self.declare_urdf_path = DeclareLaunchArgument(
            "description_name",
            default_value=self.urdf_path,
            description="Absolute path to robot urdf file",
        )
        self.declare_gait_command_file = DeclareLaunchArgument(
            "target_gait_file",
            default_value=default_gait_command_path,
            description="Path to MPC gait reference file",
        )
        self.declare_rviz_config_path = DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )

        print("Finished launch config initialization")
