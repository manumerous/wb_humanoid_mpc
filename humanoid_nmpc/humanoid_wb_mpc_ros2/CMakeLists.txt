cmake_minimum_required(VERSION 3.5)
project(humanoid_wb_mpc_ros2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ocs2_ros2_interfaces REQUIRED)
find_package(ocs2_ros2_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(humanoid_wb_mpc REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(humanoid_common_mpc_ros2 REQUIRED)
find_package(mujoco_sim_interface REQUIRED)


find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

# find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(glfw3 REQUIRED)

find_package(GLEW REQUIRED)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 20)

set(FLAGS ${OCS2_CXX_FLAGS} -Wno-invalid-partial-specialization)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


set(dependencies
  rclcpp
  ament_index_cpp
  tf2_ros
  tf2_eigen
  urdf
  kdl_parser
  visualization_msgs
  geometry_msgs
  sensor_msgs
  ocs2_ros2_interfaces
  ocs2_ros2_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_robotic_tools
  humanoid_wb_mpc
  ocs2_pinocchio_interface
  humanoid_common_mpc_ros2
  mujoco_sim_interface
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)


################
## Executable ##
################

# Dummy Simulator Node
add_executable(humanoid_wb_mpc_dummy_sim_node
  src/WBMpcDummySimNode.cpp
)
ament_target_dependencies(humanoid_wb_mpc_dummy_sim_node
  ${dependencies}
)
target_compile_options(humanoid_wb_mpc_dummy_sim_node PUBLIC ${FLAGS})

# MPC Node
add_executable(humanoid_wb_mpc_sqp_node
  src/WBMpcSqpNode.cpp
)
ament_target_dependencies(humanoid_wb_mpc_sqp_node
  ${dependencies}
)
target_compile_options(humanoid_wb_mpc_sqp_node PUBLIC ${FLAGS})

# Pose Command Node
add_executable(humanoid_wb_mpc_pose_command_node
  src/WBMpcPoseCommandNode.cpp
)
ament_target_dependencies(humanoid_wb_mpc_pose_command_node
  ${dependencies}
)
target_compile_options(humanoid_wb_mpc_pose_command_node PUBLIC ${FLAGS})

# Sim executable
add_executable(humanoid_wb_mpc_sim
  src/WBMpcRobotSim.cpp
)
ament_target_dependencies(humanoid_wb_mpc_sim
  ${dependencies}
)
target_link_libraries(humanoid_wb_mpc_sim
        absl::flat_hash_map
        glfw
        GLEW
)
target_compile_options(humanoid_wb_mpc_sim PUBLIC ${FLAGS})




#############
## Install ##
#############

# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include/
# )

install(TARGETS
  humanoid_wb_mpc_sqp_node
  humanoid_wb_mpc_dummy_sim_node
  humanoid_wb_mpc_pose_command_node
  humanoid_wb_mpc_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

