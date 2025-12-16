cmake_minimum_required(VERSION 3.5)
project(humanoid_centroidal_mpc_ros2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(humanoid_centroidal_mpc REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
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
find_package(absl REQUIRED)


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
  ocs2_mpc
  ocs2_sqp
  ocs2_robotic_tools
  humanoid_centroidal_mpc
  ocs2_centroidal_model
  ocs2_pinocchio_interface
  humanoid_common_mpc_ros2
  mujoco_sim_interface
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)


# ROS2 interface library
# add_library(${PROJECT_NAME}
#     src/gains/GainsReceiver.cpp
#     src/gains/GainsUpdaterUtils.cpp
#     )
# ament_target_dependencies(${PROJECT_NAME}
#   ${dependencies}
# )
# target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

################
## Executable ##
################

# Dummy Simulator Node
add_executable(humanoid_centroidal_mpc_dummy_sim_node
  src/CentroidalMpcDummySimNode.cpp
)
ament_target_dependencies(humanoid_centroidal_mpc_dummy_sim_node
  ${dependencies}
)
# target_link_libraries(humanoid_centroidal_mpc_dummy_sim_node
#   ${PROJECT_NAME}
# )
target_compile_options(humanoid_centroidal_mpc_dummy_sim_node PUBLIC ${FLAGS})

# MPC Node
add_executable(humanoid_centroidal_mpc_sqp_node
  src/CentroidalMpcSqpNode.cpp
)
ament_target_dependencies(humanoid_centroidal_mpc_sqp_node
  ${dependencies}
)
# target_link_libraries(humanoid_centroidal_mpc_sqp_node
#   ${PROJECT_NAME}
# )
target_compile_options(humanoid_centroidal_mpc_sqp_node PUBLIC ${FLAGS})

# MPC Sim Executable
add_executable(humanoid_centroidal_mpc_sim
  src/CentroidalMpcRobotSim.cpp
)
ament_target_dependencies(humanoid_centroidal_mpc_sim
  ${dependencies}
)
target_link_libraries(humanoid_centroidal_mpc_sim
        absl::flat_hash_map
        glfw
        GLEW
)
target_compile_options(humanoid_centroidal_mpc_sim PUBLIC ${FLAGS})

# Pose Command Node
add_executable(humanoid_centroidal_mpc_pose_command_node
  src/CentroidalMpcKeyboardPoseCommandNode.cpp
)
ament_target_dependencies(humanoid_centroidal_mpc_pose_command_node
  ${dependencies}
)
# target_link_libraries(humanoid_centroidal_mpc_pose_command_node
#   ${PROJECT_NAME}
# )
target_compile_options(humanoid_centroidal_mpc_pose_command_node PUBLIC ${FLAGS})

#############
## Testing ##
#############

add_executable(test_visualizer
  test/testHumanoidVisualizer.cpp
)
ament_target_dependencies(test_visualizer
  ${dependencies}
)
# target_link_libraries(test_visualizer
#   ${PROJECT_NAME}
# )
target_compile_options(test_visualizer PUBLIC ${FLAGS})




if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

endif()


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

install(TARGETS
  test_visualizer
  humanoid_centroidal_mpc_sim
  humanoid_centroidal_mpc_sqp_node
  humanoid_centroidal_mpc_dummy_sim_node
  humanoid_centroidal_mpc_pose_command_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

