cmake_minimum_required(VERSION 3.5)
project(humanoid_common_mpc_ros2)

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
find_package(ocs2_sqp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(humanoid_common_mpc REQUIRED)
find_package(humanoid_mpc_msgs REQUIRED)



find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
    log
    log_setup
)

# find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


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
    humanoid_common_mpc
    ocs2_sqp
    humanoid_mpc_msgs
)

link_directories(
    ${pinocchio_LIBRARY_DIRS}
)


# ROS2 interface library
add_library(${PROJECT_NAME}
    src/gait/GaitKeyboardPublisher.cpp
    src/gait/GaitScheduleUpdaterRos2.cpp
    src/visualization/HumanoidVisualizer.cpp
    src/visualization/HumanoidVisualizerRos2Interface.cpp
    src/visualization/EquivalentContactCornerForcesVisualizer.cpp
    src/ros_comm/MRTPolicySubscriber.cpp
    src/ros_comm/Ros2ProceduralMpcMotionManager.cpp
    src/ros_comm/VelocityCommandKeyboardPublisher.cpp
    src/benchmarks/SqpBenchmarksPublisher.cpp
    src/benchmarks/DdpBenchmarksPublisher.cpp
)
ament_target_dependencies(${PROJECT_NAME}
    ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

################
## Executable ##
################

# Gait Command Node
add_executable(gait_keyboard_command_node
    src/HumanoidGaitCommandNode.cpp
)
ament_target_dependencies(gait_keyboard_command_node
    ${dependencies}
)
target_link_libraries(gait_keyboard_command_node
    ${PROJECT_NAME}
)
target_compile_options(gait_keyboard_command_node PUBLIC ${FLAGS})

# Gait Command Node
add_executable(velocity_keyboard_command_node
    src/MpcKeyboardVelocityCommandNode.cpp
)
ament_target_dependencies(velocity_keyboard_command_node
    ${dependencies}
)
target_link_libraries(velocity_keyboard_command_node
    ${PROJECT_NAME}
)
target_compile_options(velocity_keyboard_command_node PUBLIC ${FLAGS})

#############
## Testing ##
#############


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(TARGETS
    gait_keyboard_command_node
    velocity_keyboard_command_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

