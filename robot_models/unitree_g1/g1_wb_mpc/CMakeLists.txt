cmake_minimum_required(VERSION 3.5)
project(g1_wb_mpc)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(humanoid_wb_mpc_ros2 REQUIRED)

find_package(GTest REQUIRED)

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

set(dependencies
  ament_cmake
  ament_index_cpp
  rclcpp
  humanoid_wb_mpc_ros2
)

#############
## Install ##
#############

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
