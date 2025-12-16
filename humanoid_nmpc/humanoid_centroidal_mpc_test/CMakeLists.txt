cmake_minimum_required(VERSION 3.5)
project(humanoid_centroidal_mpc_test)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(humanoid_centroidal_mpc REQUIRED)
find_package(humanoid_centroidal_mpc_ros2 REQUIRED)
find_package(g1_description REQUIRED)
find_package(g1_centroidal_mpc REQUIRED)

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

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

set(dependencies
  ament_cmake
  ament_index_cpp
  rclcpp
  humanoid_centroidal_mpc
  humanoid_centroidal_mpc_ros2
  g1_description
  g1_centroidal_mpc
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")


# ############
# # Testing ##
# ############

## Info ==============================
## to run tests, cd package folder and run
## $ colcon build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ colcon run_tests --no-deps --this
## to see the summary of unit test results run
## $ colcon_test_results ../../../build/ocs2_core
## ====================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_centroidal_mpc_robot_model src/testCentroidalMpcRobotModel.cpp)
  ament_target_dependencies(test_centroidal_mpc_robot_model
    ${dependencies}
  )
  target_link_libraries(test_centroidal_mpc_robot_model 
    ${rclcpp_LIBRARIES} 
    ${rmw_connext_cpp_LIBRARIES} 
    ${std_interfaces}
  )

  ament_add_gtest(test_pinocchio_frame_conversions src/testPinocchioFrameConversions.cpp)
  ament_target_dependencies(test_pinocchio_frame_conversions
    ${dependencies}
  )
  target_link_libraries(test_pinocchio_frame_conversions 
    ${rclcpp_LIBRARIES} 
    ${rmw_connext_cpp_LIBRARIES} 
    ${std_interfaces}
  )

ament_add_gtest(test_dynamics_helper_functions src/testDynamicsHelperFunctions.cpp)
  ament_target_dependencies(test_dynamics_helper_functions
    ${dependencies}
  )
  target_link_libraries(test_dynamics_helper_functions
    ${rclcpp_LIBRARIES}
    ${rmw_connext_cpp_LIBRARIES}
    ${std_interfaces}
)

endif()

#############
## Install ##
#############


install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
