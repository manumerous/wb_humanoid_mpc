cmake_minimum_required(VERSION 3.5)
project(humanoid_centroidal_mpc)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_ros2_interfaces REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(humanoid_common_mpc REQUIRED)
find_package(robot_model REQUIRED)

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
  ocs2_core
  ocs2_centroidal_model
  ocs2_ros2_interfaces
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_pinocchio_interface
  humanoid_common_mpc
  robot_model
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

#############
## Library ##
#############

add_library(${PROJECT_NAME}
  src/dynamics/CentroidalDynamicsAD.cpp
  src/cost/CentroidalMpcEndEffectorFootCost.cpp
  src/cost/ICPCost.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/JointMimicKinematicConstraint.cpp
  src/initialization/CentroidalWeightCompInitializer.cpp
  src/CentroidalMpcInterface.cpp
  src/command/CentroidalMpcTargetTrajectoriesCalculator.cpp
  src/mrt/CentroidalMpcMrtJointController.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
) 
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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

endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

install(TARGETS  
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
