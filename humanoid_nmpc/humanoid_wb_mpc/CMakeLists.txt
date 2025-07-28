cmake_minimum_required(VERSION 3.5)
project(humanoid_wb_mpc)

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
  src/dynamics/DynamicsHelperFunctions.cpp
  src/dynamics/WBAccelDynamicsAD.cpp
  src/end_effector/PinocchioEndEffectorDynamicsCppAd.cpp
  src/constraint/EndEffectorDynamicsLinearAccConstraint.cpp
  src/constraint/EndEffectorDynamicsAccelerationsConstraint.cpp
  src/constraint/ZeroAccelerationConstraintCppAd.cpp
  src/constraint/SwingLegVerticalConstraintCppAd.cpp
  src/constraint/JointMimicDynamicsConstraint.cpp
  src/WBMpcInterface.cpp
  src/WBMpcPreComputation.cpp
  src/cost/EndEffectorDynamicsCostHelpers.cpp
  src/cost/EndEffectorDynamicsQuadraticCost.cpp
  src/cost/EndEffectorDynamicsFootCost.cpp
  src/cost/JointTorqueCostCppAd.cpp
  src/command/WBMpcTargetTrajectoriesCalculator.cpp
  src/mrt/WBMpcMrtJointController.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
) 
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})




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

# install(TARGETS 
#   test_pinocchio_model 
#   test_humanoid_interface
#   test_contact_moment_xy_constraint
#   test_end_effector_kinematic_cost_element
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
