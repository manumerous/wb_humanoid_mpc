cmake_minimum_required(VERSION 3.16)
project(mujoco_sim_interface)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(robot_model REQUIRED)
find_package(robot_core REQUIRED)


find_package(GTest REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(absl REQUIRED)

find_package(urdf REQUIRED)

find_package(glfw3 REQUIRED)

find_package(GLEW REQUIRED)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 20)

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

set(dependencies
  ament_cmake
  ament_index_cpp
  mujoco_vendor
  robot_core
  robot_model
  absl
)

#############
## Library ##
#############

add_library(${PROJECT_NAME}
  src/MujocoSimInterface.cpp
  src/MujocoRenderer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
) 
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${urdf_LIBRARIES}
  absl::flat_hash_map
  glfw
  GLEW
)


add_executable(mujoco_sim_no_torques
  exe/mujocoSimNoTorques.cpp
)
ament_target_dependencies(mujoco_sim_no_torques
  ${dependencies}
)
target_link_libraries(mujoco_sim_no_torques
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  glfw
)


# ############
# # Testing ##
# ############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

endif()



#############
## Install ##
#############

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  mujoco_sim_no_torques 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
