cmake_minimum_required(VERSION 3.5)

project(humanoid_mpc_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# include_directories(
#     generated/cpp/include
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WalkingVelocityCommand.idl"
    
  DEPENDENCIES rcl_interfaces builtin_interfaces geometry_msgs std_msgs unique_identifier_msgs
  ADD_LINTER_TESTS
)

# install(DIRECTORY generated/cpp/include/${PROJECT_NAME}
#         DESTINATION include/
#         )

ament_package()
