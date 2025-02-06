cmake_minimum_required(VERSION 3.5)
project(drc_atlas_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz materials
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
