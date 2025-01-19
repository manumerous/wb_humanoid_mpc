cmake_minimum_required(VERSION 3.16)
project(mujoco_vendor)

if(BUILD_HOST_YOCTO)
    message("Building for ONEX OS, skipping mujoco_vendor")
    find_package(ament_cmake REQUIRED)
    ament_package()
    return()
endif()


find_package(ament_cmake REQUIRED)

include(ExternalProject)

# Define the path to the Mujoco source
set(MUJOCO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/mujoco)

# Add Mujoco as an external project
ExternalProject_Add(mujoco_external
    SOURCE_DIR ${MUJOCO_SOURCE_DIR}
    BINARY_DIR ${CMAKE_BINARY_DIR}/mujoco_build
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    BUILD_ALWAYS 1
)

# Create an interface library for linking with Mujoco
add_library(mujoco INTERFACE)
add_dependencies(mujoco mujoco_external)

# Specify include directories and link libraries for mujoco
target_include_directories(mujoco INTERFACE
    ${CMAKE_INSTALL_PREFIX}/include
)

# Install the mujoco header files and libraries
# install(DIRECTORY ${CMAKE_INSTALL_PREFIX}/include/mujoco
#         DESTINATION include)

install(DIRECTORY ${MUJOCO_SOURCE_DIR}/include/mujoco
        DESTINATION include)

install(FILES ${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so
        DESTINATION lib)

# Export mujoco as a dependency
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include)
ament_export_libraries(mujoco)
ament_package()