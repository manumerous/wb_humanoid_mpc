cmake_minimum_required(VERSION 3.5)
project(robot_core)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(GTest REQUIRED)

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
        ${GTEST_INCLUDE_DIRS}
        ${urdf_INCLUDE_DIRS}
)

set(dependencies
        ament_cmake
        ament_index_cpp
)

#############
## Library ##
#############


# ############
# # Testing ##
# ############

if (BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

endif ()

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/
)

# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
