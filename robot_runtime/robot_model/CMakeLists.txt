cmake_minimum_required(VERSION 3.5)
project(robot_model)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(robot_core REQUIRED)

find_package(GTest REQUIRED)



find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(urdf REQUIRED)

find_package(absl REQUIRED)

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
        robot_core
        absl
)

#############
## Library ##
#############

add_library(${PROJECT_NAME}
        src/RobotDescription.cpp
        src/RobotState.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
        ${urdf_LIBRARIES}
        absl::flat_hash_map
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ############
# # Testing ##
# ############

if (BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

        ament_add_gtest(test_robot_description test/testRobotDescription.cpp)
        target_link_libraries(test_robot_description
        ${urdf_LIBRARIES}
        ${PROJECT_NAME}
        )
        ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)

endif ()

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

# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies} )
ament_package()