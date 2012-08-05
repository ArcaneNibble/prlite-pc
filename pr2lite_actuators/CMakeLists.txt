cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Release, but still debuggable
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Build the linear actuator shared library
rosbuild_add_library(${PROJECT_NAME} src/actuatorlib/LinearActuator.cpp)

# Test Executables
rosbuild_add_executable(linact_raw_test src/test/linear_actuators.cpp)
target_link_libraries(linact_raw_test ${PROJECT_NAME})

# Linear Actuators server node
rosbuild_add_executable(linact_server src/linact_server.cpp)
target_link_libraries(linact_server ${PROJECT_NAME})

# Compile plugins
rosbuild_add_library(linactrawplugin src/linact_type_raw.cpp)
target_link_libraries(linactrawplugin ${PROJECT_NAME})
rosbuild_add_library(linactdynamixelplugin src/linact_type_dynamixel.cpp)
target_link_libraries(linactdynamixelplugin ${PROJECT_NAME})
