cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(pr2lite_actuators)

#start paste
find_package(catkin REQUIRED COMPONENTS roscpp actionlib std_msgs actionlib_msgs genmsg message_generation)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roslib roscpp actionlib message_generation
)
# end paste

# Release, but still debuggable
# set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

include_directories(include
  include/pr2lite_actuators
  ../pr2lite_moveit_config/msg_gen/cpp/include/
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Build the linear actuator shared library
add_library(${PROJECT_NAME} ../pr2lite_actuators/src/actuatorlib/LinearActuator.cpp)

# Test Executables
add_executable(linact_raw_test src/test/linear_actuators.cpp)
target_link_libraries(linact_raw_test ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Linear Actuators server node
add_executable(linact_server src/linact_server.cpp)
target_link_libraries(linact_server ${PROJECT_NAME}
  dl
  ${catkin_LIBRARIES}
)

# Compile plugins
add_library(linactrawplugin src/linact_type_raw.cpp)
target_link_libraries(linactrawplugin ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_library(linactdynamixelplugin src/linact_type_dynamixel.cpp)
target_link_libraries(linactdynamixelplugin ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
