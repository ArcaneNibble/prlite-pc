cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(prlite_odom)
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roslib roscpp actionlib 
)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

include_directories(include
  include/pr2lite_actuators
  ../pr2lite_moveit_config/msg_gen/cpp/include/
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
cmake_minimum_required(VERSION 2.8.3)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )

add_executable(odom src/odom.cpp)
target_link_libraries(odom 
  ${catkin_LIBRARIES}
)

