cmake_minimum_required(VERSION 2.8.3)
project(pr2lite_nav)

find_package(catkin)
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
