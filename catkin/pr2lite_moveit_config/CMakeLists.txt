cmake_minimum_required(VERSION 2.8.3)
project(pr2lite_moveit_config)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(FILES
                  packet_485net_bootloader.msg
                  packet_485net_dgram.msg
                  packet_485net_raw.msg
                  packet_485net_stream.msg
                 )

add_service_files(FILES
                  ReloadID.srv
                  SearchID.srv
                  ReturnJointStates.srv
                 )

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
