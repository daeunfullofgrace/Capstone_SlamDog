cmake_minimum_required(VERSION 2.8.3)
project(data_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
   FILES
   dataLoader.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES qrcode
#  CATKIN_DEPENDS rospy std_msgs
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/data_loader_srv_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/data_loader_srv_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/loc_file_reader_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
