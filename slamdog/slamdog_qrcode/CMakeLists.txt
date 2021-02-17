cmake_minimum_required(VERSION 2.8.3)
project(slamdog_qrcode)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
   FILES
   CustomQrCodeSrv.srv
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
