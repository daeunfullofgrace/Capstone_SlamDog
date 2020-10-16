cmake_minimum_required(VERSION 2.8.3)
project(slamdog_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ros_msgs_tutorial
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
