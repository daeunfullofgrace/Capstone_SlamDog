cmake_minimum_required(VERSION 2.8.3)
project(slamdog_navi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  message_generation
  message_runtime
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)

add_service_files(FILES arrivalType.srv)
add_service_files(FILES dataLoader.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES simple_navigation_goals
 CATKIN_DEPENDS actionlib move_base_msgs roscpp cv_bridge image_transport sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

add_executable(pose_initialization src/pose_initialization.cpp)
target_link_libraries(pose_initialization ${catkin_LIBRARIES})

add_executable(slam_dog_navi src/slam_dog_navi.cpp)
target_link_libraries(slam_dog_navi ${catkin_LIBRARIES})
add_dependencies(slam_dog_navi slamdog_navi_generate_messages_cpp)

add_executable(slamdog_navi src/slamdog_navi.cpp)
target_link_libraries(slamdog_navi ${catkin_LIBRARIES})
add_dependencies(slamdog_navi slamdog_navi_generate_messages_cpp)

add_executable(navi_result_sender_server src/navi_result_sender_server.cpp)
target_link_libraries(navi_result_sender_server ${catkin_LIBRARIES})
add_dependencies(navi_result_sender_server slamdog_navi_generate_messages_cpp)

add_executable(navi_result_sender_client src/navi_result_sender_client.cpp)
target_link_libraries(navi_result_sender_client ${catkin_LIBRARIES})
add_dependencies(navi_result_sender_client slamdog_navi_generate_messages_cpp)

add_executable(get_depth_data src/get_depth_data.cpp)
target_link_libraries(get_depth_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(get_depth_data slamdog_navi_generate_messages_cpp)

