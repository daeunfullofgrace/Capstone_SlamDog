cmake_minimum_required(VERSION 2.8.3)
project(data_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(FILES dataLoader.srv)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES data_manager
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS system_lib
)

# generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(data_loader_srv_server src/data_loader_srv_server.cpp)
target_link_libraries(data_loader_srv_server ${catkin_LIBRARIES})
add_dependencies(data_loader_srv_server data_manager_generate_messages_cpp)

add_executable(data_loader_srv_client src/data_loader_srv_client.cpp)
target_link_libraries(data_loader_srv_client ${catkin_LIBRARIES})
add_dependencies(data_loader_srv_client data_manager_generate_messages_cpp)