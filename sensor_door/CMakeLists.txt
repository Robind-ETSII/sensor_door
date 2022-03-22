cmake_minimum_required(VERSION 3.0.2)
project(sensor_door)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_door_msgs)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  sensor_door_server 
  src/main.cpp 
  src/server.cpp
  src/client.cpp
  src/cluster.cpp
)
target_link_libraries(sensor_door_server ${catkin_LIBRARIES})
add_dependencies(sensor_door_server ${catkin_EXPORTED_TARGETS})

add_executable(
  security_node 
  src/security_node.cpp 
)
target_link_libraries(security_node ${catkin_LIBRARIES})
add_dependencies(security_node ${catkin_EXPORTED_TARGETS})