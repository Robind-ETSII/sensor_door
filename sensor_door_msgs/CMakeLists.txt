cmake_minimum_required(VERSION 3.0.2)
project(sensor_door_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
  FILES
  DoorStatus.msg
  MultipleDoorStatus.msg
  DoorSecurity.msg
)

add_service_files(
  FILES
  CleanDisconnectedDevices.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)