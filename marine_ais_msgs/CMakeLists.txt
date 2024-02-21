cmake_minimum_required(VERSION 3.8)
project(marine_ais_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  msg/Addressed.msg
  msg/AidsToNavigation.msg
  msg/AIS.msg
  msg/AISContact.msg
  msg/Binary.msg
  msg/ChannelManagement.msg
  msg/ClassB.msg
  msg/Communication.msg
  msg/DataLinkReservationBlock.msg
  msg/Destination.msg
  msg/Navigation.msg
  msg/NavigationalStatus.msg
  msg/Static.msg
  msg/Voyage.msg
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
  builtin_interfaces
  geographic_msgs
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
