cmake_minimum_required(VERSION 3.0.2)
project(marine_ais_msgs)

find_package(catkin REQUIRED geographic_msgs message_generation std_msgs)

add_message_files(
  FILES
  Addressed.msg
  AidsToNavigation.msg
  AIS.msg
  AISContact.msg
  Binary.msg
  ChannelManagement.msg
  ClassB.msg
  Communication.msg
  DataLinkReservationBlock.msg
  Destination.msg
  Navigation.msg
  NavigationalStatus.msg
  Static.msg
  Voyage.msg
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)
