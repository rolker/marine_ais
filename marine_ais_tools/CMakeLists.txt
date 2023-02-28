cmake_minimum_required(VERSION 2.8.3)
project(marine_ais_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  marine_ais_msgs
  nmea_msgs
  sensor_msgs
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES posmv
#  CATKIN_DEPENDS geometry_msgs project11_msgs rospy
#  DEPENDS system_lib
)

