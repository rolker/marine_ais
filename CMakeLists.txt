cmake_minimum_required(VERSION 2.8.3)
project(ais)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  marine_msgs
  sensor_msgs
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES posmv
#  CATKIN_DEPENDS geometry_msgs marine_msgs rospy
#  DEPENDS system_lib
)

