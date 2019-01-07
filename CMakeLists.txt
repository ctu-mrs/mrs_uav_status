cmake_minimum_required(VERSION 2.8.3)
project(mrs_status)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  mavros_msgs
  message_generation
  mrs_msgs
  mrs_lib
)

catkin_package(
  CATKIN_DEPENDS message_runtime # image_transport roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

