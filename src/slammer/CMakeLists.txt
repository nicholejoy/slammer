cmake_minimum_required(VERSION 3.0.2)
project(slammer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  nav_msgs
  openslam_gmapping
  robot_localization
  roscpp
  rospy
  sensor_msgs
  slam_gmapping
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slammer
#  CATKIN_DEPENDS geometry_msgs move_base nav_msgs openslam_gmapping robot_localization roscpp rospy sensor_msgs slam_gmapping std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


