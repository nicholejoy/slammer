cmake_minimum_required(VERSION 3.0.2)
project(slammer_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros_control
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slammer_sim
#  CATKIN_DEPENDS gazebo_ros_control roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

