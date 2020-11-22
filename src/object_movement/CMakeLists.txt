cmake_minimum_required(VERSION 3.0.2)
project(object_movement)

# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  intera_interface
  moveit_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
