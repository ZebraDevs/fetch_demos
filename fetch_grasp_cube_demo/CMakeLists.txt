cmake_minimum_required(VERSION 3.7.2)
project(fetch_grasp_cube_demo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grasping_msgs
  shape_msgs
  control_msgs
  message_generation
  actionlib_msgs
  actionlib
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fetch_grasp_cube_demo
  CATKIN_DEPENDS roscpp rospy std_msgs grasping_msgs control_msgs shape_msgs message_runtime actionlib_msgs 
)

include_directories(
#  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

