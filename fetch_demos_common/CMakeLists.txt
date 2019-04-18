cmake_minimum_required(VERSION 3.7.2)
project(fetch_demos_common)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grasping_msgs
  shape_msgs
  control_msgs
  rail_manipulation_msgs
  message_generation
  genmsg
  actionlib_msgs
  actionlib
  pcl_conversions
  pcl_ros
  nodelet
  asr_approx_mvbb
  visualization_msgs
)
catkin_python_setup()

add_action_files(
  FILES
  GetObjects.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  grasping_msgs
  shape_msgs
  control_msgs
  actionlib_msgs
  rail_manipulation_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fetch_demos_common
  CATKIN_DEPENDS roscpp rospy std_msgs grasping_msgs rail_manipulation_msgs visualization_msgs control_msgs shape_msgs message_runtime actionlib_msgs pcl_conversions pcl_ros nodelet
)
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${asr_approx_mvbb_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/perception_clustering.cpp
  src/clustering_node.cpp
  src/grasp_suggestion_interface.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${asr_approx_mvbb_TARGETS})
add_executable(grasp_suggestion_node src/grasp_suggestion_node.cpp)
add_dependencies(grasp_suggestion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(fetch_demos_common ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${asr_approx_mvbb_LIBRARIES} yaml-cpp)
target_link_libraries(grasp_suggestion_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${PCL_LIBRARIES} yaml-cpp)

catkin_install_python(PROGRAMS bin/point_head.py bin/open_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
