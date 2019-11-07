cmake_minimum_required(VERSION 2.8.3)
project(hps_camera)

add_compile_options(-std=c++11)

link_directories(${CMAKE_SOURCE_DIR}/common/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_hps_camera
  ros/src/ros_hps_camera.cpp
  ros/src/depth_image_converter.cpp
  ros/src/point_cloud_converter.cpp
)

target_link_libraries(
  ros_hps_camera
  ${catkin_LIBRARIES}
  hps3d64179
)
