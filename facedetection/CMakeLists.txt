cmake_minimum_required(VERSION 2.8.3)
project(facedetection)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zzz
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(detectFace src/camera_subscribe.cpp)
target_link_libraries(detectFace ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
