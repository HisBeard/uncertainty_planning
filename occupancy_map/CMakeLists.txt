cmake_minimum_required(VERSION 3.0.2)
project(occupancy_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}./include)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization)
catkin_package()
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_executable(map_realtime src/map_realtime.cpp)
target_link_libraries(map_realtime ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${GSLLLIB})

