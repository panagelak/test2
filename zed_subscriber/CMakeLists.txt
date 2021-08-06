cmake_minimum_required(VERSION 2.8.3)
project(zed_subscriber)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  geometry_msgs
  pcl_ros
  pcl_conversions
  message_filters  
  tf2
  tf2_ros
  std_msgs
  zed_msgs
  compress_depth_image
  compress_image
)

#find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  CATKIN_DEPENDS other_catkin_pkg
)

include_directories(
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(zed_subscriber nodes/zed_subscriber.cpp)
target_link_libraries(zed_subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(zed_subscriber ${catkin_EXPORTED_TARGETS})
