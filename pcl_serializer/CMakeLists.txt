cmake_minimum_required(VERSION 2.8.3)
project(pcl_serializer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
find_package(numpy)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  pcl_ros
  pcl_conversions
  message_filters  
  tf2
  tf2_ros
  std_msgs
  integration
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

#add_executable(detectron2_pcl_sync_node nodes/detectron2_pcl_sync_node.cpp)
#target_link_libraries(detectron2_pcl_sync_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(detectron2_pcl_sync_node ${catkin_EXPORTED_TARGETS})

add_executable(pcl_serializer nodes/pcl_serializer.cpp)
target_link_libraries(pcl_serializer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_serializer ${catkin_EXPORTED_TARGETS})

add_executable(pcl_deserializer nodes/pcl_deserializer.cpp)
target_link_libraries(pcl_deserializer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_deserializer ${catkin_EXPORTED_TARGETS})
