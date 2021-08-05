cmake_minimum_required(VERSION 3.0.2)
project(comau_tcp_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib 
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES comau_tcp_interface_plugins
  CATKIN_DEPENDS 
    roscpp 
  DEPENDS 
    Boost 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(comau_tcp_interface_plugins
  src/${PROJECT_NAME}/utils/message_package.cpp
  src/${PROJECT_NAME}/comau_tcp_interface.cpp
  src/${PROJECT_NAME}/comau_state_client.cpp
  src/${PROJECT_NAME}/comau_robot_client.cpp
)
add_dependencies(comau_tcp_interface_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_tcp_interface_plugins ${catkin_LIBRARIES})
target_compile_features(comau_tcp_interface_plugins PRIVATE cxx_std_17)

add_executable(comau_state_client_node src/comau_state_client_node.cpp)
add_dependencies(comau_state_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_state_client_node
  comau_tcp_interface_plugins
  ${catkin_LIBRARIES}
)