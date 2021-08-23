cmake_minimum_required(VERSION 2.8.3)
project(zed_msgs)

find_package(catkin
  REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
)

# Generate actions in the 'action' folder
#add_action_files(
#  DIRECTORY action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  
   actionlib_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES comau_msgs
  CATKIN_DEPENDS std_msgs geometry_msgs actionlib_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
