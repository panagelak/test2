cmake_minimum_required(VERSION 3.0.2)
project(digitaltwin_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  integration
  rosbridge_server
  rospy_message_converter
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES digitaltwin_bridge
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  nodes/digitaltwin_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# catkin_install_python(PROGRAMS
#   nodes/digitaltwin_bridge_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

add_custom_target(install_depends ALL COMMAND "pip" "install" "--user" "-r" "${PROJECT_SOURCE_DIR}/requirements.txt")