cmake_minimum_required(VERSION 3.0.2)
project(assistant_tcp_interface)

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
  LIBRARIES 
    assistant_tcp_interface_plugins
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

add_library(assistant_tcp_interface_plugins
  src/${PROJECT_NAME}/utils/message_package.cpp
  src/${PROJECT_NAME}/client/assistant_tcp_client_interface.cpp
  src/${PROJECT_NAME}/client/assistant_client.cpp
  src/${PROJECT_NAME}/server/assistant_tcp_server_interface.cpp
  src/${PROJECT_NAME}/server/assistant_server.cpp
)
add_dependencies(assistant_tcp_interface_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(assistant_tcp_interface_plugins ${catkin_LIBRARIES})
target_compile_features(assistant_tcp_interface_plugins PRIVATE cxx_std_17)


add_executable(assistant_client_node src/assistant_client_node.cpp)
add_dependencies(assistant_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(assistant_client_node
  ${catkin_LIBRARIES}
  assistant_tcp_interface_plugins
)

add_executable(assistant_server_node src/assistant_server_node.cpp)
add_dependencies(assistant_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(assistant_server_node
 ${catkin_LIBRARIES}
 assistant_tcp_interface_plugins
)

# Test executables
add_executable(test_server src/test_server.cpp)
add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_server
 ${catkin_LIBRARIES}
)
add_executable(test_client src/test_client.cpp)
add_dependencies(test_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_client
 ${catkin_LIBRARIES}
)
add_executable(test_async_server src/test_async_server.cpp)
add_dependencies(test_async_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_async_server
 ${catkin_LIBRARIES}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES assistant_tcp_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)