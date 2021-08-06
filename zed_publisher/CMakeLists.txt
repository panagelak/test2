cmake_minimum_required(VERSION 3.1.0)
project(zed_publisher)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -Wall -g3 -O0 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -Wextra -g0 -Ofast")
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(Boost REQUIRED COMPONENTS system program_options thread)
find_package(ZED 3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	zed_msgs
	compress_depth_image
	compress_image
	rosparam_shortcuts
)

catkin_package(CATKIN_DEPENDS
	roscpp
	sensor_msgs
	zed_msgs
	compress_depth_image
	compress_image
	rosparam_shortcuts
)

include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} include)
set(SOURCES src/${PROJECT_NAME}.cpp src/zed_publisher_node.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME} PRIVATE)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY} ${ZED_LIBRARIES} ${CUDA_CUDART_LIBRARY} ${CUDA_DEP_LIBRARIES_ZED})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})