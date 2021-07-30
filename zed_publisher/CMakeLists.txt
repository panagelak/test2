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

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	std_msgs
	sensor_msgs
	geometry_msgs
	message_filters
)

find_package(Boost REQUIRED COMPONENTS system program_options thread)
find_package(ZED 3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA REQUIRED)

# find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

add_message_files(FILES Skeleton.msg SkeletonArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS
	roscpp
	message_generation
	message_runtime
	std_msgs
	sensor_msgs
	geometry_msgs
	message_filters
)

include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} include)
set(SOURCES src/${PROJECT_NAME}.cpp src/Publisher.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME} PRIVATE)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY} ${ZED_LIBRARIES} ${CUDA_CUDART_LIBRARY} ${CUDA_DEP_LIBRARIES_ZED})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Find OGRE for RViz
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)
#if(NOT OGRE_OV_FOUND)
#	pkg_check_modules(OGRE_OV REQUIRED OGRE)
#endif(NOT OGRE_OV_FOUND)
#if(NOT DEFINED OGRE_PLUGIN_PATH)
#	execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE OUTPUT_VARIABLE OGRE_PLUGIN_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
#endif(NOT DEFINED OGRE_PLUGIN_PATH)
# message("STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH}")
#if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
#	unset(OGRE_OV_LIBRARIES_ABS_TMP)
#	foreach(_lib ${OGRE_OV_LIBRARIES})
#		set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")
#		find_library(${OGRE_OV_LIB_TAG} NAMES ${_lib} HINTS ${OGRE_OV_LIBRARY_DIRS} PATHS ${OGRE_OV_LIBRARY_DIRS})
#		set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
#	endforeach(_lib)
#	set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
#	CACHE FILEPATH "Pathname of library ${_lib}")
#endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
#if(NOT OGRE_OV_LIBRARIES_ABS)
#	set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES})
#endif()

#catkin_package(INCLUDE_DIRS ${OGRE_OV_INCLUDE_DIRS} LIBRARIES ${OGRE_OV_LIBRARIES_ABS})
#include_directories(${OGRE_OV_INCLUDE_DIRS})
#add_definitions(-DQT_NO_KEYWORDS)
#qt5_wrap_cpp(MOC_FILES rviz/SkeletonDisplay.h)
#set(RVIZ_SOURCE
#	rviz/SkeletonDisplay.cpp
#	rviz/SkeletonVisual.cpp
#	${MOC_FILES}
#)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#add_library(${PROJECT_NAME}_rviz ${RVIZ_SOURCE})
#add_dependencies(${PROJECT_NAME}_rviz ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_rviz ${OGRE_OV_LIBRARIES_ABS})
#qt5_use_modules(${PROJECT_NAME}_rviz Widgets)
#install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})