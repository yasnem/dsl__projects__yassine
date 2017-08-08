cmake_minimum_required(VERSION 2.8)

# Project name
project(asrl__robots__vehicle)

# ROS dependencies
find_package(catkin REQUIRED
  COMPONENTS asrl__common__library nav_msgs geometry_msgs tf) # sound_play gamepad_control

## Messages ##

# Find messages
file(GLOB MSG_FILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
  "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg"
  )

# Find services
file(GLOB SRV_FILES
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv"
    "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv"
)

# Add messages
add_message_files(
    DIRECTORY msg
    FILES
    ${MSG_FILES}
)

add_service_files(
    DIRECTORY srv
    FILES
    ${SRV_FILES}
)

# Generate messages
generate_messages(DEPENDENCIES nav_msgs geometry_msgs tf) #sound_play gamepad_control asrl__common__library 

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS asrl__common__library nav_msgs geometry_msgs tf
  DEPENDS Boost Eigen OpenCV
)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#
## Set the build type.  Options are:
##  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
##  Debug          : w/ debug symbols, w/o optimization
##  Release        : w/o debug symbols, w/ optimization
##  RelWithDebInfo : w/ debug symbols, w/ optimization
##  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
##set(ROS_BUILD_TYPE RelWithDebInfo)
#
#rosbuild_init()
#
##set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
##set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#
##uncomment if you have defined messages
#rosbuild_genmsg()
##uncomment if you have defined services
#rosbuild_gensrv()
#
##common commands for building c++ executables and libraries
##rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
##rosbuild_add_boost_directories()
##rosbuild_link_boost(${PROJECT_NAME} thread)
##rosbuild_add_executable(example examples/example.cpp)
##target_link_libraries(example ${PROJECT_NAME})
#
##rosbuild_add_library(${PROJECT_NAME} src/base.cpp)
#
