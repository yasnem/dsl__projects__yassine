cmake_minimum_required(VERSION 2.8)

# Project name
project(asrl__robots__gamepad_control)

# Find other ROS packages using CMake (this defines macros)
find_package(
    catkin REQUIRED 
    COMPONENTS roscpp joy geometry_msgs std_msgs asrl__common__library
    )

# Export build information to the catkin system (install package.xml, setup find_package() for this package, etc.)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp joy geometry_msgs std_msgs asrl__common__library
)


## Build Targets

# Includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Find lib source
file(GLOB LIB_SRC
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(gamepad_control_node src/gamepad_control_node.cpp)
target_link_libraries(gamepad_control_node ${catkin_LIBRARIES})

add_executable(gamepad_trigger_node src/gamepad_trigger_node.cpp)
target_link_libraries(gamepad_trigger_node ${catkin_LIBRARIES})

#add_executable(gamepad_twist_node src/gamepad_twist_node.cpp)
#target_link_libraries(gamepad_twist_node ${catkin_LIBRARIES})

#add_executable(gamepad_scalar_node src/gamepad_scalar_node.cpp)
#target_link_libraries(gamepad_scalar_node ${catkin_LIBRARIES})

#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Set the build type.  Options are:
##  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
##  Debug          : w/ debug symbols, w/o optimization
##  Release        : w/o debug symbols, w/ optimization
##  RelWithDebInfo : w/ debug symbols, w/ optimization
##  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

#rosbuild_init()

##set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
##set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##uncomment if you have defined messages
##rosbuild_genmsg()
##uncomment if you have defined services
##rosbuild_gensrv()

##common commands for building c++ executables and libraries
##rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
##target_link_libraries(${PROJECT_NAME} another_library)
##rosbuild_add_boost_directories()
##rosbuild_link_boost(${PROJECT_NAME} thread)
##rosbuild_add_executable(example examples/example.cpp)
##target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(gamepad_control_node src/gamepad_control_node.cpp)
#rosbuild_add_executable(gamepad_twist_node src/gamepad_twist_node.cpp)
#rosbuild_add_executable(gamepad_scalar_node src/gamepad_scalar_node.cpp)










