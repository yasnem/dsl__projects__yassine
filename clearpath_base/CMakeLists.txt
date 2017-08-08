cmake_minimum_required(VERSION 2.8.3)
project(clearpath_base)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs message_generation)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
file(GLOB msg_files RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/*.msg)
add_message_files(FILES ${msg_files})
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

install(PROGRAMS
  nodes/kinematic_node nodes/raw_node nodes/torque_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

