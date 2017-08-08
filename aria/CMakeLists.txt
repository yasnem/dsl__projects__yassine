cmake_minimum_required(VERSION 2.8)

# Project name
project(aria)

#The required minimum dependency on catkin
find_package(catkin REQUIRED)

#The required basic catkin setup function
catkin_package()

add_custom_target(build_aria ALL 
                COMMAND ${CMAKE_MAKE_PROGRAM}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                COMMENT "run custom ARIA makefile target")