cmake_minimum_required(VERSION 2.8)

# Project name
project(xboxdrv_linux)

#The required minimum dependency on catkin
find_package(catkin REQUIRED)
find_package(Boost COMPONENTS thread system REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

#The required basic catkin setup function
catkin_package()

MESSAGE("%%%%%%%%%%%%% BUILDING XBOX DRIVER -- MAY ASK FOR PASSWORD!!! ")
add_custom_target(build_xboxdrv ALL 
                COMMAND ${CMAKE_MAKE_PROGRAM}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                COMMENT "run custom xbox driver makefile target")

install(DIRECTORY
   scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
   USE_SOURCE_PERMISSIONS
   PATTERN ".svn" EXCLUDE
 )











