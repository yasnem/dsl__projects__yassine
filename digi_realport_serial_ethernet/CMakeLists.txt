cmake_minimum_required(VERSION 2.8)

# Project name
project(digi_realport_serial_ethernet)

#The required minimum dependency on catkin
find_package(catkin REQUIRED)

#The required basic catkin setup function
catkin_package()

#The build target as a makefile command
MESSAGE("BUILDING DGRP SERIAL to ETHERNET DRIVER -- MAY ASK FOR PASSWORD!")
add_custom_target(build_dgrp ALL 
                COMMAND ${CMAKE_MAKE_PROGRAM}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                COMMENT "run dgrp makefile target")



#The install target
install(DIRECTORY
  scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)