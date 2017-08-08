##### HEADER #####
cmake_minimum_required(VERSION 2.8)
project(asrl__robots__pioneer)


###### IMPORT DEPENDENCIES ######
## System ##
find_package(Boost REQUIRED COMPONENTS thread)

## ROS ##
find_package(catkin REQUIRED
             COMPONENTS 
             	std_msgs
             	rospy
             	roscpp
             	asrl__robots__vehicle
             	asrl__common__library
             	aria
)
 
##### CONFIGURE INCLUDE DIRECTORIES #####
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    $ENV{UTIAS_ROS_DIR}/extras/install_isolated/share/aria/Aria/include
                    #${[system_dependency_1]_INCLUDE_DIRS}
)

link_directories($ENV{UTIAS_ROS_DIR}/extras/install_isolated/share/aria/Aria/lib)


##### EXPORT MYSELF TO CATKIN #####
#   "INCLUDE_DIRS" specifies to location of your package's headers for inclusion in inheriting ${catkin_INCLUDE_DIRS}.
#   "LIBRARIES" specifies the libraries your package provides for inclusion in inheriting ${catkin_LIBRARIES}. This should match all add_library directives.
#   "CATKIN_DEPENDS" specifies the ros runtime dependencies
#   "DEPENDS" specifies the system runtime dependencies 
catkin_package(
               #INCLUDE_DIRS include
               #LIBRARIES ${PROJECT_NAME}
               #DEPENDS [system_dependency_1]
               CATKIN_DEPENDS std_msgs rospy roscpp asrl__robots__vehicle asrl__common__library aria)


##### SPECIFY BUILD TARGETS #####
##Executables:
add_executable(asrl_pioneer_node src/asrl_pioneer_node.cpp)
target_link_libraries(asrl_pioneer_node ${catkin_LIBRARIES} Aria)
add_dependencies(asrl_pioneer_node ${catkin_EXPORTED_TARGETS})

#A deprecated TF broadcaster between the Vicon frame on the rover and the rover frame (I think)
#add_executable(pioneer_vicon_tf_broadcaster src/pioneer_vicon_broadcaster.cpp)
#target_link_libraries(pioneer_vicon_tf_broadcaster ${catkin_LIBRARIES})
#add_dependencies(pioneer_vicon_tf_broadcaster ${catkin_EXPORTED_TARGETS})
