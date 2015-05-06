cmake_minimum_required(VERSION 2.8.3)
project(chapter6_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

## Add include directories
include_directories(include
	  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES retrorov_model_bathymetric
#  CATKIN_DEPENDS pcl pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(retrorov_model_bathymetric
#   src/${PROJECT_NAME}/retrorov_model_bathymetric.cpp
# )

## Declare a cpp executable

add_executable(pcl_create src/pcl_create.cpp)
add_executable(pcl_downsampling src/pcl_downsampling.cpp)
add_executable(pcl_filter src/pcl_filter.cpp)
add_executable(pcl_matching src/pcl_matching.cpp)
add_executable(pcl_model_estimation src/pcl_model_estimation.cpp)
add_executable(pcl_partitioning src/pcl_partitioning.cpp)
add_executable(pcl_planar_segmentation src/pcl_planar_segmentation.cpp)
add_executable(pcl_read src/pcl_read.cpp)
add_executable(pcl_visualize src/pcl_visualize.cpp)
add_executable(pcl_write src/pcl_write.cpp)
 
 ## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(retrorov_model_bathymetric_node retrorov_model_bathymetric_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pcl_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_downsampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_matching ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_model_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_partitioning ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_planar_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_visualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_write ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS retrorov_model_bathymetric retrorov_model_bathymetric_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_retrorov_model_bathymetric.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
