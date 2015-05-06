cmake_minimum_required(VERSION 2.8.3)
project(chapter2_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/chapter2.cfg
)




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
	FILES
	chapter2_msg1.msg
)

## Generate services in the 'srv' folder
add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
	FILES
	chapter2_srv1.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

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
#  LIBRARIES chapter2_tutorials
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(chapter2_tutorials
#   src/${PROJECT_NAME}/chapter2_tutorials.cpp
# )

## Declare a cpp executable
# add_executable(chapter2_tutorials_node src/chapter2_tutorials_node.cpp)
add_executable(chap2_example1_a src/example1_a.cpp)
add_executable(chap2_example1_b src/example1_b.cpp)

add_executable(chap2_example2_a src/example2_a.cpp)
add_executable(chap2_example2_b src/example2_b.cpp)

add_executable(chap2_example3_a src/example3_a.cpp)
add_executable(chap2_example3_b src/example3_b.cpp)

add_executable(chap2_example4 src/example4.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(chapter2_tutorials_node chapter2_tutorials_generate_messages_cpp)

add_dependencies(chap2_example1_a chapter2_tutorials_generate_messages_cpp)
add_dependencies(chap2_example1_b chapter2_tutorials_generate_messages_cpp)

add_dependencies(chap2_example2_a chapter2_tutorials_generate_messages_cpp)
add_dependencies(chap2_example2_b chapter2_tutorials_generate_messages_cpp)

add_dependencies(chap2_example3_a chapter2_tutorials_generate_messages_cpp)
add_dependencies(chap2_example3_b chapter2_tutorials_generate_messages_cpp)

add_dependencies(chap2_example4 chapter2_tutorials_gencfg)
## Specify libraries to link a library or executable target against
# target_link_libraries(chapter2_tutorials_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(chap2_example1_a ${catkin_LIBRARIES})
target_link_libraries(chap2_example1_b ${catkin_LIBRARIES})

target_link_libraries(chap2_example2_a ${catkin_LIBRARIES})
target_link_libraries(chap2_example2_b ${catkin_LIBRARIES})

target_link_libraries(chap2_example3_a ${catkin_LIBRARIES})
target_link_libraries(chap2_example3_b ${catkin_LIBRARIES})

target_link_libraries(chap2_example4 ${catkin_LIBRARIES})

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
# install(TARGETS chapter2_tutorials chapter2_tutorials_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chapter2_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
