cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_snippets)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

add_executable(move_group_plan_single_target src/move_group_plan_single_target.cpp)
target_link_libraries(move_group_plan_single_target ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_group_plan_group_state src/move_group_plan_group_state.cpp)
target_link_libraries(move_group_plan_group_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_group_plan_random_target src/move_group_plan_random_target.cpp)
target_link_libraries(move_group_plan_random_target ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_group_add_object src/move_group_add_object.cpp)
target_link_libraries(move_group_add_object ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_group_remove_object src/move_group_remove_object.cpp)
target_link_libraries(move_group_remove_object ${catkin_LIBRARIES} ${Boost_LIBRARIES})