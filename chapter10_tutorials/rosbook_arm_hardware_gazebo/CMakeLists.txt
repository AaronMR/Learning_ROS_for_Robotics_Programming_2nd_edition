cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_hardware_gazebo)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/rosbook_arm_hardware_gazebo.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
install (FILES rosbook_arm_hardware_gazebo_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
