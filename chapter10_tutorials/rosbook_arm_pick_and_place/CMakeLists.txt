cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_pick_and_place)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    geometry_msgs
    moveit_msgs
    moveit_simple_grasps
    rospy
    std_msgs
    tf
    trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    moveit_msgs
    std_msgs
    trajectory_msgs
)

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
