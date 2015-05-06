cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_gazebo)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
