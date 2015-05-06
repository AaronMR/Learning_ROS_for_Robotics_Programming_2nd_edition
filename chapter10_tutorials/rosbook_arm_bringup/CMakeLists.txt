cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_bringup)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
