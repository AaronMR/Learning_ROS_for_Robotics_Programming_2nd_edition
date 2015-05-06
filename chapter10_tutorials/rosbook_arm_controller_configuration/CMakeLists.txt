cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_controller_configuration)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
