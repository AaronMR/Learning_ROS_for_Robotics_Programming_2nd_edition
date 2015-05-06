cmake_minimum_required(VERSION 2.8.3)
project(rosbook_arm_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config gazebo meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
