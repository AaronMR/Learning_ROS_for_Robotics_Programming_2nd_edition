cmake_minimum_required(VERSION 2.8.3)
project(chapter9_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  move_base_msgs
  actionlib
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sendGoals src/sendGoals.cpp)
target_link_libraries(sendGoals ${catkin_LIBRARIES})

install(TARGETS sendGoals
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch maps)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
