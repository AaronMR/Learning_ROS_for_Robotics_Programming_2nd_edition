cmake_minimum_required(VERSION 2.8.3)
project(chapter3_tutorials)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      dynamic_reconfigure
      driver_base
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

find_package(OpenCV)
find_package(PCL REQUIRED)

add_service_files(FILES SetSpeed.srv)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(cfg/DynamicParam.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      dynamic_reconfigure
      driver_base
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(example1 src/example1.cpp)
target_link_libraries(example1 ${catkin_LIBRARIES})

add_executable(example2 src/example2.cpp)
target_link_libraries(example2 ${catkin_LIBRARIES})

add_executable(example3 src/example3.cpp)
target_link_libraries(example3 ${catkin_LIBRARIES})

add_executable(example4 src/example4.cpp)
add_dependencies(example4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(example4 ${catkin_LIBRARIES})

add_executable(example5 src/example5.cpp)
add_dependencies(example5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(example5 ${catkin_LIBRARIES})

add_executable(example6 src/example6.cpp)
add_dependencies(example6 ${PROJECT_NAME}_gencfg)
target_link_libraries(example6 ${catkin_LIBRARIES})

add_executable(example7 src/example7.cpp)
target_link_libraries(example7 ${catkin_LIBRARIES})

add_executable(example8 src/example8.cpp)
target_link_libraries(example8 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(example9 src/example9.cpp)
target_link_libraries(example9 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(example10 src/example10.cpp)
target_link_libraries(example10 ${catkin_LIBRARIES})

install(TARGETS
    example1 example2 example3 example4 example5 example6 example7 example8
    example9 example10
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir bag config launch output)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
