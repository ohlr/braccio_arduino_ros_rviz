cmake_minimum_required(VERSION 2.8.3)
project(braccio_arduino_ros_rviz)

 ## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)


## Declare a catkin package
catkin_package()

install(DIRECTORY launch urdf stl rviz parse_and_publish
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(parse_and_publish parse_and_publish/parse_and_publish.cpp)
target_link_libraries(parse_and_publish ${catkin_LIBRARIES})
add_dependencies(parse_and_publish std_msgs)