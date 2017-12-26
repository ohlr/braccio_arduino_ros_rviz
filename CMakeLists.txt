cmake_minimum_required(VERSION 2.8.3)
project(braccio_arduino_ros_gazebo)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch urdf stl rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})