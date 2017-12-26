# Braccio-Arduino-ROS-Gazebo
Project on the integration of the Arduino robotic arm Braccio into ROS and Gazebo.

roslaunch braccio_arduino_ros_gazebo urdf.launch model:=urdf/braccio_arm.urdf

roslaunch braccio_arduino_ros_gazebo gazeboxacro.launch model:=urdf/braccio_arm.urdf

roslaunch braccio_arduino_ros_gazebo gazebo.launch model:=urdf/braccio_arm.urdf

rostopic echo /joint_states/position

