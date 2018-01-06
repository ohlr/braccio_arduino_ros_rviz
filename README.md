# Braccio-Arduino-ROS-Rviz
Project on the integration of the Arduino robotic arm Braccio into ROS and Rviz.

![demo](Demo/demo.gif)

With this Code it is possible to control the joint angles of Braccio from a GUI.


Setup:
Dependencies ROS Kinetic
Arduino Uno
Braccio


Terminal 1:
	roscore

Terminal 2:
	source devel/setup.bash
	cd src/braccio_arduino_ros_gazebo

	//start the gui
	roslaunch braccio_arduino_ros_gazebo urdf.launch model:=urdf/braccio_arm.urdf

Terminal 3:
	source devel/setup.bash
	cd src/braccio_arduino_ros_gazebo
	rosrun braccio_arduino_ros_gazebo parse_and_publish

Terminal 4:
	//rosserial communication between arduino and pc
    rosrun rosserial_python serial_node.py /dev/ttyACM0

Terminal 5 (optional):
	//view what is published
	rostopic echo joint_array


