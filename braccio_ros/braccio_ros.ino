
#include <ros.h>
#include <Arduino.h>
#include <BraccioLibRos.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

unsigned int _baseAngle = 90;
unsigned int _shoulderAngle = 90;
unsigned int _elbowAngle = 90;
unsigned int _wrist_verAngle = 90;
unsigned int _wrist_rotAngle = 90;
unsigned int _gripperAngle =73; //closed

void BraccioMove( const std_msgs::UInt8MultiArray& angleArray){
  _baseAngle = (unsigned int)angleArray.data[0];
  _shoulderAngle = (unsigned int)angleArray.data[1];
  _elbowAngle = (unsigned int) angleArray.data[2];
  _wrist_verAngle = (unsigned int)angleArray.data[3];
  _wrist_rotAngle = (unsigned int)angleArray.data[4];
  _gripperAngle = (unsigned int)angleArray.data[5];
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("joint_array", &BraccioMove);

void setup()
{ 
  Braccio.begin();
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
 
 Braccio.ServoMovement(20,_baseAngle,_shoulderAngle,_elbowAngle,_wrist_verAngle,_wrist_rotAngle,_gripperAngle);
  nh.spinOnce();
  delay(1);
}

