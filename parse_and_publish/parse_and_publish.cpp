#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/JointState.h"

#define PI 3.1416

// Umrechnung Radian->Degree
// pi=180°
// aRadien/pi*180

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
uint _DataArray[6];

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int i=0;
  for(i=0; i<6; i++)
  {
    //ROS_INFO("I heard: [%d]", uint((msg->position[i])/PI*180));
    _DataArray[i]= uint((msg->position[i])/PI*180);
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "parse_and_publish");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("joint_states", 6, chatterCallback);

  ros::Publisher pub = n.advertise<std_msgs::UInt8MultiArray>("joint_array", 6);
  
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::UInt8MultiArray array;
    //Clear array
    array.data.clear();

    //for loop, pushing data in the size of the array
    for (int i = 0; i < 6; i++)
    {
    //assign array a random number between 0 and 255.
     array.data.push_back(_DataArray[i]);
    }
    pub.publish(array);

    ros::spinOnce();

    loop_rate.sleep();
  }
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  

  return 0;
}