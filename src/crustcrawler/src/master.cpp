#include "ros/ros.h"
#include "std_msgs/String.h"


 /*  used for subscriber example
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");
  ros::NodeHandle n;

  // example subscriber
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
