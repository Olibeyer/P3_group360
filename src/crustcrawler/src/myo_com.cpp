#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "myo_com");

  ros::NodeHandle n;


  // example publisher
  //  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);  //this is the loop rate of the node (set this to wanted sampling time)


  while (ros::ok())
  {


    //get myo data

    //pack it up

    //publish it

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
