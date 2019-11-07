
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Float64MultiArray.h>

_Float64 posRobot[5];
_Float64 velRobot[5];
_Float64 posDesired[5];
_Float64 velDesired[5];
_Float64 accDesired[5];
float kp = 20;
float kv = 0;

struct Vector3
{
  float x;
  float y;
  float z;
};

void angleFunk(const std_msgs::Float64MultiArray &robotAngles_incomming)
{
  for (int i = 0; i < 5; i++)
  {
    int dataindex = i * 2;
    posRobot[i] = robotAngles_incomming.data[dataindex];
    posRobot[i] = robotAngles_incomming.data[dataindex + 1];
  }
}

void trajectoryFunk(const std_msgs::Float64MultiArray &trajectoryAngels_incomming)
{
  for (int i; i < 5; i++)
  {
    int dataindex = i * 3;
    posDesired[i] = trajectoryAngels_incomming.data[dataindex];
    velDesired[i] = trajectoryAngels_incomming.data[dataindex + 1];
    accDesired[i] = trajectoryAngels_incomming.data[dataindex + 2];
  }
}

Vector3 getErrorPos()
{
  Vector3 posError;
  posError.x = posDesired[0] - posRobot[0];
  posError.y = posDesired[1] - posRobot[1];
  posError.z = posDesired[2] - posRobot[2];

  return posError;
}
Vector3 getErrorVel()
{
  Vector3 velError;
  velError.x = velDesired[0] - velRobot[0];
  velError.y = velDesired[1] - velRobot[1];
  velError.z = velDesired[2] - velRobot[2];

  return velError;
}

Vector3 calculateTorque(Vector3 posError, Vector3 velError)
{
  Vector3 tau;
  Vector3 tmark;

  tmark.x = kp * posError.x + kv * velError.x + accDesired[0];
  tmark.y = kp * posError.y + kv * velError.y + accDesired[1];
  tmark.z = kp * posError.z + kv * velError.z + accDesired[2];

  float H11, H12, H13, H21, H22, H23, H31, H32, H33, VG1, VG2, VG3;

  tau.x = H12 * tmark.x + H12 * tmark.y + H13 * tmark.z + VG1;
  tau.y = H21 * tmark.x + H22 * tmark.y + H23 * tmark.z + VG2;
  tau.z = H31 * tmark.x + H32 * tmark.y + H33 * tmark.z + VG3;

  return tau;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher torque_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/setTorques", 1);
  ros::Subscriber angleGetter_sub = n.subscribe("crustcravler/getAngleVel", 1, angleFunk);
  ros::Subscriber desiredAngle_sub = n.subscribe("crustcravler/trajectory", 1, trajectoryFunk);

  ros::Rate loop_rate(30);
  int count = 0;

  while (ros::ok())
  {

    Vector3 posError = getErrorPos();
    Vector3 velError = getErrorVel();

    calculateTorque(posError, velError);


    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}
