#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros_myo/MyoPose.h>
#include <tf/transform_broadcaster.h>


float angles[3];
uint8_t gesture = 0;

void myo_raw_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  float x = msg->pose.orientation.x;
  float y = msg->pose.orientation.y;
  float z = msg->pose.orientation.z;
  float w = msg->pose.orientation.w;

  float roll, pitch, yaw;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
  pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}

void myo_raw_gest_callback(const ros_myo::MyoPose::ConstPtr& msg){
  gesture = msg->pose;
  /*
  UNKNOWN = 0
  REST = 1
  FIST = 2
  WAVE_IN = 3
  WAVE_OUT = 4
  FINGERS_SPREAD = 5
  THUMB_TO_PINKY = 6
  */
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber pose_sub = n.subscribe("myo_raw/pose", 10, myo_raw_pose_callback);
  ros::Subscriber gest_sub = n.subscribe("myo_raw/gest", 10, myo_raw_gest_callback);

  ros::Rate loop_rate(30);

  const double degree = M_PI/180;

  // message declarations
  sensor_msgs::JointState joint_state;

  uint8_t pos3 = 0;
  uint8_t grip = 0;

  while (ros::ok()) {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(5);
    joint_state.position.resize(5);
    joint_state.name[0] ="joint1";
    joint_state.position[0] = -angles[0];
    joint_state.name[1] ="joint2";
    joint_state.position[1] = -angles[1];
    joint_state.name[2] ="joint3";
    joint_state.position[2] = pos3;
    joint_state.name[3] ="joint4";
    joint_state.position[3] = grip;
    joint_state.name[4] ="joint5";
    joint_state.position[4] = -grip;

    if (gesture == 3) {
      pos3 += 0.1;
    }
    else if (gesture == 4) {
      pos3 -= 0.1;
    }

    if (gesture == 1) {
      grip += 0.1;
    }
    else if (gesture == 5) {
      grip -= 0.1;
    }



    //send the joint state and transform
    joint_pub.publish(joint_state);


    ros::spinOnce();

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }


  return 0;
}
