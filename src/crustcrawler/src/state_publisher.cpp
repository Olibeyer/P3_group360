#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // message declarations
    sensor_msgs::JointState joint_state;

    float pos = 0;
    bool up = true;


    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5);
        joint_state.position.resize(5);
        joint_state.name[0] ="joint1";
        joint_state.position[0] = pos;
        joint_state.name[1] ="joint2";
        joint_state.position[1] = 0;
        joint_state.name[2] ="joint3";
        joint_state.position[2] = 0;
        joint_state.name[3] ="joint4";
        joint_state.position[3] = 0;
        joint_state.name[4] ="joint5";
        joint_state.position[4] = 0;


        if(pos > 3.14)
          up = !up;
        if(pos < 0)
          up = !up;

        if(up)
          pos += 0.1 ;
        else
          pos -= 0.1;



        //send the joint state and transform
        joint_pub.publish(joint_state);



        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
