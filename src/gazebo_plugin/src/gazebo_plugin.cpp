#ifndef CRUSTCRAWLER_CONTROL
#define CRUSTCRAWLER_CONTROL

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "string.h"

namespace gazebo
{
class CrustCrawlerPlugin : public ModelPlugin
{
    ros::NodeHandle *n;
    ros::Publisher anglePublisher;
    ros::Subscriber angleSubscriber;
    physics::JointPtr joints[5];
    event::ConnectionPtr updateConnection;
    ros::Time prevRun;
    physics::ModelPtr crustyModel;

public:
    CrustCrawlerPlugin() : ModelPlugin()
    {
        //ros::init(0, 0, "gazebo_plugin_node");
    }
    ~CrustCrawlerPlugin()
    {
        delete n;
        for (int i = 0; i < 5; i++)
        {
            delete &joints[i];
        }
        //delete joints;
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        crustyModel = _model;

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        n = new ros::NodeHandle();
        anglePublisher = n->advertise<std_msgs::Float64MultiArray>("/crustcrawler/getjointangles", 1);
        angleSubscriber = n->subscribe("/crustcrawler/setjointtorques", 1, &CrustCrawlerPlugin::torqueUpdater, this);
        
        ROS_INFO("____________________________________YO______________________________________");
        ROS_INFO_STREAM("The plugin is attach to model: " << _model->GetName());

        joints[0] = _model->GetJoint("joint1");
        joints[1] = _model->GetJoint("joint2");
        joints[2] = _model->GetJoint("joint3");
        joints[3] = _model->GetJoint("joint4");
        joints[4] = _model->GetJoint("joint5");

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CrustCrawlerPlugin::OnUpdate, this));
    }

public:
    void OnUpdate()
    {
        if (!ros::ok())
        {
            ROS_INFO("Failed Ros");
            return;
        }
        ros::Time currentTime = ros::Time::now();
        //if (currentTime.nsec - prevRun.nsec > (1.0 / 30.0) * 1000000000.0)
        if (currentTime.toSec() - prevRun.toSec() > (3.0))
        {
            prevRun = ros::Time::now();
            std_msgs::Float64MultiArray msg;
            msg.data.resize(5);
            for (int i = 0; i < 5; i++)
            {
                ignition::math::Quaternion<double> currentErrorPose = joints[i]->AnchorErrorPose().Rot();
                msg.data[i] = currentErrorPose.Roll();
            }
            anglePublisher.publish(msg);
            ros::spinOnce();
        }
    }

    void torqueUpdater(const std_msgs::Float64MultiArray& incoming) {
        for (int i = 0; i < 5; i++ ) {
            joints[i]->SetVelocity(0, incoming.data[i]);
        }
    }
};
GZ_REGISTER_MODEL_PLUGIN(CrustCrawlerPlugin)
} // namespace gazebo

#endif