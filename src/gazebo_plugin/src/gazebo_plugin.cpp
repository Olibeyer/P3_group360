#ifndef CRUSTCRAWLER_CONTROL
#define CRUSTCRAWLER_CONTROL

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
class CrustCrawlerPlugin : public ModelPlugin
{
public:
    CrustCrawlerPlugin() : ModelPlugin()
    {
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized())
        {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
        }

        // Just output a message for now
        ROS_INFO("\nThe plugin is attach to model:");
        std::cout << (_model->GetName());

        ROS_INFO("____________________________________YO______________________________________");
        _model->GetJoint("joint1")->SetVelocity(0,100.0);
    }

};
GZ_REGISTER_MODEL_PLUGIN(CrustCrawlerPlugin)
}

#endif