#ifndef _CATAPULT_PLUGIN_HH_
#define _CATAPULT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "catapult_control/Catapult.h"
#include <iostream>
#include <thread>

namespace gazebo
{
  // A plugin to control a Catapult.
  class CatapultPlugin : public ModelPlugin
  {
    public: CatapultPlugin() {}

    // Function called by Gazebo when the plugin is inserted into simulation
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Catapult plugin not loaded\n";
        return;
      }else{
        std::cout << "Catapult plugin loaded\n";
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      const std::string jointName = "catapult__arm_base_geom_arm_trunk_JOINT__revolute";
      this->joint = _model->GetJoint(jointName);

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);
      
      this->model->GetJointController()->SetPositionPID(
          this->joint->GetScopedName(), this->pid);


      // Default to zero velocity/upper_limit
      //double velocity = 0;
      //double upper_limit = 0;
      //this->SetUpperLimit(upper_limit);
      //this->SetVelocity(velocity);

      
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_catapult_control_listener",
            ros::init_options::NoSigintHandler);
      }

      // Create the ROS node.
      this->rosNode.reset(new ros::NodeHandle("gazebo_catapult_control_listener"));

      // Create CatapultConstPtr needed for boost
      typedef boost::shared_ptr<catapult_control::Catapult const> CatapultConstPtr;

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<catapult_control::Catapult>(
            "/" + this->model->GetName() + "/chatter",
            1,
            boost::bind(&CatapultPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);
      this->rosQueueThread = std::thread(std::bind(&CatapultPlugin::QueueThread, this));
    }

    public: void OnRosMsg(const catapult_control::CatapultConstPtr &_msg)
    {
      this->SetVelocity(_msg->velocity);
      this->SetUpperLimit(_msg->upper_limit);
    }

    public: void SetVelocity(const double &_vel)
    {
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    public: void SetUpperLimit(const double &_upper_limit)
    {
      this->joint->SetHighStop(0,_upper_limit);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.001;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief A node used for transport
    private: transport::NodePtr node;
    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;
    /// \brief A PID controller for the joint.
    private: common::PID pid;
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(CatapultPlugin)
}
#endif
