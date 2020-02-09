#ifndef _POSITION_PLUGIN_HH_
#define _POSITION_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control joint position.
  class PositionPlugin : public ModelPlugin
  {
    /// \brief Constructor 
    public: PositionPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to. 
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, position plugin not loaded\n";
        return;
      }
      
      // Default value
      double position = 0.0, p_gain, i_gain, d_gain;

      // Check that the element exists, then read the value
      if (_sdf->HasElement("jointname"))
        joint_name_ori = _sdf->Get<std::string>("jointname");
        joint_name = _model->GetScopedName() + "::" + _model->GetScopedName() + "::" + joint_name_ori;
        std::cerr << "We find the joint [" <<
        joint_name << "]\n";
      if (_sdf->HasElement("p_gain"))
        p_gain = _sdf->Get<double>("p_gain");
      if (_sdf->HasElement("i_gain"))
        i_gain = _sdf->Get<double>("i_gain");
      if (_sdf->HasElement("d_gain"))
        d_gain = _sdf->Get<double>("d_gain");


      // Store the model pointer for convenience.
      this->model = _model;
      std::cerr << "\n The model's name is [" <<
        _model->GetScopedName() << "]\n";


      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(p_gain, i_gain, d_gain);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetPositionPID(
          joint_name, this->pid);

      // Set the joint's target position.
      this->model->GetJointController()->SetPositionTarget(
          joint_name, position);
        
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, joint_name_ori + "_" + "node",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node.
      this->rosNode.reset(new ros::NodeHandle(joint_name_ori + "_" + "Handle"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/" + joint_name_ori + "/pos_cmd",
            1,
            boost::bind(&PositionPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&PositionPlugin::QueueThread, this));
    }
    
    /// \brief Set the position of the joint
    /// \param[in] _pos New target position
    public: void SetPosition(const double &_pos)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetPositionTarget(
          joint_name, _pos);
    }
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the position
    /// of the joint.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetPosition(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
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
    
    public: std::string joint_name, joint_name_ori;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin. 
  GZ_REGISTER_MODEL_PLUGIN(PositionPlugin)
}
#endif
