#ifndef _SUV_PLUGIN_HH_
#define _SUV_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <thread>

#include "geometry_msgs/Twist.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo {
/// \brief A plugin to control a SUV sensor.
class SUVPlugin : public ModelPlugin {
    /// \brief Constructor
   public:
    SUVPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
	ROS_WARN("Load!------------");
	// Store the model pointer for convenience.
	this->model = _model;

	// Create the node
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(this->model->GetWorld()->Name());

	// Create a topic name
	//		std::string topicName = "~/" + this->model->GetName() +
	//"/vel_cmd";
	std::string topicName = "/my_suv_controller/vel_cmd";

	// Subscribe to the topic, and register a callback
	this->sub = this->node->Subscribe(topicName, &SUVPlugin::OnMsg, this);

	initial_pose = this->model->WorldPose();

	std::cerr << "注册成功!" << std::endl;
    }

    void SetVelocity(const gazebo::msgs::GzString& _msg) {
	// 设置模型线速度
	//	  const gazebo::msgs::Vector3d& linear = _msg.linear();
	//	  ignition::math::Vector3<double> vel;
	//	  vel.Set(linear.x(), linear.y(), linear.z());
	//      this->model->SetLinearVel(vel);
	ignition::math::Vector3<double> vel;
	char c = _msg.data()[0];
	switch (c) {
	    case 'r':  // 重设为初始位姿并静止
		this->model->SetWorldPose(initial_pose);
		vel.Set(0, 0, 0);
		this->model->SetLinearVel(vel);
		std::cout << "reset" << std::endl;
		break;
	    case 's':  // 前进
		vel.Set(0, -0.4, 0);
		this->model->SetLinearVel(vel);
		std::cout << "forward" << std::endl;
		break;
	    default:
		break;
	}
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
   private:
    /* 坑……这里不能用 const gazebo::msgs::TwistPtr, 因为它是const
     * boost::shared_ptr<gazebo::msgs::Twist 而应该用ConstTwistPtr, 它是const
     * boost::shared_ptr<gazebo::msgs::Twist const>
     */
    void OnMsg(ConstGzStringPtr& _msg) { this->SetVelocity(*_msg); }

   private:
    physics::ModelPtr model;
    // physics::JointPtr joint;
    transport::NodePtr node;
    transport::SubscriberPtr sub;
    ignition::math::Pose3d initial_pose;

   private:
    // std::unique_ptr<ros::NodeHandle> rosNode;
    // ros::Subscriber rosSub;
    // ros::CallbackQueue rosQueue;
    // std::thread rosQueueThread;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(SUVPlugin)
}  // namespace gazebo
#endif
