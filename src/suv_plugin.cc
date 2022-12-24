#ifndef _SUV_PLUGIN_HH_
#define _SUV_PLUGIN_HH_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// #include "geometry_msgs/Twist.h"
// #include "ros/callback_queue.h"
#include <ros/ros.h>

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
        // Store the model pointer for convenience.
        this->model = _model;

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->Name());
        std::string topicName = "/my_suv_controller/vel_cmd";

        // Subscribe to the topic, and register a callback
        this->sub = this->node->Subscribe(topicName, &SUVPlugin::OnMsg, this);

        initial_pose = this->model->WorldPose();

        std::cerr << "注册成功!" << std::endl;
    }

    void SetVelInBodyFrame(const ignition::math::Vector3d& lin,
                           const ignition::math::Vector3d& ang) {
        auto current_pose = this->model->WorldPose();
        auto linear = current_pose.Pos();
        auto angular = current_pose.Rot();
        Eigen::Quaterniond q;
        q.x() = angular.X();
        q.y() = angular.Y();
        q.z() = angular.Z();
        q.w() = angular.W();
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        Eigen::Vector3d p(linear.X(), linear.Y(), linear.Z());
        Eigen::Matrix3d p_skew;
        p_skew << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
        Eigen::Vector3d angular_world, linear_world;
        Eigen::Vector3d angular_body, linear_body;
        angular_body.x() = ang.X();
        angular_body.y() = ang.Y();
        angular_body.z() = ang.Z();
        linear_body.x() = lin.X();
        linear_body.y() = lin.Y();
        linear_body.z() = lin.Z();
        // adjoint representation of T=(R, p), 不知道为什么不对，线速度多出来的那一项到底是啥
        angular_world = R * angular_body;
        linear_world = /* p_skew * R * angular_body + */ R * linear_body;

        ignition::math::Vector3d ang_world(angular_world.x(), angular_world.y(), angular_world.z());
        ignition::math::Vector3d lin_world(linear_world.x(), linear_world.y(), linear_world.z());

        ROS_INFO("(v, w): %.1f,%.1f,%.1f, %.1f,%.1f,%.1f", lin_world.X(), lin_world.Y(),
                 lin_world.Z(), ang_world.X(), ang_world.Y(), ang_world.Z());
        this->model->SetLinearVel(lin_world);
        this->model->SetAngularVel(ang_world);
    }

    void SetVelocity(const gazebo::msgs::Int& _msg) {
        // 设置模型线速度
        ignition::math::Vector3<double> vel(0, 0, 0);
        ignition::math::Vector3<double> ang(0, 0, 0);
        char c = _msg.data();
        const float speed = 0.4, ang_speed = 0.2;
        std::cout << "Op = " << c << std::endl;
        switch (c) {
            case 'r':  // 重设为初始位姿并静止
                this->model->SetWorldPose(initial_pose);
                SetVelInBodyFrame(vel, ang);
                //                std::cout << "reset" << std::endl;
                break;
            case 'w':
                vel.Set(speed, 0, 0);
                SetVelInBodyFrame(vel, ang);
                break;
            case 'a':
                vel.Set(0, speed, 0);
                SetVelInBodyFrame(vel, ang);
                break;
            case 's':
                vel.Set(-speed, 0, 0);
                SetVelInBodyFrame(vel, ang);
                break;
            case 'd':
                vel.Set(0, -speed, 0);
                SetVelInBodyFrame(vel, ang);
                break;
            case 'q':
                ang.Set(0, 0, ang_speed);
                SetVelInBodyFrame(vel, ang);
                break;
            case 'e':
                ang.Set(0, 0, -ang_speed);
                SetVelInBodyFrame(vel, ang);
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
    void OnMsg(ConstIntPtr& _msg) { this->SetVelocity(*_msg); }

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
