#ifndef _SUV_PLUGIN_HH_
#define _SUV_PLUGIN_HH_

#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <memory>

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
        this->ros_node_ = std::make_unique<ros::NodeHandle>("gazebo_client");
        std::string topicName = "/my_suv_controller/vel_cmd";

        // Subscribe to the topic, and register a callback
        this->sub_ = this->ros_node_->subscribe(topicName, 1, &SUVPlugin::OnRosMsg, this);
        initial_pose = this->model->WorldPose();

        // 类内一定要加this
        // 10Hz的更新频率
        update_rate_ = 10;
        speed_interval_ = 0.1 / update_rate_;
        ang_interval_ = (0.5 / 57.3) / update_rate_;
        car_vel_.Set(0, 0, 0);
        car_ang_.Set(0, 0, 0);
        timer_ = ros_node_->createTimer(ros::Duration(1.0 / update_rate_),
                                        &SUVPlugin::timerCallback, this);
        // 创建一个Service Client，用于调用/gazebo/set_model_state服务
        set_model_state_client_ =
            ros_node_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        std::cerr << "注册成功!" << std::endl;
    }

    void BodyToWorld(const ignition::math::Vector3d& lin, const ignition::math::Vector3d& ang,
                     ignition::math::Vector3d& lin_world, ignition::math::Vector3d& ang_world) {
        auto current_pose = this->model->WorldPose();
        auto linear = current_pose.Pos();
        auto angular = current_pose.Rot();
        Eigen::Quaterniond q;
        q.x() = angular.X();
        q.y() = angular.Y();
        q.z() = angular.Z();
        q.w() = angular.W();
        Eigen::Matrix3d R = q.normalized().toRotationMatrix().inverse();
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

        ang_world.X() = angular_world.x();
        ang_world.Y() = angular_world.y();
        ang_world.Z() = angular_world.z();
        lin_world.X() = linear_world.x();
        lin_world.Y() = linear_world.y();
        lin_world.Z() = linear_world.z();
        std::cout << linear_world.x() << " " << linear_world.y() << " " << linear_world.z()
                  << std::endl;
    }

    void timerCallback(const ros::TimerEvent&) { MoveOneStep(car_vel_, car_ang_); }

    // 差速模型运动一拍
    void MoveOneStep(const ignition::math::Vector3d& lin, const ignition::math::Vector3d& ang) {
        // print every 10 times to reduce the output
        //        static int count = 0;
        //        if (count++ % 10 == 0) {
        //            ROS_INFO("car_vel: %f, %f, %f", car_vel_.X(), car_vel_.Y(), car_vel_.Z());
        //            count = 0;
        //        }

        auto current_pose = this->model->WorldPose();
        ignition::math::Vector3d lin_world, ang_world;
        BodyToWorld(lin, ang, lin_world, ang_world);

        // 创建一个SetModelState消息对象，用于设置模型状态
        gazebo_msgs::SetModelState set_model_state_msg;

        // 设置模型名称
        set_model_state_msg.request.model_state.model_name = "suv";

        // 设置模型位置
        set_model_state_msg.request.model_state.pose.position.x =
            current_pose.Pos().X() + lin_world.X();
        set_model_state_msg.request.model_state.pose.position.y =
            current_pose.Pos().Y() + lin_world.Y();
        set_model_state_msg.request.model_state.pose.position.z = 0.0;

        // 设置模型姿态
        /* 将当前位姿转换为欧拉角，应用当前旋转角速度，再转换为四元数
         * */
        ignition::math::Quaterniond q = current_pose.Rot();
        ignition::math::Vector3d euler = q.Euler();
        euler += ang_world;
        q.Euler(euler);
        set_model_state_msg.request.model_state.pose.orientation.x = q.X();
        set_model_state_msg.request.model_state.pose.orientation.y = q.Y();
        set_model_state_msg.request.model_state.pose.orientation.z = q.Z();
        set_model_state_msg.request.model_state.pose.orientation.w = q.W();

        // 调用服务，设置模型状态
        if (set_model_state_client_.call(set_model_state_msg)) {
            //            ROS_INFO("Set model state success!");
        } else {
            ROS_ERROR("Failed to set model state");
        }
    }

    void SetVelocity(const std_msgs::Int8ConstPtr& _msg) {
        // 设置模型线速度
        char c = _msg->data;
        std::cout << "Op = " << c << std::endl;
        switch (c) {
            case 'r':  // 重设为初始位姿并静止
                car_vel_.Set(0, 0, 0);
                car_ang_.Set(0, 0, 0);
                this->model->SetWorldPose(initial_pose);
                break;
            case 'w':
                car_vel_.Set(car_vel_.X() + speed_interval_, 0, 0);
                break;
            case 's':
                car_vel_.Set(car_vel_.X() - speed_interval_, 0, 0);
                break;
            case 'a':
                car_ang_.Set(0, 0, car_ang_.Z() + ang_interval_);
                break;
            case 'd':
                car_ang_.Set(0, 0, car_ang_.Z() - ang_interval_);
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
    void OnRosMsg(const std_msgs::Int8ConstPtr& _msg) {
        this->SetVelocity(_msg);
        ROS_INFO("car_vel: %f, %f, interval: %f", car_vel_.X(), car_vel_.Y(), speed_interval_);
    }

   private:
    physics::ModelPtr model;
    // physics::JointPtr joint;
    ignition::math::Pose3d initial_pose;
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Subscriber sub_;
    ignition::math::Vector3d car_vel_, car_ang_;
    ros::Timer timer_;
    int update_rate_;
    float speed_interval_, ang_interval_;
    ros::ServiceClient set_model_state_client_;

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
