#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

#include "mono_motor_plugin/mono_motor_plugin.hpp"     // Header file.

#include <memory>

namespace gazebo{
    class MonoMotorPluginPrivate{
        public:
            // ROS node for communication, managed by gazebo_ros.
            gazebo_ros::Node::SharedPtr ros_node_;

            // The joint that controls the movement of the belt:
            gazebo::physics::JointPtr motor_joint_;

            // Additional parametres:
            double motor_torque_;
            double max_torque_;
            double max_rpm_;
            double motor_rpm_;

            // PUBLISH ConveyorBelt status:
            void PublishStatus();
            void AnglePublisher();

            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr status_pub_;      // Publisher.
            std_msgs::msg::Float32 status_msg_;                                    // motor status.

            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr msg_sub_;      // Subscriber0.

            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;      // angle pub
            std_msgs::msg::Float32 motor_angle_;

            double radius;
            std::string motor_name = "nameless";

            // WORLD UPDATE event:
            void OnUpdate();
            gazebo::event::ConnectionPtr update_connection_;  // Connection to world update event. Callback is called while this is alive.
    };
    
    MonoMotorPlugin::MonoMotorPlugin()
    : impl_(std::make_unique<MonoMotorPluginPrivate>())
    {
    }

    MonoMotorPlugin::~MonoMotorPlugin()
    {
    }

    void MonoMotorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

        impl_->motor_joint_ = _model->GetJoint(_sdf->GetElement("motor_joint")->Get<std::string>());

        if(!impl_->motor_joint_){
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Motor joint not found, unable to start Motor plugin.");
            return;
        }

        impl_->radius = _sdf->GetElement("radius")->Get<double>();

        if(!impl_->radius) impl_->radius = 10;

        impl_->max_rpm_ = 2*M_PI*impl_->radius*_sdf->GetElement("max_rpm")->Get<double>();

        if(!impl_->max_rpm_) impl_->max_rpm_ = 1000;

        impl_->max_torque_ = _sdf->GetElement("max_torque")->Get<double>();

        if(!impl_->max_torque_) impl_->max_torque_ = 100*M_PI*impl_->radius;

        impl_->motor_name = _sdf->GetElement("name")->Get<std::string>();

        impl_->status_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(impl_->motor_name+"/get_rpm", 10);//impl_->motor_name+
        impl_->status_msg_.data = 0;

        impl_->msg_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(impl_->motor_name+"/set_power",10,std::bind(&MonoMotorPlugin::ForceSubCallBack,this,std::placeholders::_1));

        impl_->angle_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(impl_->motor_name+"/get_angle",10);

        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MonoMotorPluginPrivate::OnUpdate, impl_.get()));

        impl_->motor_joint_->SetProvideFeedback(true);

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO MonoMotorPlugin plugin loaded successfully.");
    }

    void MonoMotorPluginPrivate::OnUpdate(){
        //double motor_position = motor_joint_->Position(0);

        //if (motor_rpm_ <= max_rpm_) 
        motor_joint_->SetForce(0, motor_torque_);
        //else motor_joint_->SetForce(0, 0);

        /*if (motor_position >= 900){
            motor_joint_->SetPosition(0, 0);
        }*/

        AnglePublisher();
        PublishStatus();
    }

    void MonoMotorPlugin::ForceSubCallBack(const std_msgs::msg::Float32::SharedPtr msg_) const
    {
        impl_->motor_torque_ = impl_->max_torque_ * (double)msg_->data;
        if ((double)msg_->data <= -1) impl_->motor_torque_ = -impl_->max_torque_;
        if ((double)msg_->data >= 1) impl_->motor_torque_ = impl_->max_torque_;
    }

    void MonoMotorPluginPrivate::AnglePublisher(){
        motor_angle_.data = 180*motor_joint_->Position()/M_PI;
        angle_pub_->publish(motor_angle_);
        //RCLCPP_INFO(impl_->ros_node_->get_logger(), "I heard.\n");
    }

    void MonoMotorPluginPrivate::PublishStatus(){
        motor_rpm_ = 60*motor_joint_->GetVelocity(0)/(2*M_PI);
        status_msg_.data = motor_rpm_;
        status_pub_->publish(status_msg_);
    }
    GZ_REGISTER_MODEL_PLUGIN(MonoMotorPlugin)
}