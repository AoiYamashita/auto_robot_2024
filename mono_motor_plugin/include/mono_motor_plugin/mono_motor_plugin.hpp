#ifndef MOTOR_GAZEBO_PLUGIN_HPP_
#define MOTOR_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>

#include "std_msgs/msg/float32.hpp"

#include <memory>

namespace gazebo{
    class MonoMotorPluginPrivate;
    class MonoMotorPlugin : public gazebo::ModelPlugin
    {
    public:
        /// Constructor:
        MonoMotorPlugin();

        /// Destructor:
        virtual ~MonoMotorPlugin();

        // LOAD plugin:
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        void ForceSubCallBack(const std_msgs::msg::Float32::SharedPtr msg_) const;

    private:

        std::unique_ptr<MonoMotorPluginPrivate> impl_;
    };
}

#endif