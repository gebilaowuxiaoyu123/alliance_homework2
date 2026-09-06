#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::motor_demo {

class JoystickVelocityMapping
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    JoystickVelocityMapping()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , max_velocity_(get_parameter("max_velocity").as_double()) {

        register_input("/remote/joystick/left", joystick_);
        register_output("/motor_demo/target_velocity", target_velocity_, 0.0);
    }

    void update() override {
        // 摇杆未连接/断连时输出 0
        *target_velocity_ = joystick_.ready() ? joystick_->y() * max_velocity_ : 0.0;
    }

private:
    double max_velocity_;

    InputInterface<Eigen::Vector2d> joystick_;
    OutputInterface<double> target_velocity_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::JoystickVelocityMapping, rmcs_executor::Component)
