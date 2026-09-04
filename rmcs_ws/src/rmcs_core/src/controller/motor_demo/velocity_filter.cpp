// ============================================================================
// 任务二软件层组件②：电机测速 一阶低通滤波
//   读 /motor_demo/motor/velocity (DjiMotor 原始测速)
//   出 /motor_demo/motor/velocity_filtered (滤波后，给 PID 当反馈)
//   参数 cutoff_frequency / sampling_frequency (Hz)
// ============================================================================

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::motor_demo {

class VelocityFilter
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VelocityFilter()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , low_pass_(
              get_parameter("cutoff_frequency").as_double(),
              get_parameter("sampling_frequency").as_double()) {

        register_input("/motor_demo/motor/velocity", velocity_);
        register_output("/motor_demo/motor/velocity_filtered", velocity_filtered_, 0.0);
    }

    void update() override {
        if (velocity_.ready())
            *velocity_filtered_ = low_pass_.update(*velocity_);
    }

private:
    InputInterface<double> velocity_;
    OutputInterface<double> velocity_filtered_;

    rmcs_core::filter::LowPassFilter<> low_pass_;   // 默认模板参数=单变量 double
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::VelocityFilter, rmcs_executor::Component)
