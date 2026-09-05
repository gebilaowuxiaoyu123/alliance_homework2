// ============================================================================
// 任务二软件层组件②：电机测速 中值 + 一阶低通 串联滤波
//   读 /motor_demo/motor/velocity (DjiMotor 原始测速)
//   出 /motor_demo/motor/velocity_filtered (滤波后，给 PID 当反馈)
//   链路: 中值(砍偶发量化尖刺) → 低通(平滑残余高频)
//   参数 median_window(奇数) / cutoff_frequency / sampling_frequency
// ============================================================================

#include <cstddef>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "filter/low_pass_filter.hpp"
#include "filter/median_filter.hpp"

namespace rmcs_core::controller::motor_demo {

class VelocityFilter
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VelocityFilter()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , median_(median_window_param())
        , low_pass_(
              get_parameter("cutoff_frequency").as_double(),
              get_parameter("sampling_frequency").as_double()) {

        register_input("/motor_demo/motor/velocity", velocity_);
        register_output("/motor_demo/motor/velocity_filtered", velocity_filtered_, 0.0);
    }

    void update() override {
        if (velocity_.ready())
            // 中值先砍尖刺 → 低通再平滑，给内环 PID 干净反馈
            *velocity_filtered_ = low_pass_.update(median_.update(*velocity_));
    }

private:
    // median_window 参数缺省时给 5（奇数窗口），避免别的配置没配就崩
    std::size_t median_window_param() {
        return has_parameter("median_window")
                   ? static_cast<std::size_t>(get_parameter("median_window").as_int())
                   : 5;
    }

    InputInterface<double> velocity_;
    OutputInterface<double> velocity_filtered_;

    rmcs_core::filter::MedianFilter median_;              // ★中值：先砍尖刺
    rmcs_core::filter::LowPassFilter<> low_pass_;          // 低通：再平滑(默认单变量 double)
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::VelocityFilter, rmcs_executor::Component)
