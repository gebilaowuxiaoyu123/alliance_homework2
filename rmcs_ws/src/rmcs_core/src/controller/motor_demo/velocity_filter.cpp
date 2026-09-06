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
            *velocity_filtered_ = low_pass_.update(median_.update(*velocity_));
    }

private:
    // median_window 缺省 5(奇数窗口)
    std::size_t median_window_param() {
        return has_parameter("median_window")
                   ? static_cast<std::size_t>(get_parameter("median_window").as_int())
                   : 5;
    }

    InputInterface<double> velocity_;
    OutputInterface<double> velocity_filtered_;

    rmcs_core::filter::MedianFilter median_;
    rmcs_core::filter::LowPassFilter<> low_pass_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::VelocityFilter, rmcs_executor::Component)
