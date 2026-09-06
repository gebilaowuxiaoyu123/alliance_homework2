#include <cmath>
#include <cstdint>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller::motor_demo {

class AngleTargetController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleTargetController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/motor_demo/motor/angle", current_angle_);
        register_output("/motor_demo/angle_error", angle_error_, 0.0);
        register_output("/motor_demo/target_angle", target_angle_output_, 0.0);

        // 可选内置角度方波(angle_waveform: square)；缺省听外部 angle_cmd
        if (has_parameter("angle_waveform") &&
            get_parameter("angle_waveform").as_string() == "square") {
            square_active_ = true;
            square_low_    = get_parameter("angle_square_low").as_double();
            square_high_   = get_parameter("angle_square_high").as_double();
            square_half_s_ = get_parameter("angle_square_half_period_s").as_double();
            square_cycles_ = get_parameter("angle_square_cycles").as_int();
            t0_            = now();
        }

        cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/motor_demo/angle_cmd", rclcpp::QoS{1},
            [this](std_msgs::msg::Float64::UniquePtr&& msg) {
                target_angle_ = msg->data;
                has_command_ = true;
            });
    }

    void update() override {
        if (!current_angle_.ready())
            return;

        if (square_active_) {
            const double elapsed = (now() - t0_).seconds();
            const double period  = 2.0 * square_half_s_;
            if (square_cycles_ > 0 && elapsed >= square_cycles_ * period) {
                target_angle_ = *current_angle_;
            } else {
                const double phase = std::fmod(elapsed, period);
                target_angle_ = (phase < square_half_s_) ? square_high_ : square_low_;
            }
        } else if (!has_command_) {
            // 未收到指令前锁定当前角
            target_angle_ = *current_angle_;
        }

        // 优弧误差(卷绕到 [-π,π))；target_angle 广播抬升值(当前角+误差)供叠图
        const double error =
            std::remainder(target_angle_ - *current_angle_, 2.0 * std::numbers::pi);
        *angle_error_ = error;
        *target_angle_output_ = *current_angle_ + error;
    }

private:
    InputInterface<double> current_angle_;
    OutputInterface<double> angle_error_;
    OutputInterface<double> target_angle_output_;

    double target_angle_ = 0.0;
    bool has_command_ = false;

    bool square_active_ = false;
    double square_low_  = 0.0;
    double square_high_ = 0.0;
    double square_half_s_ = 1.0;
    int64_t square_cycles_ = 0;
    rclcpp::Time t0_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::AngleTargetController, rmcs_executor::Component)
