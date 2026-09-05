// ============================================================================
// 速度/力矩 目标源组件（test.yaml 的“方案B”用）——只负责出“目标”，不碰角度环
// ----------------------------------------------------------------------------
//   角度测试(方案A)走独立链：AngleTargetController → ErrorPidController(外环)
//   （见 test.yaml 顶部两套方案说明；外环 kp/ki 在 motor_angle_pid_controller 段）
//
//   yaml (motor_command_mode 段)：
//     mode: velocity | torque      （若误写 angle，将警告并强制转 velocity）
//   velocity: 出目标速度 → 内环速度PID
//             可：常数(velocity) / 方波(velocity_waveform: square) / 正弦(sine)
//   torque  : 直驱！出 control_torque（★需注释掉 components 里的内环PID那行）
//
// 输出(两模式都注册，broadcaster 永远不用改)：
//   /motor_demo/target_velocity  /motor_demo/target_angle(占位)  /motor_demo/angle_error(占位)
//   （torque 模式额外 /motor_demo/motor/control_torque）
// 外部 topic 可在“非方波”时覆盖固定目标：velocity_cmd / torque_cmd
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller::motor_demo {

class CommandModeController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CommandModeController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        mode_ = get_parameter("mode").as_string();
        // 角度外环已由方案A(独立 ErrorPidController 块)承接；本组件只做 速度/力矩 目标源
        if (mode_ == "angle") {
            RCLCPP_WARN(this->get_logger(),
                        "角度模式已改为独立外环PID(方案A)；本组件仅用 velocity/torque，已强制转 velocity");
            mode_ = "velocity";
        }

        // 通用输出（broadcaster 不用改）
        register_input("/motor_demo/motor/angle", current_angle_);
        register_output("/motor_demo/target_velocity", target_velocity_out_, 0.0);
        register_output("/motor_demo/target_angle", target_angle_out_, 0.0);
        register_output("/motor_demo/angle_error", angle_error_out_, 0.0);

        if (mode_ == "velocity") {
            target_ = get_parameter("velocity").as_double();

            velocity_waveform_ = get_parameter("velocity_waveform").as_string();
            if (velocity_waveform_ != "none") {
                v_amplitude_ = get_parameter("waveform_amplitude").as_double();
                v_freq_      = get_parameter("waveform_frequency_hz").as_double();
                v_cycles_    = get_parameter("waveform_cycles").as_int();
            }
            t0_ = now();

            cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
                "/motor_demo/velocity_cmd", rclcpp::QoS{1},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) { target_ = msg->data; });
        } else {  // torque
            target_ = get_parameter("torque").as_double();
            register_output("/motor_demo/motor/control_torque", torque_out_, 0.0);

            cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
                "/motor_demo/torque_cmd", rclcpp::QoS{1},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) { target_ = msg->data; });
        }
    }

    void update() override {
        if (mode_ == "velocity") {
            *target_velocity_out_ = velocity_target_of_time((now() - t0_).seconds());
            if (current_angle_.ready()) {
                *target_angle_out_ = *current_angle_;   // 观测占位
                *angle_error_out_  = 0.0;
            }
        } else {  // torque
            *torque_out_ = target_;
            if (current_angle_.ready()) {
                *target_angle_out_ = *current_angle_;   // 观测占位
                *angle_error_out_  = 0.0;
            }
            *target_velocity_out_ = 0.0;
        }
    }

private:
    // 目标速度 = 时间的函数 v(t)（none=常数；square/sine 波形）
    double velocity_target_of_time(double t) {
        if (velocity_waveform_ == "none")
            return target_;
        const double period = 1.0 / v_freq_;
        if (v_cycles_ > 0 && t >= v_cycles_ * period)
            return 0.0;
        const double phase = std::fmod(t, period);
        if (velocity_waveform_ == "square")
            return (phase < 0.5 * period) ? v_amplitude_ : -v_amplitude_;
        if (velocity_waveform_ == "sine")
            return v_amplitude_ * std::sin(2.0 * std::numbers::pi * v_freq_ * t);
        return target_;
    }

    std::string mode_;

    double target_ = 0.0;   // 固定目标/外部覆盖(velocity/torque 共用)

    std::string velocity_waveform_ = "none";   // none|square|sine
    double v_amplitude_ = 0.0, v_freq_ = 1.0;
    int64_t v_cycles_ = 0;

    rclcpp::Time t0_;

    InputInterface<double> current_angle_;
    OutputInterface<double> target_velocity_out_;
    OutputInterface<double> target_angle_out_;
    OutputInterface<double> angle_error_out_;
    OutputInterface<double> torque_out_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::CommandModeController, rmcs_executor::Component)

