// ============================================================================
// 模式设定组件：在 yaml 里一键切 角度/速度/力矩 三模式 + 设固定目标值
// ----------------------------------------------------------------------------
//   yaml (motor_command_mode 段):
//     mode: angle | velocity | torque        ← 控制模式
//     angle / velocity / torque              ← 对应模式的固定目标(开机默认值)
//     angle_kp / angle_velocity_max          ← angle 模式内置"外环 P"参数
//
// 三模式输出（各自接不同下游）：
//   angle    : 读当前角 → 优弧误差 → 出 target_velocity(交给内环速度PID)
//              ★额外广播 target_angle / angle_error 供 Foxglove 叠图比对
//   velocity : 出 target_velocity(交给内环速度PID)
//              ★测试接口: velocity_waveform = none | square | sine
//                让"目标速度"成为时间的函数 v(t)，由 yaml 驱动，无需外部 topic
//   torque   : 直驱！出 control_torque 给电机(绕过所有 PID, yaml 需停用速度PID)
//
// 外部覆盖：yaml 固定值只是默认；收到下列 topic 后即被实时覆盖
//   angle    ← /motor_demo/angle_cmd      (Float64, rad)
//   velocity ← /motor_demo/velocity_cmd   (Float64, rad/s)
//   torque   ← /motor_demo/torque_cmd     (Float64, N·m)
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

        // mode 参数在构造时就能读（yaml 里 motor_command_mode.ros__parameters.mode）
        mode_ = get_parameter("mode").as_string();

        if (mode_ == "angle") {
            // 角度模式：读当前多圈角，输出优弧误差 + 目标速度(内置 P 映射)
            register_input("/motor_demo/motor/angle", current_angle_);
            register_output("/motor_demo/target_angle", target_angle_out_, 0.0);
            register_output("/motor_demo/angle_error", angle_error_out_, 0.0);
            register_output("/motor_demo/target_velocity", target_velocity_out_, 0.0);

            target_ = get_parameter("angle").as_double();
            kp_     = get_parameter("angle_kp").as_double();
            vmax_   = get_parameter("angle_velocity_max").as_double();

            cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
                "/motor_demo/angle_cmd", rclcpp::QoS{1},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) { target_ = msg->data; });
        } else if (mode_ == "velocity") {
            // 速度模式：出 target_velocity。默认常数；可开"波形测试接口"
            register_output("/motor_demo/target_velocity", target_velocity_out_, 0.0);
            target_ = get_parameter("velocity").as_double();

            // ★测试接口: velocity_waveform = none(常数) | square | sine
            //   目标速度 = 时间的函数 v(t)，由 yaml 参数驱动
            waveform_ = get_parameter("velocity_waveform").as_string();
            if (waveform_ != "none") {
                amplitude_ = get_parameter("waveform_amplitude").as_double();
                frequency_ = get_parameter("waveform_frequency_hz").as_double();
                cycles_    = get_parameter("waveform_cycles").as_int();
            }
            t0_ = now();   // 波形时间起点(组件启动时刻)

            cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
                "/motor_demo/velocity_cmd", rclcpp::QoS{1},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) { target_ = msg->data; });
        } else {
            // 力矩模式：直驱（yaml 里必须停用速度PID，否则两个组件抢写 control_torque）
            register_output("/motor_demo/motor/control_torque", torque_out_, 0.0);
            target_ = get_parameter("torque").as_double();

            cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
                "/motor_demo/torque_cmd", rclcpp::QoS{1},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) { target_ = msg->data; });
        }
    }

    void update() override {
        if (mode_ == "angle") {
            if (!current_angle_.ready())
                return;
            // 优弧误差卷绕到 [-π,π)，内置 P 映射成目标速度(限幅防飞)
            const double error =
                std::remainder(target_ - *current_angle_, 2.0 * std::numbers::pi);
            // 广播的 target_angle 用"抬升值"= 当前角 + 卷绕误差（电机实际要去的
            // 连续位置）。这样多圈累计后 Foxglove 里 target 能与 angle 重合，
            // 不会因为差了整圈 2π 而"看着像没追上"。
            *target_angle_out_    = *current_angle_ + error;
            *angle_error_out_     = error;
            *target_velocity_out_ = std::clamp(kp_ * error, -vmax_, vmax_);
        } else if (mode_ == "velocity") {
            // 目标速度 = 时间的函数 v(t)（常数 or 方波/正弦波形测试）
            *target_velocity_out_ =
                velocity_target_of_time((now() - t0_).seconds());
        } else {
            *torque_out_ = target_;
        }
    }

private:
    // ★测试接口核心：目标速度 随 时间 的函数 v(t)
    double velocity_target_of_time(double t) {
        if (waveform_ == "none")
            return target_;                              // 常数
        const double period = 1.0 / frequency_;
        if (cycles_ > 0 && t >= cycles_ * period)
            return 0.0;                                  // 跑完指定周期 → 自动归零停车
        const double phase = std::fmod(t, period);
        if (waveform_ == "square")
            return (phase < 0.5 * period) ? amplitude_ : -amplitude_;   // 方波 ±A
        if (waveform_ == "sine")
            return amplitude_ *
                   std::sin(2.0 * std::numbers::pi * frequency_ * t);   // 正弦
        return target_;
    }
    std::string mode_;
    double target_ = 0.0;   // 当前目标(初始=yaml 固定值，收到 cmd 后更新)
    double kp_     = 0.0;   // angle 模式: 角度误差→速度增益(外环P)
    double vmax_   = 0.0;   // angle 模式: 目标速度限幅

    // —— 速度波形测试接口参数 ——
    std::string waveform_ = "none";   // none | square | sine
    double amplitude_ = 0.0;           // 波形幅值(rad/s)
    double frequency_ = 1.0;           // 波形频率(Hz)
    int64_t cycles_   = 0;             // 跑满几周期后归零; 0=一直跑
    rclcpp::Time t0_;                  // 波形时间起点

    InputInterface<double> current_angle_;
    OutputInterface<double> target_angle_out_;
    OutputInterface<double> angle_error_out_;
    OutputInterface<double> target_velocity_out_;
    OutputInterface<double> torque_out_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::CommandModeController, rmcs_executor::Component)
