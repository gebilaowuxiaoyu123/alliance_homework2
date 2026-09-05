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
//   velocity : 直接出固定目标速度 target_velocity(交给内环速度PID)
//   torque   : 直驱！出 control_torque 给电机(绕过所有 PID, yaml 需停用速度PID)
//
// 外部覆盖：yaml 固定值只是默认；收到下列 topic 后即被实时覆盖
//   angle    ← /motor_demo/angle_cmd      (Float64, rad)
//   velocity ← /motor_demo/velocity_cmd   (Float64, rad/s)
//   torque   ← /motor_demo/torque_cmd     (Float64, N·m)
// ============================================================================

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
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
            // 速度模式：直接输出固定目标速度
            register_output("/motor_demo/target_velocity", target_velocity_out_, 0.0);
            target_ = get_parameter("velocity").as_double();

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
            *target_angle_out_    = target_;
            *angle_error_out_     = error;
            *target_velocity_out_ = std::clamp(kp_ * error, -vmax_, vmax_);
        } else if (mode_ == "velocity") {
            *target_velocity_out_ = target_;
        } else {
            *torque_out_ = target_;
        }
    }

private:
    std::string mode_;
    double target_ = 0.0;   // 当前目标(初始=yaml 固定值，收到 cmd 后更新)
    double kp_     = 0.0;   // angle 模式: 角度误差→速度增益(外环P)
    double vmax_   = 0.0;   // angle 模式: 目标速度限幅

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
