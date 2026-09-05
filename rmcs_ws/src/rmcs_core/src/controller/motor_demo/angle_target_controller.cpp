// ============================================================================
// 任务三 · 角度指令桥组件：外部 ros2 话题的角度 → 卷绕角度误差(优弧)
// ----------------------------------------------------------------------------
//   [ros2 topic pub /motor_demo/angle_cmd <Float64 rad>]  (外部 ROS2 话题)
//        │ create_subscription（复用 omni_infantry.cpp 校准订阅的写法）
//        ▼
//   读内部当前角 /motor_demo/motor/angle （DjiMotor 输出，多圈连续）
//        ▼ std::remainder 卷绕到 [-π, π)  ← 走优弧(短弧)
//   出 /motor_demo/angle_error  → 给外环 ErrorPidController 吃
//   出 /motor_demo/target_angle → 目标角(多圈连续)，供 Foxglove 与 angle 叠图比对
//
// 安全：收到第一条指令前，把目标锁在当前角 → 一启动电机不乱转。
// ============================================================================

#include <cmath>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
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

        // QoS{1}=KeepLast(1)：收发各存最近 1 帧即可（QoS{0} 在 jazzy 会打 KEEP_LAST 警告）
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

        // 安全：还没收到角度指令前，目标锁在当前角 → 启动不乱转
        if (!has_command_)
            target_angle_ = *current_angle_;

        // 优弧：误差卷绕到 [-π, π)。remainder(a, 2π) 返回离 0 最近的余数，
        // 即"从当前角到目标角最近的那条有向弧"→ 正走正、负走负，不绕远路
        const double error =
            std::remainder(target_angle_ - *current_angle_, 2.0 * std::numbers::pi);
        *angle_error_ = error;
        // 目标角广播用"抬升值"= 当前角 + 卷绕误差(显示用)：多圈累计后
        // Foxglove 里能与 angle 重合，避免"差一整圈 2π 看着像没追上"
        *target_angle_output_ = *current_angle_ + error;
    }

private:
    InputInterface<double> current_angle_;
    OutputInterface<double> angle_error_;
    OutputInterface<double> target_angle_output_;

    double target_angle_ = 0.0;
    bool has_command_ = false;   // 是否已收到用户角度指令
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::AngleTargetController, rmcs_executor::Component)
