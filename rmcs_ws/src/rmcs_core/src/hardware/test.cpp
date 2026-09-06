#include <cstddef>
#include <memory>
#include <span>
#include <utility>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class MotorTest
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    MotorTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<MotorCommand>(get_component_name() + "_command", *this))
        , motor_(*this, *command_, "/motor_demo/motor")
        , dr16_{} {
        motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                .set_reversed()
                .enable_multi_turn_angle());

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);
    }

    void update() override {
        motor_.update_status();
        dr16_.update_status();
        remote_control_->update();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        auto packet = device::CanPacket8{};
        packet << motor_;

        builder.can_transmit(
            Spec::kCans.kCan1, {.can_id = motor_.send_id(), .can_data = packet.as_bytes()});
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can == Spec::kCans.kCan1)
            motor_.match_then_store_status(data.can_id, data.can_data);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

private:
    std::unique_ptr<librmcs::board::CBoard> board_;

    class MotorCommand : public rmcs_executor::Component {
    public:
        explicit MotorCommand(MotorTest& motor)
            : motor_(motor) {}
        void update() override { motor_.command_update(); }
    private:
        MotorTest& motor_;
    };
    std::shared_ptr<MotorCommand> command_;

    device::DjiMotor motor_;
    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorTest, rmcs_executor::Component)
