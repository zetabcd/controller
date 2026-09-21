#include <px4ctrl/fcu_reboot.h>

#include <memory>

namespace px4ctrl {
namespace {
constexpr uint8_t kSystem = 1;
constexpr uint16_t kComponent = 191;
constexpr auto kReplyTimeout = std::chrono::seconds(5);
}

FcuRebootClient::FcuRebootClient(rclcpp::Node &node) : node_(node)
{
    publisher_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", rclcpp::QoS(10).reliable().durability_volatile());
    subscription_ = node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
        // Foxy supports shared-pointer callbacks, but not const Message & callbacks.
        [this](std::shared_ptr<const px4_msgs::msg::VehicleCommandAck> ack) { on_ack(*ack); });
}

void FcuRebootClient::update(uint64_t aux5_changes, bool allowed)
{
    const auto now = std::chrono::steady_clock::now();
    if (request_active_) {
        if (aux5_changes != 0) {
            RCLCPP_WARN(node_.get_logger(),
                "AUX5 PX4 reboot ignored: previous request is still in the 5-second guard interval");
        }
        if (now - sent_at_ >= kReplyTimeout) {
            if (ack_pending_) {
                RCLCPP_WARN(node_.get_logger(),
                    "PX4 reboot ACK timed out; PX4 may have disconnected to reboot. Outcome unknown, not retrying");
            }
            request_active_ = false;
            ack_pending_ = false;
        }
        // Drop changes even on the iteration that ends the guard interval.
        return;
    }
    if (aux5_changes == 0) return;
    if (!allowed) {
        RCLCPP_WARN(node_.get_logger(),
            "AUX5 PX4 reboot ignored: requires fresh RC/status and DISARMED");
        return;
    }

    px4_msgs::msg::VehicleCommand command{};
    command.timestamp = node_.now().nanoseconds() / 1000;
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    command.param1 = 1.0F; // Reboot the autopilot.
    command.param2 = 0.0F; // Leave the onboard computer running.
    command.target_system = kSystem;
    command.target_component = 1;
    command.source_system = kSystem;
    command.source_component = kComponent;
    command.confirmation = 0; // First transmission; this is not a boolean ACK request.
    command.from_external = true;
    request_active_ = true;
    ack_pending_ = true;
    sent_at_ = now;
    // Multiple changes collected by one spin_some() request only one reboot.
    publisher_->publish(command);
    RCLCPP_WARN(node_.get_logger(), "AUX5 changed while disarmed: PX4 autopilot reboot requested (command 246)");
}

void FcuRebootClient::on_ack(const px4_msgs::msg::VehicleCommandAck &ack)
{
    if (!request_active_ || !ack_pending_ ||
        ack.command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN ||
        ack.target_system != kSystem || ack.target_component != kComponent) return;
    // Standard ACKs do not echo a private request sequence in result_param2.
    if (ack.result == ack.VEHICLE_CMD_RESULT_IN_PROGRESS) return;
    ack_pending_ = false;
    if (ack.result == ack.VEHICLE_CMD_RESULT_ACCEPTED) {
        RCLCPP_INFO(node_.get_logger(),
            "PX4 accepted the autopilot reboot command; wait for PX4/DDS to reconnect (reboot completion not confirmed)");
    } else {
        RCLCPP_ERROR(node_.get_logger(),
            "PX4 autopilot reboot rejected/failed (result=%u); not retrying. Check arming state and board reboot support",
            ack.result);
    }
}

} // namespace px4ctrl
