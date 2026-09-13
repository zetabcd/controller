#include <px4ctrl/ekf_restart.h>

namespace px4ctrl {
namespace {
constexpr uint8_t kSystem = 1;
constexpr uint16_t kComponent = 191;
constexpr uint32_t kMaxSequence = 0x00ffffff; // Exactly representable by param2 float32.
}

EkfRestartClient::EkfRestartClient(rclcpp::Node &node) : node_(node)
{
    publisher_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", rclcpp::QoS(10).reliable().durability_volatile());
    subscription_ = node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleCommandAck &ack) { on_ack(ack); });
    sequence_ = static_cast<uint32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count()) & kMaxSequence;
}

void EkfRestartClient::update(uint64_t aux5_changes, bool allowed)
{
    if (!allowed) {
        queued_ = 0;
        if (aux5_changes != 0) {
            RCLCPP_WARN(node_.get_logger(),
                "AUX5 EKF restart ignored: requires fresh RC/status and DISARMED");
        }
    } else {
        queued_ += aux5_changes;
    }

    if (in_flight_) {
        if (std::chrono::steady_clock::now() - sent_at_ > std::chrono::seconds(35)) {
            in_flight_ = false;
            queued_ = 0;
            RCLCPP_ERROR(node_.get_logger(),
                "EKF restart reply timed out; outcome unknown, not retrying");
        }
        return;
    }
    if (!allowed || queued_ == 0) return;
    --queued_;

    px4_msgs::msg::VehicleCommand command{};
    sequence_ = sequence_ == kMaxSequence ? 1 : sequence_ + 1;
    command.timestamp = node_.now().nanoseconds() / 1000;
    command.command = kEkfRestartCommand;
    command.param1 = kEkfRestartKey;
    command.param2 = static_cast<float>(sequence_);
    command.target_system = kSystem;
    command.target_component = 1;
    command.source_system = kSystem;
    command.source_component = kComponent;
    command.from_external = true;
    in_flight_ = true;
    sent_at_ = std::chrono::steady_clock::now();
    publisher_->publish(command); // Exactly one publish per eligible AUX5 edge, no retry.
    RCLCPP_WARN(node_.get_logger(), "AUX5 changed while disarmed: EKF restart requested (%u)", sequence_);
}

void EkfRestartClient::on_ack(const px4_msgs::msg::VehicleCommandAck &ack)
{
    if (!in_flight_ || ack.command != kEkfRestartCommand ||
        ack.target_system != kSystem || ack.target_component != kComponent) return;
    // Unpatched PX4 replies UNSUPPORTED without our sequence extension.
    if (ack.result != ack.VEHICLE_CMD_RESULT_UNSUPPORTED &&
        ack.result_param2 != static_cast<int32_t>(sequence_)) return;
    if (ack.result == ack.VEHICLE_CMD_RESULT_IN_PROGRESS) return;
    in_flight_ = false;
    if (ack.result == ack.VEHICLE_CMD_RESULT_ACCEPTED) {
        RCLCPP_INFO(node_.get_logger(), "PX4 EKF2 restarted (%u)", sequence_);
    } else {
        RCLCPP_ERROR(node_.get_logger(),
            "PX4 EKF restart rejected/failed (%u, result=%u); not retrying. Check firmware and arming state",
            sequence_, ack.result);
    }
}

} // namespace px4ctrl
