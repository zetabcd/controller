#pragma once

#include <chrono>
#include <cstdint>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4ctrl {

// Standard PX4 autopilot reboot command over DDS (MAV_CMD 246, param1 = 1).
class FcuRebootClient
{
public:
    explicit FcuRebootClient(rclcpp::Node &node);
    // Called from main(), before the arming/kill checks, including while disarmed.
    void update(uint64_t aux5_changes, bool allowed);

private:
    void on_ack(const px4_msgs::msg::VehicleCommandAck &ack);
    rclcpp::Node &node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr subscription_;
    // Keep the guard after an ACK: acceptance precedes the actual reboot.
    // AUX5 changes during this interval are discarded, never queued/replayed.
    bool request_active_{false};
    bool ack_pending_{false};
    std::chrono::steady_clock::time_point sent_at_{};
};

} // namespace px4ctrl
