#pragma once

#include <chrono>
#include <cstdint>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4ctrl {

// Project-private DDS command. Requires firmware/px4-v1.16-ekf-restart.patch.
constexpr uint32_t kEkfRestartCommand = 100010;
constexpr float kEkfRestartKey = 4541254.0F; // ASCII "EKF".

class EkfRestartClient
{
public:
    explicit EkfRestartClient(rclcpp::Node &node);
    // Called from main(), before the arming/kill checks, including while disarmed.
    void update(uint64_t aux5_changes, bool allowed);

private:
    void on_ack(const px4_msgs::msg::VehicleCommandAck &ack);
    rclcpp::Node &node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr subscription_;
    uint64_t queued_{0};
    uint32_t sequence_{0};
    bool in_flight_{false};
    std::chrono::steady_clock::time_point sent_at_{};
};

} // namespace px4ctrl
