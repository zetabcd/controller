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
    // Reuse the existing input caches. Only a changed, nonzero source timestamp
    // counts as new data; reading the same cache again must not refresh liveness.
    void update(uint64_t aux5_changes, bool allowed,
                uint64_t status_timestamp, uint64_t attitude_timestamp);

private:
    using Clock = std::chrono::steady_clock;
    enum class Phase { Idle, WaitDisconnect, WaitReconnect, Verifying, Guard };
    struct Stream {
        uint64_t timestamp{0};
        uint64_t observations{0};
        Clock::time_point last_progress{};
        bool observe(uint64_t sample_timestamp, Clock::time_point now);
        bool fresh(Clock::time_point now) const;
    };
    void update_at(uint64_t aux5_changes, bool allowed,
                   uint64_t status_timestamp, uint64_t attitude_timestamp,
                   Clock::time_point now);
    void on_ack(const px4_msgs::msg::VehicleCommandAck &ack);
    rclcpp::Node &node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr subscription_;
    // Pending AUX5 changes are discarded, never queued/replayed. Guard also
    // preserves the minimum 5-second interval after rejection or fast recovery.
    Phase phase_{Phase::Idle};
    bool ack_pending_{false};
    bool reconnect_timeout_reported_{false};
    Stream status_{};
    Stream attitude_{};
    uint64_t status_at_disconnect_{0};
    uint64_t attitude_at_disconnect_{0};
    uint64_t status_at_verification_{0};
    uint64_t attitude_at_verification_{0};
    Clock::time_point sent_at_{};
    Clock::time_point disconnected_at_{};
    Clock::time_point stable_since_{};
};

} // namespace px4ctrl
