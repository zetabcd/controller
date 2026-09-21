#include <px4ctrl/fcu_reboot.h>

#include <memory>

namespace px4ctrl {
namespace {
constexpr uint8_t kSystem = 1;
constexpr uint16_t kComponent = 191;
constexpr auto kReplyTimeout = std::chrono::seconds(5);
constexpr auto kDataTimeout = std::chrono::seconds(1);
constexpr auto kDisconnectTimeout = std::chrono::seconds(10);
constexpr auto kReconnectTimeout = std::chrono::seconds(60);
constexpr auto kStableDuration = std::chrono::seconds(2);
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

bool FcuRebootClient::Stream::observe(uint64_t sample_timestamp, Clock::time_point now)
{
    if (sample_timestamp == 0 || sample_timestamp == timestamp) return false;
    const bool went_backwards = timestamp != 0 && sample_timestamp < timestamp;
    timestamp = sample_timestamp;
    last_progress = now;
    ++observations;
    return went_backwards;
}

bool FcuRebootClient::Stream::fresh(Clock::time_point now) const
{
    return observations != 0 && now - last_progress < kDataTimeout;
}

void FcuRebootClient::update(uint64_t aux5_changes, bool allowed,
                            uint64_t status_timestamp, uint64_t attitude_timestamp)
{
    update_at(aux5_changes, allowed, status_timestamp, attitude_timestamp, Clock::now());
}

void FcuRebootClient::update_at(uint64_t aux5_changes, bool allowed,
                               uint64_t status_timestamp, uint64_t attitude_timestamp,
                               Clock::time_point now)
{
    // Inspect the gap BEFORE observing this iteration's samples. This also
    // catches a gap if the first resumed messages arrive between main-loop ticks.
    const bool status_was_fresh = status_.fresh(now);
    const bool attitude_was_fresh = attitude_.fresh(now);
    const auto previous_status_count = status_.observations;
    const auto previous_attitude_count = attitude_.observations;
    const bool status_went_backwards = status_.observe(status_timestamp, now);
    const bool attitude_went_backwards = attitude_.observe(attitude_timestamp, now);

    if (phase_ != Phase::Idle) {
        if (aux5_changes != 0) {
            RCLCPP_WARN(node_.get_logger(),
                "[FCU_REBOOT] AUX5 change ignored: previous reboot/guard still pending; not queued");
        }
        if (ack_pending_ && now - sent_at_ >= kReplyTimeout) {
            ack_pending_ = false;
            RCLCPP_WARN(node_.get_logger(),
                "[FCU_REBOOT][ACK_TIMEOUT] No ACK within 5 s; continue monitoring DDS data, not retrying");
        }
        if (phase_ == Phase::Guard) {
            if (now - sent_at_ >= kReplyTimeout) phase_ = Phase::Idle;
            return;
        }
        if (phase_ == Phase::WaitDisconnect) {
            if (!status_was_fresh && !attitude_was_fresh && now - sent_at_ >= kDataTimeout) {
                phase_ = Phase::WaitReconnect;
                disconnected_at_ = now;
                status_at_disconnect_ = previous_status_count;
                attitude_at_disconnect_ = previous_attitude_count;
                RCLCPP_WARN(node_.get_logger(),
                    "[FCU_REBOOT][DISCONNECTED] Status and attitude stopped updating; waiting for DDS recovery");
            } else if (now - sent_at_ >= kDisconnectTimeout) {
                phase_ = Phase::Idle;
                RCLCPP_ERROR(node_.get_logger(),
                    "[FCU_REBOOT][NOT_CONFIRMED] No joint data interruption observed within 10 s; "
                    "cannot confirm reboot/reconnection. No retry; a new AUX5 change is required");
            }
        }
        if (phase_ == Phase::WaitReconnect || phase_ == Phase::Verifying) {
            if (!reconnect_timeout_reported_ && now - disconnected_at_ >= kReconnectTimeout) {
                reconnect_timeout_reported_ = true;
                RCLCPP_ERROR(node_.get_logger(),
                    "[FCU_REBOOT][RECONNECT_TIMEOUT] DDS recovery not confirmed within 60 s "
                    "(status_fresh=%d, attitude_fresh=%d). Check PX4 Ethernet/uxrce_dds_client and Agent; "
                    "still watching for late recovery, not retrying reboot",
                    static_cast<int>(status_.fresh(now)), static_cast<int>(attitude_.fresh(now)));
            }
            if (phase_ == Phase::Verifying &&
                (!status_was_fresh || !attitude_was_fresh ||
                 status_went_backwards || attitude_went_backwards)) {
                phase_ = Phase::WaitReconnect;
                RCLCPP_WARN(node_.get_logger(),
                    "[FCU_REBOOT][UNSTABLE] Data gap or timestamp rollback; restarting DDS stability check");
            }
            const bool both_resumed = status_.observations > status_at_disconnect_ &&
                                      attitude_.observations > attitude_at_disconnect_;
            if (phase_ == Phase::WaitReconnect && both_resumed &&
                status_.fresh(now) && attitude_.fresh(now)) {
                phase_ = Phase::Verifying;
                stable_since_ = now;
                status_at_verification_ = status_.observations;
                attitude_at_verification_ = attitude_.observations;
                RCLCPP_INFO(node_.get_logger(),
                    "[FCU_REBOOT][VERIFYING] Status and attitude resumed; checking continuous updates for 2 s");
            }
            if (phase_ == Phase::Verifying && now - stable_since_ >= kStableDuration &&
                status_.observations - status_at_verification_ >= 2 &&
                attitude_.observations - attitude_at_verification_ >= 2) {
                phase_ = Phase::Guard;
                ack_pending_ = false;
                RCLCPP_INFO(node_.get_logger(),
                    "\033[32m[FCU_REBOOT][DDS_RECONNECTED] PX4 status + attitude data restored and stable. "
                    "DDS downlink confirmed; check RC, estimator and reference frame before arming.\033[0m");
            }
        }
        // Drop changes even on the iteration that ends monitoring/the guard.
        return;
    }
    if (aux5_changes == 0) return;
    if (!allowed) {
        RCLCPP_WARN(node_.get_logger(),
            "AUX5 PX4 reboot ignored: requires fresh RC/status and DISARMED");
        return;
    }
    if (status_.observations < 2 || attitude_.observations < 2 ||
        !status_.fresh(now) || !attitude_.fresh(now)) {
        RCLCPP_WARN(node_.get_logger(),
            "[FCU_REBOOT] AUX5 reboot ignored: need live status and attitude updates before monitoring a reboot");
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
    phase_ = Phase::WaitDisconnect;
    ack_pending_ = true;
    reconnect_timeout_reported_ = false;
    sent_at_ = now;
    // Multiple changes collected by one spin_some() request only one reboot.
    publisher_->publish(command);
    RCLCPP_WARN(node_.get_logger(),
        "[FCU_REBOOT][REQUESTED] AUX5: autopilot reboot sent (246, param1=1); waiting for data interruption");
}

void FcuRebootClient::on_ack(const px4_msgs::msg::VehicleCommandAck &ack)
{
    if (phase_ == Phase::Idle || !ack_pending_ ||
        ack.command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN ||
        ack.target_system != kSystem || ack.target_component != kComponent) return;
    // Standard ACKs do not echo a private request sequence in result_param2.
    if (ack.result == ack.VEHICLE_CMD_RESULT_IN_PROGRESS) return;
    ack_pending_ = false;
    if (ack.result == ack.VEHICLE_CMD_RESULT_ACCEPTED) {
        RCLCPP_INFO(node_.get_logger(),
            "[FCU_REBOOT][ACCEPTED] PX4 accepted reboot; ACK does not confirm DDS recovery");
    } else {
        phase_ = Phase::Guard;
        RCLCPP_ERROR(node_.get_logger(),
            "[FCU_REBOOT][REJECTED] PX4 reboot rejected/failed (result=%u); not retrying. "
            "Check arming state and board reboot support",
            ack.result);
    }
}

} // namespace px4ctrl
