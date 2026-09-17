#pragma once

#include <Eigen/Dense>
#include <px4_msgs/msg/esc_status.hpp>
#include <px4debug_msgs/msg/motor_feedback_debug.hpp>
#include <px4ctrl/motor_calculate.h>
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rate_diagnostics
{
constexpr double nan = std::numeric_limits<double>::quiet_NaN();

// No clock-domain subtraction: now/received are host steady seconds. Source age is
// computed separately, only between timestamps belonging to the same PX4 message.
struct InputStamp
{
  uint64_t timestamp{0};
  uint64_t sequence{0};
  double received{nan};
  bool valid{false};
  void update(uint64_t source, double now, bool is_valid = true)
  {
    timestamp = source;
    received = now;
    valid = is_valid;
    ++sequence;
  }
  double age(double now) const {return sequence ? now - received : nan;}
  bool fresh(double now, double timeout) const
  {
    return valid && sequence && age(now) >= 0 && age(now) <= timeout;
  }
};

class MotorFeedback
{
public:
  using Message = px4debug_msgs::msg::MotorFeedbackDebug;

  // Slot fallback is opt-in: an empty list means match unique actuator functions.
  void configure(double timeout, const std::vector<int64_t> & slots,
    const std::vector<int64_t> & functions)
  {
    if (!std::isfinite(timeout) || timeout <= 0 || functions.size() != 4 ||
      (!slots.empty() && slots.size() != 4))
    {
      throw std::invalid_argument("ESC timeout must be positive; mapping must contain four motors");
    }
    for (size_t i = 0; i < 4; ++i) {
      if (functions[i] < 101 || functions[i] > 112 ||
        std::count(functions.begin(), functions.end(), functions[i]) != 1)
      {
        throw std::invalid_argument("ESC actuator functions must be distinct motor functions 101..112");
      }
      if (!slots.empty() && (slots[i] < 0 || slots[i] > 7 ||
        std::count(slots.begin(), slots.end(), slots[i]) != 1))
      {
        throw std::invalid_argument("ESC slots must be distinct indices in 0..7");
      }
      functions_[i] = functions[i];
      slots_[i] = slots.empty() ? -1 : slots[i];
    }
    timeout_ = timeout;
    explicit_slots_ = !slots.empty();
  }

  void update(const px4_msgs::msg::EscStatus & msg, double now)
  {
    // A reboot/source-clock reset invalidates every cached freshness marker.
    if (sequence_ && msg.timestamp < latest_.timestamp) {
      report_changed_at_.fill(nan);
      latest_ = px4_msgs::msg::EscStatus{};
    }
    for (size_t i = 0; i < 8; ++i) {
      if (msg.esc[i].timestamp && (msg.esc[i].timestamp != latest_.esc[i].timestamp ||
        msg.esc[i].actuator_function != latest_.esc[i].actuator_function))
      {
        report_changed_at_[i] = now;
      }
    }
    latest_ = msg;
    received_at_ = now;
    ++sequence_;
  }

  Message sample(double now, const Parameter_t & p, const Eigen::Vector3d & va_b,
    bool airflow_valid) const
  {
    Message out;
    out.esc_sequence = sequence_;
    out.esc_timestamp = latest_.timestamp;
    out.esc_receive_age_s = sequence_ ? now - received_at_ : nan;
    out.counter = latest_.counter;
    out.esc_count = latest_.esc_count;
    out.esc_connectiontype = latest_.esc_connectiontype;
    out.esc_online_flags = latest_.esc_online_flags;
    out.esc_armed_flags = latest_.esc_armed_flags;
    out.mapping_mode = explicit_slots_ ? 1 : 0;
    out.expected_actuator_function = functions_;
    out.feedback_timeout_s = timeout_;
    out.estimation_method = 1;
    out.airflow_valid = airflow_valid && va_b.allFinite();
    out.air_density = p.aero.rho;
    out.propeller_radius = p.uav.rp;
    out.arm_length = p.uav.l;
    out.arm_angle_rad = deg2rad(p.uav.beta_deg);
    out.ct_coefficients = {p.motor.Ct_a, p.motor.Ct_b, p.motor.Ct_c};
    out.cq_coefficients = {p.motor.Cq_a, p.motor.Cq_b, p.motor.Cq_c};
    for (size_t i = 0; i < 3; ++i) out.va_b[i] = va_b[i];
    out.motor_rpm.fill(nan);
    out.motor_rad.fill(nan);
    out.motor_voltage.fill(nan);
    out.motor_current.fill(nan);
    out.electrical_power.fill(nan);
    out.rpm_error.fill(nan);
    out.desired_motor_rpm.fill(nan);
    out.cts.fill(nan);
    out.cms.fill(nan);
    out.motor_thrust_est.fill(nan);
    out.motor_reaction_torque_est.fill(nan);
    out.mechanical_power_est.fill(nan);
    out.thrust_est = nan;
    out.tau_est.fill(nan);
    for (size_t i = 0; i < 8; ++i) {
      const auto & report = latest_.esc[i];
      out.report_timestamp[i] = report.timestamp;
      out.esc_rpm[i] = report.esc_rpm;
      out.esc_voltage[i] = report.esc_voltage;
      out.esc_current[i] = report.esc_current;
      out.esc_temperature[i] = report.esc_temperature;
      out.esc_errorcount[i] = report.esc_errorcount;
      out.esc_address[i] = report.esc_address;
      out.esc_cmdcount[i] = report.esc_cmdcount;
      out.esc_state[i] = report.esc_state;
      out.actuator_function[i] = report.actuator_function;
      out.failures[i] = report.failures;
      out.esc_power[i] = report.esc_power;
      out.report_receive_age_s[i] = now - report_changed_at_[i];
      out.report_source_age_s[i] = report.timestamp && latest_.timestamp >= report.timestamp ?
        (latest_.timestamp - report.timestamp) * 1e-6 : nan;
    }
    for (size_t motor = 0; motor < 4; ++motor) {
      int slot = slots_[motor];
      if (!explicit_slots_) {
        for (size_t i = 0; i < 8; ++i) {
          if (latest_.esc[i].actuator_function == functions_[motor]) {
            if (slot != -1) {slot = -2; break;}
            slot = i;
          }
        }
      }
      out.esc_slot[motor] = slot;
      if (slot < 0) continue;
      const auto & r = latest_.esc[slot];
      out.online[motor] = latest_.esc_online_flags & (1u << slot);
      out.armed[motor] = latest_.esc_armed_flags & (1u << slot);
      const bool fresh = sequence_ && out.online[motor] &&
        out.esc_receive_age_s >= 0 && out.esc_receive_age_s <= timeout_ &&
        out.report_receive_age_s[slot] >= 0 && out.report_receive_age_s[slot] <= timeout_ &&
        out.report_source_age_s[slot] >= 0 && out.report_source_age_s[slot] <= timeout_;
      out.motor_rpm[motor] = r.esc_rpm;
      out.motor_rad[motor] = r.esc_rpm * (2.0 * pi / 60.0);
      out.motor_voltage[motor] = r.esc_voltage;
      out.motor_current[motor] = r.esc_current;
      out.rpm_valid[motor] = fresh && r.esc_rpm >= 0;
      out.electrical_valid[motor] = fresh && std::isfinite(r.esc_voltage) &&
        r.esc_voltage > 0 && std::isfinite(r.esc_current) && r.esc_current >= 0;
      if (out.electrical_valid[motor]) {
        out.electrical_power[motor] = double(r.esc_voltage) * r.esc_current;
      }
      if (!out.rpm_valid[motor] || !out.airflow_valid) continue;
      const auto omega = Eigen::Array4d::Constant(out.motor_rad[motor]).eval();
      const double ct = get_cts_from_speed(omega, va_b.z(), p)[0];
      const double cm = get_cms_from_speed(omega, va_b.z(), p)[0];
      const double thrust = ct * omega[0] * omega[0];
      const double torque = cm * omega[0] * omega[0];
      const double power = torque * omega[0];
      if (!std::isfinite(ct) || ct <= 0 || !std::isfinite(cm) || cm <= 0 ||
        !std::isfinite(thrust) || !std::isfinite(torque) || !std::isfinite(power)) continue;
      out.estimate_valid[motor] = true;
      out.cts[motor] = ct;
      out.cms[motor] = cm;
      out.motor_thrust_est[motor] = thrust;
      out.motor_reaction_torque_est[motor] = torque;
      out.mechanical_power_est[motor] = power;
    }
    out.total_estimate_valid = std::all_of(
      out.estimate_valid.begin(), out.estimate_valid.end(), [](bool v) {return v;}) &&
      std::isfinite(p.uav.l) && p.uav.l > 0 && std::isfinite(p.uav.beta_deg);
    if (out.total_estimate_valid) {
      const auto & f = out.motor_thrust_est;
      const auto & q = out.motor_reaction_torque_est;
      out.thrust_est = f[0] + f[1] + f[2] + f[3];
      out.tau_est = {
        p.uav.l * std::sin(out.arm_angle_rad) * (f[0] - f[1] - f[2] + f[3]),
        p.uav.l * std::cos(out.arm_angle_rad) * (-f[0] - f[1] + f[2] + f[3]),
        q[0] - q[1] + q[2] - q[3]};
    }
    return out;
  }

private:
  px4_msgs::msg::EscStatus latest_;
  uint64_t sequence_{0};
  double received_at_{nan};
  std::array<double, 8> report_changed_at_{{nan, nan, nan, nan, nan, nan, nan, nan}};
  std::array<uint16_t, 4> functions_{{101, 102, 103, 104}};
  std::array<int8_t, 4> slots_{{-1, -1, -1, -1}};
  double timeout_{0.25};
  bool explicit_slots_{false};
};
}  // namespace rate_diagnostics
