#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

// One motor: static throttle feedforward + integral speed correction (rad/s).
// The fast feedforward path is unchanged; only new ESC samples update the I term.
class MotorSpeedControl
{
public:
  void configure(double ki, double limit, double timeout)
  {
    if (!std::isfinite(ki) || ki < 0 || !std::isfinite(limit) || limit < 0 || limit > 1 ||
      !std::isfinite(timeout) || timeout <= 0)
    {
      throw std::invalid_argument("motor speed ki >= 0, integral_limit in [0,1], timeout > 0 required");
    }
    ki_ = ki;
    limit_ = limit;
    timeout_ = timeout;
    reset();
  }

  void reset()
  {
    integral_ = 0;
    last_stamp_ = 0;
    last_call_ = -1;
  }

  double update(double feedforward, double desired_rad, double measured_rad,
    uint64_t report_stamp, double now, bool valid, bool active)
  {
    if (!active || ki_ == 0 || !std::isfinite(feedforward) ||
      !std::isfinite(desired_rad) || desired_rad <= 0)
    {
      reset();
      return feedforward;
    }
    const double call_dt = last_call_ >= 0 ? std::clamp(now - last_call_, 0.0, timeout_) : 0;
    last_call_ = now;
    if (!valid || !report_stamp || !std::isfinite(measured_rad) || measured_rad < 0) {
      // After the freshness timeout, remove correction gradually over <= timeout_;
      // never integrate an outage on recovery, or substitute zero for missing RPM.
      last_stamp_ = 0;
      const double step = limit_ * call_dt / timeout_;
      integral_ -= std::clamp(integral_, -step, step);
    } else if (report_stamp != last_stamp_) {
      const double dt = report_stamp > last_stamp_ ? (report_stamp - last_stamp_) * 1e-6 : 0;
      if (last_stamp_ && dt > 0 && dt <= timeout_) {
        double delta = ki_ * (desired_rad - measured_rad) * dt;
        // Allow unwinding, but never accumulate further into actuator saturation.
        const double output = feedforward + integral_;
        delta = std::clamp(delta, std::min(0.0, -output), std::max(0.0, 1.0 - output));
        integral_ = std::clamp(integral_ + delta, -limit_, limit_);
      }
      // First sample, long gap or source-clock reset: prime timing without a jump.
      last_stamp_ = report_stamp;
    }
    return std::clamp(feedforward + integral_, 0.0, 1.0);
  }

private:
  double ki_{0};
  double limit_{0};
  double timeout_{0.25};
  double integral_{0};
  uint64_t last_stamp_{0};
  double last_call_{-1};
};
