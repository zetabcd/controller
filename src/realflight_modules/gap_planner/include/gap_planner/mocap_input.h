#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace gap_planner {
struct MocapPose {
  double stamp;
  Eigen::Vector3d center;
  Eigen::Quaterniond rotation;
};

// Raw rigid-body PoseStamped, as recorded on /sun1/pose. Do not apply the
// aircraft bridge's NWU -> NED / FLU -> FRD conversion to a gate pose.
class MocapInput {
  std::string frame_;
  bool frame_locked_;
  Eigen::Vector3d translation_;
  Eigen::Quaterniond rotation_;
  double timeout_;
public:
  MocapInput(std::string frame, const Eigen::Vector3d &translation,
    const Eigen::Quaterniond &rotation, double timeout)
    : frame_(std::move(frame)), frame_locked_(!frame_.empty()),
      translation_(translation), rotation_(rotation), timeout_(timeout)
  {
    if(!std::isfinite(timeout_) || timeout_<=0 || !translation_.allFinite() ||
      !rotation_.coeffs().allFinite() || !std::isfinite(rotation_.norm()) ||
      rotation_.norm()<1e-8) {throw std::invalid_argument("Invalid mocap transform/timeout");}
    rotation_.normalize();
  }
  bool frameLocked() const {return frame_locked_;}
  const std::string &frame() const {return frame_;}

  bool accept(const geometry_msgs::msg::PoseStamped &m, double now, double previous_stamp,
    const Eigen::Vector3d &opening_offset, const Eigen::Quaterniond &opening_rotation,
    MocapPose &out, std::string &reason)
  {
    const auto fail=[&](const std::string &why) {reason=why;return false;};
    if(m.header.stamp.sec<0 || m.header.stamp.nanosec>=1000000000u) {
      return fail("Invalid PoseStamped header.stamp");
    }
    const double stamp=m.header.stamp.sec+1e-9*m.header.stamp.nanosec;
    if(stamp<=0 || stamp<=previous_stamp) {return fail("Zero/duplicate/out-of-order PoseStamped stamp");}
    if(!std::isfinite(now) || now-stamp>timeout_ || now-stamp < -0.05) {
      return fail("Stale/future PoseStamped stamp; check mocap and ROS clock synchronization");
    }
    const auto &p=m.pose.position;const auto &r=m.pose.orientation;
    const Eigen::Vector3d position(p.x,p.y,p.z);
    Eigen::Quaterniond q(r.w,r.x,r.y,r.z);
    if(!position.allFinite() || !q.coeffs().allFinite() ||
      !std::isfinite(q.norm()) || q.norm()<1e-8) {return fail("Invalid rigid-body position/quaternion");}
    if(frame_locked_ && m.header.frame_id!=frame_) {
      return fail("Mocap frame mismatch: expected '"+frame_+"', got '"+m.header.frame_id+"'");
    }
    q.normalize();
    MocapPose candidate{stamp,translation_+rotation_*(position+q*opening_offset),
      (rotation_*q*opening_rotation).normalized()};
    if(!candidate.center.allFinite() || !candidate.rotation.coeffs().allFinite()) {
      return fail("Invalid transformed opening pose");
    }
    // Empty configuration learns the first VALID label, even if it is empty.
    // All gates share this parent frame; calibration is never auto-estimated.
    if(!frame_locked_) {frame_=m.header.frame_id;frame_locked_=true;}
    out=candidate;reason.clear();return true;
  }
};
}
