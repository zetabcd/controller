#ifndef PX4CTRL_OMMPC_H_
#define PX4CTRL_OMMPC_H_

#include <px4ctrl/controller.h>
#include <px4ctrl/ommpc_solver.h>

class PX4ControlNode;

class OmMpcControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OmMpcControl(PX4ControlNode & node, const OmMpcOptions & options = {});

  // ROS adapter: measured state -> solver -> total thrust [N] and body rates.
  px4debug_msgs::msg::Px4ctrlDebug calculateControl(
    const px4ctrl::ReferenceWindow & window,
    const px4ctrl::ControlModeReference & reference,
    const LocalPose_Data_t & pose,
    const Attitude_Data_t & attitude,
    const Sensor_Data_t & sensor,
    double now_seconds, double dt,
    Control_Setpoint_t & control_setpoint,
    const Parameter_t & parameters);

  void setOptions(const OmMpcOptions & options);
  const OmMpcOptions & options() const {return solver_->options();}
  const OmMpcDiagnostics & diagnostics() const {return solver_->diagnostics();}

  // 清空诊断并把故障回退输入重置为悬停 [g,0,0,0]。
  void resetControlParams();
  // 为兼容工程现有控制器接口保留；本控制器不使用油门辨识映射。
  void resetThrustMapping(const Parameter_t & parameters);
  // 本控制器不在线估计 thrust model，恒返回 false。
  bool estimateThrustModel(const Eigen::Vector3d & estimated_acceleration);
  void init_filters(const Parameter_t &) {}
  void reset_filters() {}

private:
  PX4ControlNode & node_;
  std::unique_ptr < OmMpcSolver > solver_;
};
#endif
