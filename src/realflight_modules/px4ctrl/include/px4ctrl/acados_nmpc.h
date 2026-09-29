#ifndef PX4CTRL_ACADOS_NMPC_H_
#define PX4CTRL_ACADOS_NMPC_H_

#include <px4ctrl/acados_nmpc_solver.h>
#include <px4ctrl/controller.h>
#include <px4ctrl/omtraj.h>

// Thin FSM adapter: trajectory sampling, manual mode, unit conversion and debug.
// Solver owns optimization and failure recovery; no node dependency.
class AcadosNmpcControl
{
public:
  void configure(const AcadosNmpcOptions & options, double mass);
  const AcadosNmpcOptions & options() const;
  const AcadosNmpcDiagnostics & diagnostics() const;
  void reset();
  void setTrajectory(OmTrajectoryResult trajectory, const rclcpp::Time & start);
  void clearTrajectory();
  px4debug_msgs::msg::Px4ctrlDebug calculate(
    const Ref_State_t & reference, const LocalPose_Data_t & pose,
    const Attitude_Data_t & attitude, const Eigen::Vector3d & measured_body_rate,
    double now_seconds, double elapsed_seconds,
    Control_Setpoint_t & output);

private:
  void sampleReferences(const Ref_State_t & reference, double now_seconds);
  std::unique_ptr < AcadosNmpcSolver > solver_;
  AcadosNmpcReferences references_;
  OmTrajectoryResult trajectory_;
  double trajectory_start_ {0.0};
  double mass_ {1.0};
  bool trajectory_active_ {false};
};

#endif
