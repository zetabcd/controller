#ifndef PX4CTRL_ACADOS_NMPC_H_
#define PX4CTRL_ACADOS_NMPC_H_

#include <px4ctrl/acados_nmpc_solver.h>
#include <px4ctrl/controller.h>

// Controller adapter: explicit reference window, manual mode, units and debug.
// Solver owns optimization and failure recovery; no node dependency.
class AcadosNmpcControl
{
public:
  void configure(const AcadosNmpcOptions & options, double mass);
  const AcadosNmpcOptions & options() const;
  const AcadosNmpcDiagnostics & diagnostics() const;
  void reset();
  px4debug_msgs::msg::Px4ctrlDebug calculate(
    const px4ctrl::ReferenceWindow & window, const px4ctrl::ControlModeReference & reference,
    const AcadosNmpcState & state,
    double now_seconds, double elapsed_seconds,
    Control_Setpoint_t & output);

private:
  void prepareReferences(const px4ctrl::ReferenceWindow & window, const Eigen::Quaterniond & attitude);
  std::unique_ptr < AcadosNmpcSolver > solver_;
  AcadosNmpcReferences references_;
  double mass_ {1.0};
  bool feedforward_valid_{false};
};

#endif
