#ifndef PX4CTRL_SOLVER_NMPC_BACKEND_H_
#define PX4CTRL_SOLVER_NMPC_BACKEND_H_

#include <px4ctrl/solver_nmpc.h>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

// 公共数学符号定义：
//   x=[p,v,q] in R^3 x R^3 x S^3，q 按 wxyz 排列；
//   u=[a_T,omega] in R x R^3。
// Eigen::Quaterniond 负责保持 q 属于单位四元数流形 S^3。
struct SolverNmpcState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
};

struct SolverNmpcReference
{
  // x_ref,k：位置、速度、姿态参考。
  SolverNmpcState state;
  // u_ref,k=[a_T,ref, omega_ref]：由轨迹加速度和姿态变化率得到。
  double thrust_acceleration{9.805};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
};

struct SolverNmpcCommand
{
  double thrust_acceleration{9.805};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
};

struct SolverNmpcSolveResult
{
  bool success{false};
  std::string status;
  int iterations{0};
  double solve_time_ms{0.0};
  double objective{0.0};
  SolverNmpcCommand command;
};

class SolverNmpcBackendBase
{
public:
  virtual ~SolverNmpcBackendBase() = default;
  // 求解有限时域问题并返回 u_0^*。references 必须包含 N+1 个状态参考；
  // 前 N 个元素同时携带阶段输入参考，最后一个只用于终端状态代价。
  virtual SolverNmpcSolveResult solve(
    const SolverNmpcState &state,
    const std::vector<SolverNmpcReference,
      Eigen::aligned_allocator<SolverNmpcReference>> &references) = 0;
  virtual void reset() = 0;
};

// 离散动力学公式 [DYN-1]--[DYN-4]，详见 solver_nmpc_common.cpp。
SolverNmpcState propagateSolverNmpcState(
  const SolverNmpcState &state, const SolverNmpcCommand &control,
  double dt, double gravity);

// 有限时域目标公式 [COST-1]--[COST-4]，由 Eigen 型后端共同调用。
double solverNmpcObjective(
  const SolverNmpcState &initial_state,
  const std::vector<SolverNmpcReference,
    Eigen::aligned_allocator<SolverNmpcReference>> &references,
  const double *controls, const SolverNmpcOptions &options,
  const SolverNmpcCommand &previous_command);

std::unique_ptr<SolverNmpcBackendBase> createIpoptEigenBackend(
  const SolverNmpcOptions &options);
std::unique_ptr<SolverNmpcBackendBase> createAcadosBackend(
  const SolverNmpcOptions &options);
std::unique_ptr<SolverNmpcBackendBase> createNloptEigenBackend(
  const SolverNmpcOptions &options);
std::unique_ptr<SolverNmpcBackendBase> createIpoptCasadiBackend(
  const SolverNmpcOptions &options);

#endif  // PX4CTRL_SOLVER_NMPC_BACKEND_H_
