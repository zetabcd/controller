#include "solver_nmpc_backend.h"
#include "solver_nmpc_casadi_bridge.h"

#include <chrono>
#include <memory>
#include <vector>

namespace
{
using ReferenceVector = std::vector<SolverNmpcReference,
  Eigen::aligned_allocator<SolverNmpcReference>>;

void appendState(const SolverNmpcState &state, std::vector<double> *values)
{
  // 桥接口序列化顺序固定为 PVQ=[p(3),v(3),q_wxyz(4)]，共 10 个 double。
  values->insert(values->end(), {
    state.position.x(), state.position.y(), state.position.z(),
    state.velocity.x(), state.velocity.y(), state.velocity.z(),
    state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z()});
}

class CasadiBackend final : public SolverNmpcBackendBase
{
public:
  explicit CasadiBackend(const SolverNmpcOptions &options) : options_(options)
  {
    // 隔离层只暴露 C ABI，使本文件不需要包含庞大的 CasADi C++ 模板头；
    // 真正的符号图、自动微分和 Ipopt NLP 在 solver_nmpc_casadi_bridge.cpp。
    const double maximum_rate[3]{
      options.body_rate_max.x(), options.body_rate_max.y(), options.body_rate_max.z()};
    char error[512]{};
    bridge_ = px4ctrl_casadi_nmpc_create(
      static_cast<std::size_t>(options.horizon), options.prediction_dt,
      options.gravity, options.thrust_acceleration_min,
      options.thrust_acceleration_max, maximum_rate,
      options.maximum_iterations, error, sizeof(error));
    construction_status_ = error;
  }

  ~CasadiBackend() override {px4ctrl_casadi_nmpc_destroy(bridge_);}

  SolverNmpcSolveResult solve(
    const SolverNmpcState &state, const ReferenceVector &references) override
  {
    SolverNmpcSolveResult output;
    if (bridge_ == nullptr) {
      output.status = "CasADi construction failed: " + construction_status_;
      return output;
    }
    std::vector<double> parameters;
    // [CASADI-PARAM-1] 外部参数布局：
    //   P_ext=[x_measured(10), r_0(14),...,r_N(14)]，
    //   r_k=[x_ref,k(10),u_ref,k(4)]。
    parameters.reserve(10 + 14 * references.size());
    appendState(state, &parameters);
    for (const auto &reference : references) {
      appendState(reference.state, &parameters);
      parameters.push_back(reference.thrust_acceleration);
      parameters.push_back(reference.body_rate.x());
      parameters.push_back(reference.body_rate.y());
      parameters.push_back(reference.body_rate.z());
    }
    double command[4]{};
    char status[256]{};
    const auto start = std::chrono::steady_clock::now();
    // 桥内求解单重射击 U^*，该 C 接口仅返回第一项 [a_T,omega]。
    output.success = px4ctrl_casadi_nmpc_solve(
      bridge_, parameters.data(), parameters.size(), command,
      &output.objective, &output.iterations, status, sizeof(status)) != 0;
    output.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    output.status = status;
    if (output.success) {
      output.command.thrust_acceleration = command[0];
      output.command.body_rate = Eigen::Vector3d(command[1], command[2], command[3]);
    }
    return output;
  }

  void reset() override {px4ctrl_casadi_nmpc_reset(bridge_);}

private:
  SolverNmpcOptions options_;
  Px4ctrlCasadiNmpcBridge *bridge_{nullptr};
  std::string construction_status_;
};
}  // namespace

std::unique_ptr<SolverNmpcBackendBase> createIpoptCasadiBackend(
  const SolverNmpcOptions &options)
{
  return std::make_unique<CasadiBackend>(options);
}
