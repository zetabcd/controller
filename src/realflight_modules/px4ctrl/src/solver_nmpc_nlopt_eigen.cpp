#include "solver_nmpc_backend.h"

#include <nlopt.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <vector>

namespace
{
using ReferenceVector = std::vector<SolverNmpcReference,
  Eigen::aligned_allocator<SolverNmpcReference>>;

struct NloptObjectiveContext
{
  // NLopt 的 C++ 回调没有捕获上下文，故在此集中保存公共目标函数所需数据。
  SolverNmpcState state;
  const ReferenceVector *references{nullptr};
  const SolverNmpcOptions *options{nullptr};
  SolverNmpcCommand previous;

  double evaluate(const std::vector<double> &controls) const
  {
    return solverNmpcObjective(
      state, *references, controls.data(), *options, previous);
  }

  static double callback(
    const std::vector<double> &controls, std::vector<double> &gradient,
    void *user_data)
  {
    auto &context = *static_cast<NloptObjectiveContext *>(user_data);
    if (!gradient.empty()) {
      // [FD-1] 与 Ipopt+Eigen 相同的二阶中心差分：
      //   df/dU_i ~= [f(U+h_i e_i)-f(U-h_i e_i)]/(2h_i)，
      //   h_i=1e-5*max(1,|U_i|)。
      std::vector<double> work = controls;
      for (std::size_t i = 0; i < controls.size(); ++i) {
        const double original = controls[i];
        const double step = 1.0e-5 * std::max(1.0, std::abs(original));
        work[i] = original + step;
        const double forward = context.evaluate(work);
        work[i] = original - step;
        const double backward = context.evaluate(work);
        work[i] = original;
        gradient[i] = (forward - backward) / (2.0 * step);
      }
    }
    return context.evaluate(controls);
  }
};

class NloptEigenBackend final : public SolverNmpcBackendBase
{
public:
  explicit NloptEigenBackend(const SolverNmpcOptions &options)
  : options_(options), warm_start_(static_cast<std::size_t>(4 * options.horizon), 0.0)
  {
    reset();
  }

  SolverNmpcSolveResult solve(
    const SolverNmpcState &state, const ReferenceVector &references) override
  {
    SolverNmpcSolveResult output;
    const std::size_t count = warm_start_.size();
    std::vector<double> lower(count);
    std::vector<double> upper(count);
    // [CONSTRAINT-1] U=[a_T,omega_x,omega_y,omega_z] 的逐阶段盒约束。
    for (int k = 0; k < options_.horizon; ++k) {
      const std::size_t offset = static_cast<std::size_t>(4 * k);
      lower[offset] = options_.thrust_acceleration_min;
      upper[offset] = options_.thrust_acceleration_max;
      for (int axis = 0; axis < 3; ++axis) {
        lower[offset + 1 + axis] = -options_.body_rate_max(axis);
        upper[offset + 1 + axis] = options_.body_rate_max(axis);
      }
    }
    NloptObjectiveContext context{state, &references, &options_, previous_command_};
    try {
      // [NLOPT-SOLVE-1] LD_LBFGS 是有界、梯度型拟牛顿法；状态由公共动力学
      // 单重射击消元，因此优化变量维数为 4N，没有显式动力学等式约束。
      nlopt::opt optimizer(nlopt::LD_LBFGS, count);
      optimizer.set_lower_bounds(lower);
      optimizer.set_upper_bounds(upper);
      optimizer.set_min_objective(&NloptObjectiveContext::callback, &context);
      optimizer.set_ftol_rel(1.0e-6);
      optimizer.set_xtol_rel(1.0e-4);
      optimizer.set_maxeval(options_.maximum_iterations);
      const auto start = std::chrono::steady_clock::now();
      const nlopt::result status = optimizer.optimize(warm_start_, output.objective);
      output.solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      output.iterations = optimizer.get_numevals();
      output.status = "NLopt status=" + std::to_string(static_cast<int>(status));
      output.success = static_cast<int>(status) > 0;
    } catch (const std::exception &error) {
      output.status = std::string("NLopt exception: ") + error.what();
      return output;
    }
    if (!output.success) {return output;}
    // [RHC-1] NLopt 会原地把 warm_start_ 改写成 U^*；只执行第一项 u_0^*。
    output.command.thrust_acceleration = warm_start_[0];
    output.command.body_rate = Eigen::Vector3d(
      warm_start_[1], warm_start_[2], warm_start_[3]);
    previous_command_ = output.command;
    // [WARM-2] 保存后左移最优序列，作为下一个控制周期的初值。
    const std::vector<double> solution = warm_start_;
    for (int k = 0; k + 1 < options_.horizon; ++k) {
      std::copy_n(solution.begin() + 4 * (k + 1), 4, warm_start_.begin() + 4 * k);
    }
    std::copy_n(solution.end() - 4, 4, warm_start_.end() - 4);
    return output;
  }

  void reset() override
  {
    // [WARM-3] 初值为 N 个悬停输入 [g,0,0,0]。
    previous_command_.thrust_acceleration = options_.gravity;
    previous_command_.body_rate.setZero();
    for (int k = 0; k < options_.horizon; ++k) {
      const std::size_t offset = static_cast<std::size_t>(4 * k);
      warm_start_[offset] = options_.gravity;
      std::fill_n(warm_start_.begin() + offset + 1, 3, 0.0);
    }
  }

private:
  SolverNmpcOptions options_;
  std::vector<double> warm_start_;
  SolverNmpcCommand previous_command_;
};
}  // namespace

std::unique_ptr<SolverNmpcBackendBase> createNloptEigenBackend(
  const SolverNmpcOptions &options)
{
  return std::make_unique<NloptEigenBackend>(options);
}
