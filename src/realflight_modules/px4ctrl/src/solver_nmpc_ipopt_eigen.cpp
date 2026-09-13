#include "solver_nmpc_backend.h"

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <IpTNLP.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace
{
using ReferenceVector = std::vector<SolverNmpcReference,
  Eigen::aligned_allocator<SolverNmpcReference>>;

class IpoptEigenProblem final : public Ipopt::TNLP
{
public:
  // Ipopt+Eigen 采用单重射击：Eigen 数值模型由 controls 唯一确定整段状态，
  // 因而 TNLP 中只有 U，没有额外的 X 或动力学等式约束。
  IpoptEigenProblem(
    SolverNmpcState state, ReferenceVector references, SolverNmpcOptions options,
    SolverNmpcCommand previous, std::vector<double> initial_guess)
  : state_(std::move(state)), references_(std::move(references)), options_(options),
    previous_(previous), initial_guess_(std::move(initial_guess)),
    solution_(static_cast<std::size_t>(4 * options.horizon), 0.0)
  {
  }

  bool get_nlp_info(
    Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
    Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style) override
  {
    // [IPOPT-NLP-1] z=U=[u_0,...,u_{N-1}] in R^(4N)，m=0。
    // 输入上下界由 Ipopt 的变量 bounds 表达，因此不计入约束向量 g(z)。
    n = 4 * options_.horizon;
    m = 0;
    nnz_jac_g = 0;
    nnz_h_lag = 0;
    index_style = TNLP::C_STYLE;
    return true;
  }

  bool get_bounds_info(
    Ipopt::Index n, Ipopt::Number *lower, Ipopt::Number *upper,
    Ipopt::Index, Ipopt::Number *, Ipopt::Number *) override
  {
    // [CONSTRAINT-1] 对每个 k：
    //   a_T,min <= U_4k <= a_T,max，
    //   -omega_max,i <= U_(4k+1+i) <= omega_max,i。
    for (Ipopt::Index i = 0; i < n; i += 4) {
      lower[i] = options_.thrust_acceleration_min;
      upper[i] = options_.thrust_acceleration_max;
      for (int axis = 0; axis < 3; ++axis) {
        lower[i + 1 + axis] = -options_.body_rate_max(axis);
        upper[i + 1 + axis] = options_.body_rate_max(axis);
      }
    }
    return true;
  }

  bool get_starting_point(
    Ipopt::Index n, bool init_x, Ipopt::Number *x,
    bool init_z, Ipopt::Number *, Ipopt::Number *,
    Ipopt::Index, bool init_lambda, Ipopt::Number *) override
  {
    // [WARM-1] 初次求解使用悬停序列 [g,0,0,0]；之后使用上周期最优解左移。
    if (!init_x || init_z || init_lambda ||
      initial_guess_.size() != static_cast<std::size_t>(n))
    {
      return false;
    }
    std::copy(initial_guess_.begin(), initial_guess_.end(), x);
    return true;
  }

  bool eval_f(
    Ipopt::Index n, const Ipopt::Number *x, bool,
    Ipopt::Number &objective) override
  {
    // [IPOPT-NLP-2] f(U)=sum l_k+l_N，状态通过公共 [DYN-1..4] 前向滚动。
    if (n != 4 * options_.horizon) {return false;}
    objective = solverNmpcObjective(
      state_, references_, x, options_, previous_);
    return std::isfinite(objective);
  }

  bool eval_grad_f(
    Ipopt::Index n, const Ipopt::Number *x, bool,
    Ipopt::Number *gradient) override
  {
    // [FD-1] 二阶中心差分梯度：
    //   df/dU_i ~= [f(U+h_i*e_i)-f(U-h_i*e_i)]/(2*h_i)，
    //   h_i=1e-5*max(1,|U_i|)。
    // 相对步长兼顾推力与角速度量纲；此后端不使用自动微分。
    std::vector<double> work(x, x + n);
    for (Ipopt::Index i = 0; i < n; ++i) {
      const double original = work[static_cast<std::size_t>(i)];
      const double step = 1.0e-5 * std::max(1.0, std::abs(original));
      work[static_cast<std::size_t>(i)] = original + step;
      const double forward = solverNmpcObjective(
        state_, references_, work.data(), options_, previous_);
      work[static_cast<std::size_t>(i)] = original - step;
      const double backward = solverNmpcObjective(
        state_, references_, work.data(), options_, previous_);
      work[static_cast<std::size_t>(i)] = original;
      gradient[i] = (forward - backward) / (2.0 * step);
    }
    return true;
  }

  bool eval_g(
    Ipopt::Index, const Ipopt::Number *, bool,
    Ipopt::Index, Ipopt::Number *) override
  {
    // m=0：动力学已被单重射击消元，输入限制是变量 bounds。
    return true;
  }
  bool eval_jac_g(
    Ipopt::Index, const Ipopt::Number *, bool, Ipopt::Index,
    Ipopt::Index, Ipopt::Index *, Ipopt::Index *, Ipopt::Number *) override
  {return true;}
  bool eval_h(
    Ipopt::Index, const Ipopt::Number *, bool, Ipopt::Number,
    Ipopt::Index, const Ipopt::Number *, bool, Ipopt::Index,
    Ipopt::Index *, Ipopt::Index *, Ipopt::Number *) override
  {
    // 配置 hessian_approximation=limited-memory 后，Ipopt 使用 L-BFGS 近似
    // 拉格朗日函数 Hessian，不会请求这里填充精确二阶导。
    return true;
  }

  void finalize_solution(
    Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
    const Ipopt::Number *, const Ipopt::Number *, Ipopt::Index,
    const Ipopt::Number *, const Ipopt::Number *, Ipopt::Number objective,
    const Ipopt::IpoptData *, Ipopt::IpoptCalculatedQuantities *) override
  {
    solution_.assign(x, x + n);
    objective_ = objective;
    internal_status_ = status;
  }

  const std::vector<double> &solution() const {return solution_;}
  double objective() const {return objective_;}
  Ipopt::SolverReturn internalStatus() const {return internal_status_;}

private:
  SolverNmpcState state_;
  ReferenceVector references_;
  SolverNmpcOptions options_;
  SolverNmpcCommand previous_;
  std::vector<double> initial_guess_;
  std::vector<double> solution_;
  double objective_{std::numeric_limits<double>::quiet_NaN()};
  Ipopt::SolverReturn internal_status_{Ipopt::INTERNAL_ERROR};
};

class IpoptEigenBackend final : public SolverNmpcBackendBase
{
public:
  explicit IpoptEigenBackend(const SolverNmpcOptions &options)
  : options_(options), warm_start_(static_cast<std::size_t>(4 * options.horizon), 0.0)
  {
    reset();
    application_ = IpoptApplicationFactory();
    application_->Options()->SetIntegerValue("print_level", 0);
    application_->Options()->SetStringValue("sb", "yes");
    // [IPOPT-SOLVE-1] 梯度由 [FD-1] 给出，Hessian 由 limited-memory BFGS 近似。
    application_->Options()->SetStringValue("hessian_approximation", "limited-memory");
    application_->Options()->SetIntegerValue("max_iter", options.maximum_iterations);
    application_->Options()->SetNumericValue("tol", 1.0e-3);
    application_->Options()->SetNumericValue("acceptable_tol", 5.0e-2);
    application_->Options()->SetIntegerValue("acceptable_iter", 3);
    initialized_ = application_->Initialize() == Ipopt::Solve_Succeeded;
  }

  SolverNmpcSolveResult solve(
    const SolverNmpcState &state, const ReferenceVector &references) override
  {
    SolverNmpcSolveResult output;
    if (!initialized_) {
      output.status = "Ipopt initialization failed";
      return output;
    }
    const auto start = std::chrono::steady_clock::now();
    Ipopt::SmartPtr<IpoptEigenProblem> problem = new IpoptEigenProblem(
      state, references, options_, previous_command_, warm_start_);
    const Ipopt::ApplicationReturnStatus status = application_->OptimizeTNLP(problem);
    output.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    output.objective = problem->objective();
    output.iterations = application_->Statistics()->IterationCount();
    output.status = "Ipopt status=" + std::to_string(static_cast<int>(status)) +
      ", internal=" + std::to_string(static_cast<int>(problem->internalStatus()));
    output.success = status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level;
    const auto &solution = problem->solution();
    if (!output.success || solution.size() != warm_start_.size()) {
      return output;
    }
    // [RHC-1] 只输出最优序列第一项 u_0^*。
    output.command.thrust_acceleration = solution[0];
    output.command.body_rate = Eigen::Vector3d(solution[1], solution[2], solution[3]);
    previous_command_ = output.command;
    // [WARM-2] 时域左移：U_guess,k <- U^*_(k+1)，末项重复 U^*_(N-1)。
    for (int k = 0; k + 1 < options_.horizon; ++k) {
      std::copy_n(solution.begin() + 4 * (k + 1), 4, warm_start_.begin() + 4 * k);
    }
    std::copy_n(solution.end() - 4, 4, warm_start_.end() - 4);
    return output;
  }

  void reset() override
  {
    // [WARM-3] 悬停初值满足 a_T=g、omega=0，对水平姿态有零加速度。
    previous_command_.thrust_acceleration = options_.gravity;
    previous_command_.body_rate.setZero();
    for (int k = 0; k < options_.horizon; ++k) {
      warm_start_[static_cast<std::size_t>(4 * k)] = options_.gravity;
      for (int j = 1; j < 4; ++j) {
        warm_start_[static_cast<std::size_t>(4 * k + j)] = 0.0;
      }
    }
  }

private:
  SolverNmpcOptions options_;
  std::vector<double> warm_start_;
  SolverNmpcCommand previous_command_;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> application_;
  bool initialized_{false};
};
}  // namespace

std::unique_ptr<SolverNmpcBackendBase> createIpoptEigenBackend(
  const SolverNmpcOptions &options)
{
  return std::make_unique<IpoptEigenBackend>(options);
}
