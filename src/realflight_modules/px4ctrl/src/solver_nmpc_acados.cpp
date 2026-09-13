#include "solver_nmpc_backend.h"

#include "acados_solver_px4ctrl_nmpc.h"

#include <acados_c/ocp_nlp_interface.h>

#include <algorithm>
#include <array>
#include <memory>
#include <vector>

namespace
{
using ReferenceVector = std::vector<SolverNmpcReference,
  Eigen::aligned_allocator<SolverNmpcReference>>;

/*
 * acados 后端的离散多重射击问题（公式由生成脚本固化）：
 *
 *   min_{X,U} sum_{k=0}^{N-1} 0.5*(y_k-y_ref,k)^T W (y_k-y_ref,k)
 *             + 0.5*(x_N-x_ref,N)^T W_N (x_N-x_ref,N)
 *   s.t. x_0=x_measured,
 *        x_{k+1}=F_ERK(x_k,u_k),
 *        u_min<=u_k<=u_max,
 *
 *   y_k=[x_k,u_k], x=[p,v,q_wxyz], u=[a_T,omega]。
 *
 * 与三个单重射击后端的有意实现差异：当前生成 OCP 没有 Delta-u 项；姿态项是
 * 5||q-q_ref||^2，而不是 5(1-<q,q_ref>^2)。公共层的 [QUAT-SIGN]
 * 会先保证 q_ref 符号连续，避免 q/-q 二义性触发错误的大分量残差。
 */

std::array<double, PX4CTRL_NMPC_NX> stateArray(const SolverNmpcState &state)
{
  // acados 生成模型固定采用 x=[p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z]。
  // 此处集中转换，防止 Eigen 四元数内部 coeffs() 的 xyzw 顺序被误用。
  return {state.position.x(), state.position.y(), state.position.z(),
    state.velocity.x(), state.velocity.y(), state.velocity.z(),
    state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z()};
}

class AcadosBackend final : public SolverNmpcBackendBase
{
public:
  explicit AcadosBackend(const SolverNmpcOptions &options) : options_(options)
  {
    // acados 的数组尺寸和射击节点数被编译进生成代码；N 改变后必须重新运行
    // script/generate_px4ctrl_acados_nmpc.py，不能仅在运行时改变 options.horizon。
    if (options.horizon != PX4CTRL_NMPC_N) {
      construction_status_ = "generated acados backend requires horizon=" +
        std::to_string(PX4CTRL_NMPC_N);
      return;
    }
    capsule_ = px4ctrl_nmpc_acados_create_capsule();
    if (capsule_ == nullptr) {
      construction_status_ = "acados capsule allocation failed";
      return;
    }
    // [ACADOS-DISC-1] 每段积分时间均为 dt，总预测时域 T=N*dt。
    std::vector<double> time_steps(
      static_cast<std::size_t>(options.horizon), options.prediction_dt);
    const int status = px4ctrl_nmpc_acados_create_with_discretization(
      capsule_, options.horizon, time_steps.data());
    if (status != 0) {
      construction_status_ = "acados create status=" + std::to_string(status);
      return;
    }
    config_ = px4ctrl_nmpc_acados_get_nlp_config(capsule_);
    dims_ = px4ctrl_nmpc_acados_get_nlp_dims(capsule_);
    input_ = px4ctrl_nmpc_acados_get_nlp_in(capsule_);
    output_ = px4ctrl_nmpc_acados_get_nlp_out(capsule_);
    solver_ = px4ctrl_nmpc_acados_get_nlp_solver(capsule_);
    valid_ = true;
  }

  ~AcadosBackend() override
  {
    if (capsule_ != nullptr) {
      if (valid_) {px4ctrl_nmpc_acados_free(capsule_);}
      px4ctrl_nmpc_acados_free_capsule(capsule_);
    }
  }

  SolverNmpcSolveResult solve(
    const SolverNmpcState &state, const ReferenceVector &references) override
  {
    SolverNmpcSolveResult result;
    if (!valid_ || references.size() !=
      static_cast<std::size_t>(options_.horizon + 1))
    {
      result.status = construction_status_.empty() ?
        "acados reference size mismatch" : construction_status_;
      return result;
    }
    // [WARM-1] 首帧用参考轨迹初始化全部 X/U；后续帧左移上次多重射击解。
    if (has_solution_) {shiftSolution();} else {initializeGuess(references);}

    // [ACADOS-CONSTRAINT-1] 初始状态等式约束：x_0=x_measured。
    // acados 用同一组索引上的 lbx=ubx 表示等式，并同步设置迭代初值 x_0。
    auto x0 = stateArray(state);
    ocp_nlp_constraints_model_set(
      config_, dims_, input_, output_, 0, "lbx", x0.data());
    ocp_nlp_constraints_model_set(
      config_, dims_, input_, output_, 0, "ubx", x0.data());
    ocp_nlp_out_set(config_, dims_, output_, input_, 0, "x", x0.data());

    // [ACADOS-CONSTRAINT-2] 每个射击阶段的输入盒约束：
    //   a_T,min<=a_T,k<=a_T,max，-omega_max<=omega_k<=omega_max。
    std::array<double, PX4CTRL_NMPC_NU> lower{
      options_.thrust_acceleration_min,
      -options_.body_rate_max.x(), -options_.body_rate_max.y(),
      -options_.body_rate_max.z()};
    std::array<double, PX4CTRL_NMPC_NU> upper{
      options_.thrust_acceleration_max,
      options_.body_rate_max.x(), options_.body_rate_max.y(),
      options_.body_rate_max.z()};
    for (int stage = 0; stage < options_.horizon; ++stage) {
      ocp_nlp_constraints_model_set(
        config_, dims_, input_, output_, stage, "lbu", lower.data());
      ocp_nlp_constraints_model_set(
        config_, dims_, input_, output_, stage, "ubu", upper.data());
      // [ACADOS-COST-1] NONLINEAR_LS 阶段输出 y_k=[x_k,u_k]，
      // 此处写入 y_ref,k=[x_ref,k,u_ref,k]。W 在生成脚本中定义。
      std::array<double, PX4CTRL_NMPC_NY> reference{};
      fillStageReference(references[static_cast<std::size_t>(stage)], &reference);
      ocp_nlp_cost_model_set(
        config_, dims_, input_, stage, "yref", reference.data());
    }
    // [ACADOS-COST-2] 终端输出 y_N=x_N，仅包含状态参考，不再含输入。
    auto terminal = stateArray(references.back().state);
    ocp_nlp_cost_model_set(
      config_, dims_, input_, options_.horizon, "yref", terminal.data());

    // 生成求解器内部执行 SQP：线性化非线性动力学/输出，使用 Gauss-Newton
    // Hessian 和 HPIPM 解每次 QP；具体配置见生成脚本 [ACADOS-SOLVE-1]。
    const int status = px4ctrl_nmpc_acados_solve(capsule_);
    result.status = "acados status=" + std::to_string(status);
    double solve_seconds = 0.0;
    ocp_nlp_get(solver_, "time_tot", &solve_seconds);
    result.solve_time_ms = 1.0e3 * solve_seconds;
    ocp_nlp_get(solver_, "sqp_iter", &result.iterations);
    ocp_nlp_get(solver_, "cost_value", &result.objective);
    if (status != 0) {return result;}
    std::array<double, PX4CTRL_NMPC_NU> command{};
    // [RHC-1] 多重射击虽求得 X_0..X_N 和 U_0..U_(N-1)，仍只执行 u_0^*。
    ocp_nlp_out_get(config_, dims_, output_, 0, "u", command.data());
    result.command.thrust_acceleration = command[0];
    result.command.body_rate = Eigen::Vector3d(command[1], command[2], command[3]);
    result.success = true;
    has_solution_ = true;
    return result;
  }

  void reset() override
  {
    has_solution_ = false;
    if (valid_) {
      px4ctrl_nmpc_acados_reset(capsule_, 1, 1, 0, 0);
    }
  }

private:
  static void fillStageReference(
    const SolverNmpcReference &reference,
    std::array<double, PX4CTRL_NMPC_NY> *values)
  {
    // y_ref=[p_ref(3),v_ref(3),q_ref_wxyz(4),a_T_ref,omega_ref(3)]。
    const auto state = stateArray(reference.state);
    std::copy(state.begin(), state.end(), values->begin());
    (*values)[10] = reference.thrust_acceleration;
    (*values)[11] = reference.body_rate.x();
    (*values)[12] = reference.body_rate.y();
    (*values)[13] = reference.body_rate.z();
  }

  void initializeGuess(const ReferenceVector &references)
  {
    // [WARM-2] 多重射击变量包含每个节点的 x 和每段的 u；直接用参考序列
    // 初始化通常比全零更接近可行轨迹，尤其适合翻滚等大姿态动作。
    for (int stage = 0; stage < options_.horizon; ++stage) {
      auto x = stateArray(references[static_cast<std::size_t>(stage)].state);
      std::array<double, PX4CTRL_NMPC_NU> u{
        references[static_cast<std::size_t>(stage)].thrust_acceleration,
        references[static_cast<std::size_t>(stage)].body_rate.x(),
        references[static_cast<std::size_t>(stage)].body_rate.y(),
        references[static_cast<std::size_t>(stage)].body_rate.z()};
      ocp_nlp_out_set(config_, dims_, output_, input_, stage, "x", x.data());
      ocp_nlp_out_set(config_, dims_, output_, input_, stage, "u", u.data());
    }
    auto terminal = stateArray(references.back().state);
    ocp_nlp_out_set(
      config_, dims_, output_, input_, options_.horizon, "x", terminal.data());
  }

  void shiftSolution()
  {
    // [WARM-3] 滚动时域左移：x_guess,k<-x^*_(k+1)，
    // u_guess,k<-u^*_(min(k+1,N-1))。最后一项输入重复以补齐预测窗。
    std::array<double, PX4CTRL_NMPC_NX> x{};
    std::array<double, PX4CTRL_NMPC_NU> u{};
    for (int stage = 0; stage < options_.horizon; ++stage) {
      ocp_nlp_out_get(config_, dims_, output_, stage + 1, "x", x.data());
      ocp_nlp_out_set(config_, dims_, output_, input_, stage, "x", x.data());
      const int source = std::min(stage + 1, options_.horizon - 1);
      ocp_nlp_out_get(config_, dims_, output_, source, "u", u.data());
      ocp_nlp_out_set(config_, dims_, output_, input_, stage, "u", u.data());
    }
  }

  SolverNmpcOptions options_;
  px4ctrl_nmpc_solver_capsule *capsule_{nullptr};
  ocp_nlp_config *config_{nullptr};
  ocp_nlp_dims *dims_{nullptr};
  ocp_nlp_in *input_{nullptr};
  ocp_nlp_out *output_{nullptr};
  ocp_nlp_solver *solver_{nullptr};
  std::string construction_status_;
  bool valid_{false};
  bool has_solution_{false};
};
}  // namespace

std::unique_ptr<SolverNmpcBackendBase> createAcadosBackend(
  const SolverNmpcOptions &options)
{
  return std::make_unique<AcadosBackend>(options);
}
