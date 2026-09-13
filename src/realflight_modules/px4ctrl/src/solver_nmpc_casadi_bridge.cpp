#include "solver_nmpc_casadi_bridge.h"

#include <casadi/casadi.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

namespace
{
void copyText(const std::string &text, char *output, size_t capacity)
{
  if (output != nullptr && capacity > 0) {
    std::snprintf(output, capacity, "%s", text.c_str());
  }
}
}  // namespace

struct Px4ctrlCasadiNmpcBridge
{
  // CasADi 负责建立符号计算图并自动微分，Ipopt 负责有界 NLP 数值迭代。
  // 该类只存在于桥编译单元，外部通过 solver_nmpc_casadi_bridge.h 的 C ABI 使用。
  Px4ctrlCasadiNmpcBridge(
    size_t horizon_in, double dt_in, double gravity_in,
    double minimum_thrust, double maximum_thrust,
    const double maximum_rate[3], int maximum_iterations)
  : horizon(horizon_in), dt(dt_in), gravity(gravity_in),
    // 单重射击只优化 N 个四维输入。外部参数不含 u_previous；桥在内部保存它，
    // 因此符号参数比外部参数多 4 维：P=[x0,u_-1,r_0,...,r_N]。
    variable_count(4 * horizon), external_parameter_count(10 + 14 * (horizon + 1)),
    symbolic_parameter_count(14 + 14 * (horizon + 1)),
    warm_start(variable_count, 0.0), lower(variable_count, 0.0),
    upper(variable_count, 0.0), previous{gravity, 0.0, 0.0, 0.0}
  {
    for (size_t k = 0; k < horizon; ++k) {
      const size_t offset = 4 * k;
      warm_start[offset] = gravity;
      lower[offset] = minimum_thrust;
      upper[offset] = maximum_thrust;
      for (size_t axis = 0; axis < 3; ++axis) {
        lower[offset + 1 + axis] = -maximum_rate[axis];
        upper[offset + 1 + axis] = maximum_rate[axis];
      }
    }
    buildSolver(maximum_iterations);
  }

  casadi::MX propagate(const casadi::MX &state, const casadi::MX &control) const
  {
    // 本函数是公共 Eigen 模型 [DYN-1..4] 的 CasADi 符号等价实现：
    //   x=[p(3),v(3),q_wxyz(4)]，u=[a_T,omega_x,omega_y,omega_z]。
    const casadi::MX p = state(casadi::Slice(0, 3));
    const casadi::MX v = state(casadi::Slice(3, 6));
    const casadi::MX qw = state(6);
    const casadi::MX qx = state(7);
    const casadi::MX qy = state(8);
    const casadi::MX qz = state(9);
    const casadi::MX wx = control(1);
    const casadi::MX wy = control(2);
    const casadi::MX wz = control(3);
    // [DYN-1] R(q)e3（机体 z 轴在世界系中的方向）：
    //   b3=[2(qx*qz+qw*qy), 2(qy*qz-qw*qx), 1-2(qx^2+qy^2)]^T。
    const casadi::MX b3 = casadi::MX::vertcat({
      2.0 * (qx * qz + qw * qy),
      2.0 * (qy * qz - qw * qx),
      1.0 - 2.0 * (qx * qx + qy * qy)});
    // [DYN-2] a=R(q)e3*a_T-g*e3；随后采用常加速度离散 p/v。
    const casadi::MX acceleration = control(0) * b3 +
      casadi::MX::vertcat({0.0, 0.0, -gravity});
    const casadi::MX next_p = p + dt * v + 0.5 * dt * dt * acceleration;
    const casadi::MX next_v = v + dt * acceleration;
    // [DYN-3] q_dot=1/2*q tensor [0,omega]。这里先写 Hamilton 积的四个分量，
    // 下一行以 0.5*dt 一次性乘入 1/2 和 Euler 步长。
    const casadi::MX q_dot = casadi::MX::vertcat({
      -qx * wx - qy * wy - qz * wz,
      qw * wx + qy * wz - qz * wy,
      qw * wy + qz * wx - qx * wz,
      qw * wz + qx * wy - qy * wx});
    const casadi::MX q_raw = state(casadi::Slice(6, 10)) + 0.5 * dt * q_dot;
    // [DYN-4] q_(k+1)=q_raw/||q_raw||；1e-12 防止符号求值出现除零。
    const casadi::MX next_q = q_raw /
      sqrt(casadi::MX::dot(q_raw, q_raw) + 1.0e-12);
    return casadi::MX::vertcat({next_p, next_v, next_q});
  }

  void buildSolver(int maximum_iterations)
  {
    // [CASADI-NLP-1] 单重射击 NLP：
    //   minimize_U J(x0,U,R,u_-1),  U in R^(4N)，
    //   subject to u_min<=u_k<=u_max。
    // x_1..x_N 由 propagate 符号展开，不作为独立决策变量。
    const casadi::MX controls = casadi::MX::sym(
      "U", static_cast<casadi_int>(variable_count));
    const casadi::MX parameters = casadi::MX::sym(
      "P", static_cast<casadi_int>(symbolic_parameter_count));
    casadi::MX predicted = parameters(casadi::Slice(0, 10));
    casadi::MX previous_input = parameters(casadi::Slice(10, 14));
    casadi::MX cost = 0.0;
    for (size_t k = 0; k < horizon; ++k) {
      const casadi_int u_offset = static_cast<casadi_int>(4 * k);
      const casadi::MX control = controls(casadi::Slice(u_offset, u_offset + 4));
      // x_(k+1)=f_d(x_k,u_k)，因此状态误差对应 r_(k+1)，输入误差对应 r_k。
      predicted = propagate(predicted, control);
      const casadi_int state_ref_offset = static_cast<casadi_int>(
        14 + 14 * (k + 1));
      const casadi_int input_ref_offset = static_cast<casadi_int>(14 + 14 * k + 10);
      const casadi::MX ep = predicted(casadi::Slice(0, 3)) -
        parameters(casadi::Slice(state_ref_offset, state_ref_offset + 3));
      const casadi::MX ev = predicted(casadi::Slice(3, 6)) -
        parameters(casadi::Slice(state_ref_offset + 3, state_ref_offset + 6));
      const casadi::MX q_ref = parameters(
        casadi::Slice(state_ref_offset + 6, state_ref_offset + 10));
      const casadi::MX q_dot = casadi::MX::dot(
        predicted(casadi::Slice(6, 10)), q_ref);
      const casadi::MX input_ref = parameters(
        casadi::Slice(input_ref_offset, input_ref_offset + 4));
      const casadi::MX input_error = control - input_ref;
      const casadi::MX input_delta = control - previous_input;
      // 阶段目标 [COST-2..4]：
      //   18||e_p||^2+3||e_v||^2+5(1-<q,q_r>^2)
      //   +0.10 e_aT^2+0.12||e_omega||^2
      //   +0.02 Delta aT^2+0.12||Delta omega||^2。
      // q 点积平方保证 q 与 -q 的代价相同。
      cost += 18.0 * casadi::MX::dot(ep, ep);
      cost += 3.0 * casadi::MX::dot(ev, ev);
      cost += 5.0 * (1.0 - q_dot * q_dot);
      cost += 0.10 * input_error(0) * input_error(0);
      cost += 0.12 * casadi::MX::dot(
        input_error(casadi::Slice(1, 4)), input_error(casadi::Slice(1, 4)));
      cost += 0.02 * input_delta(0) * input_delta(0);
      cost += 0.12 * casadi::MX::dot(
        input_delta(casadi::Slice(1, 4)), input_delta(casadi::Slice(1, 4)));
      previous_input = control;
    }
    const casadi_int terminal_offset = static_cast<casadi_int>(14 + 14 * horizon);
    const casadi::MX terminal_ep = predicted(casadi::Slice(0, 3)) -
      parameters(casadi::Slice(terminal_offset, terminal_offset + 3));
    const casadi::MX terminal_ev = predicted(casadi::Slice(3, 6)) -
      parameters(casadi::Slice(terminal_offset + 3, terminal_offset + 6));
    const casadi::MX terminal_q = parameters(
      casadi::Slice(terminal_offset + 6, terminal_offset + 10));
    const casadi::MX terminal_dot = casadi::MX::dot(
      predicted(casadi::Slice(6, 10)), terminal_q);
    // 终端目标 [COST-5]：35||e_p,N||^2+6||e_v,N||^2+10(1-<q_N,q_r,N>^2)。
    cost += 35.0 * casadi::MX::dot(terminal_ep, terminal_ep);
    cost += 6.0 * casadi::MX::dot(terminal_ev, terminal_ev);
    cost += 10.0 * (1.0 - terminal_dot * terminal_dot);

    casadi::Dict options;
    options["print_time"] = false;
    options["expand"] = true;
    options["ipopt.print_level"] = 0;
    options["ipopt.sb"] = "yes";
    options["ipopt.max_iter"] = maximum_iterations;
    options["ipopt.tol"] = 1.0e-5;
    options["ipopt.acceptable_tol"] = 1.0e-3;
    options["ipopt.acceptable_iter"] = 3;
    // CasADi 对上述符号图做精确一/二阶自动微分，Ipopt 不再需要有限差分；
    // 这里没有 g 字段，因为动力学已消元，输入盒约束在 solve() 用 lbx/ubx 传入。
    const casadi::MXDict nlp{{"x", controls}, {"p", parameters}, {"f", cost}};
    solver = casadi::nlpsol("px4ctrl_casadi_nmpc", "ipopt", nlp, options);
  }

  void reset()
  {
    // [WARM-1] 清除历史输入与热启动，恢复水平悬停序列 [g,0,0,0]。
    previous = {gravity, 0.0, 0.0, 0.0};
    for (size_t k = 0; k < horizon; ++k) {
      warm_start[4 * k] = gravity;
      std::fill_n(warm_start.begin() + 4 * k + 1, 3, 0.0);
    }
  }

  bool solve(
    const double *external_parameters, size_t supplied_count,
    double command[4], double *objective, int *iterations, std::string &status)
  {
    if (external_parameters == nullptr || command == nullptr || objective == nullptr ||
      supplied_count != external_parameter_count)
    {
      status = "CasADi parameter count mismatch";
      return false;
    }
    // [CASADI-PARAM-2] 把桥内部保存的 u_-1 插入外部参数：
    //   P=[x_measured(10),u_-1(4),r_0(14),...,r_N(14)]。
    std::vector<double> parameters;
    parameters.reserve(symbolic_parameter_count);
    parameters.insert(parameters.end(), external_parameters, external_parameters + 10);
    parameters.insert(parameters.end(), previous.begin(), previous.end());
    parameters.insert(
      parameters.end(), external_parameters + 10,
      external_parameters + supplied_count);
    casadi::DMDict arguments;
    // [CONSTRAINT-1] lbx/ubx 是每段输入盒约束；x0 仅表示优化初值，并非状态 x_0。
    arguments["x0"] = casadi::DM(warm_start);
    arguments["p"] = casadi::DM(parameters);
    arguments["lbx"] = casadi::DM(lower);
    arguments["ubx"] = casadi::DM(upper);
    const casadi::DMDict result = solver(arguments);
    const std::vector<double> solution = result.at("x").get_elements();
    *objective = result.at("f").scalar();
    const casadi::Dict statistics = solver.stats();
    status = statistics.at("return_status").to_string();
    if (iterations != nullptr && statistics.find("iter_count") != statistics.end()) {
      *iterations = statistics.at("iter_count").to_int();
    }
    if (!statistics.at("success").to_bool() || solution.size() != variable_count) {
      return false;
    }
    // [RHC-1] 返回并记忆第一项 u_0^*，后者也是下周期 Delta u_0 的 u_-1。
    std::copy_n(solution.begin(), 4, command);
    std::copy_n(solution.begin(), 4, previous.begin());
    // [WARM-2] U_guess,k<-U^*_(k+1)，末段重复，构造下一周期热启动。
    for (size_t k = 0; k + 1 < horizon; ++k) {
      std::copy_n(solution.begin() + 4 * (k + 1), 4, warm_start.begin() + 4 * k);
    }
    std::copy_n(solution.end() - 4, 4, warm_start.end() - 4);
    return true;
  }

  size_t horizon;
  double dt;
  double gravity;
  size_t variable_count;
  size_t external_parameter_count;
  size_t symbolic_parameter_count;
  std::vector<double> warm_start;
  std::vector<double> lower;
  std::vector<double> upper;
  std::array<double, 4> previous;
  casadi::Function solver;
};

extern "C" Px4ctrlCasadiNmpcBridge *px4ctrl_casadi_nmpc_create(
  size_t horizon, double dt, double gravity,
  double minimum_thrust_acceleration, double maximum_thrust_acceleration,
  const double maximum_body_rate[3], int maximum_iterations,
  char *error, size_t error_capacity)
{
  try {
    if (horizon == 0 || dt <= 0.0 || gravity <= 0.0 ||
      maximum_body_rate == nullptr)
    {
      copyText("invalid CasADi NMPC options", error, error_capacity);
      return nullptr;
    }
    return new Px4ctrlCasadiNmpcBridge(
      horizon, dt, gravity, minimum_thrust_acceleration,
      maximum_thrust_acceleration, maximum_body_rate, maximum_iterations);
  } catch (const std::exception &exception) {
    copyText(exception.what(), error, error_capacity);
    return nullptr;
  }
}

extern "C" void px4ctrl_casadi_nmpc_destroy(Px4ctrlCasadiNmpcBridge *bridge)
{
  delete bridge;
}

extern "C" void px4ctrl_casadi_nmpc_reset(Px4ctrlCasadiNmpcBridge *bridge)
{
  if (bridge != nullptr) {bridge->reset();}
}

extern "C" int px4ctrl_casadi_nmpc_solve(
  Px4ctrlCasadiNmpcBridge *bridge,
  const double *parameters, size_t parameter_count,
  double command[4], double *objective, int *iterations,
  char *status, size_t status_capacity)
{
  if (bridge == nullptr) {
    copyText("CasADi bridge is null", status, status_capacity);
    return 0;
  }
  try {
    std::string solver_status;
    const bool success = bridge->solve(
      parameters, parameter_count, command, objective, iterations, solver_status);
    copyText(solver_status, status, status_capacity);
    return success ? 1 : 0;
  } catch (const std::exception &exception) {
    copyText(exception.what(), status, status_capacity);
    return 0;
  }
}
