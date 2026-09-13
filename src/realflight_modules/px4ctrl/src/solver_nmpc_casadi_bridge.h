#ifndef PX4CTRL_SOLVER_NMPC_CASADI_BRIDGE_H_
#define PX4CTRL_SOLVER_NMPC_CASADI_BRIDGE_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Px4ctrlCasadiNmpcBridge Px4ctrlCasadiNmpcBridge;

// 创建 CasADi 符号 NLP。a_T 的单位为 m/s^2，body rate 的单位为 rad/s；
// 返回 opaque handle，避免 CasADi 类型进入其余控制器编译单元。
Px4ctrlCasadiNmpcBridge *px4ctrl_casadi_nmpc_create(
  size_t horizon, double dt, double gravity,
  double minimum_thrust_acceleration, double maximum_thrust_acceleration,
  const double maximum_body_rate[3], int maximum_iterations,
  char *error, size_t error_capacity);

void px4ctrl_casadi_nmpc_destroy(Px4ctrlCasadiNmpcBridge *bridge);
void px4ctrl_casadi_nmpc_reset(Px4ctrlCasadiNmpcBridge *bridge);

// 参数公式 [CASADI-PARAM-1]：
//   parameters = x_measured(PVQ,10)
//              + (N+1) * [x_ref(PVQ,10),a_T_ref,omega_ref(3)]。
// 输出 command=[a_T,omega_x,omega_y,omega_z]，即滚动时域的 u_0^*。
int px4ctrl_casadi_nmpc_solve(
  Px4ctrlCasadiNmpcBridge *bridge,
  const double *parameters, size_t parameter_count,
  double command[4], double *objective, int *iterations,
  char *status, size_t status_capacity);

#ifdef __cplusplus
}
#endif

#endif  // PX4CTRL_SOLVER_NMPC_CASADI_BRIDGE_H_
