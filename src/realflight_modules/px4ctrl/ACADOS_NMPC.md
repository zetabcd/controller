# acados NMPC

顶层控制器只保留 `0=QuadControl`、`1=OmMpcControl`、`2=AcadosNmpcControl`，
默认选择 2（由旧编号 3 调整）。旧 Ipopt/NLopt 控制器实现、`SolverNmpcControl`、后端工厂和 CasADi C++ bridge 已删除。

## 接口和职责

```text
FSM → AcadosNmpcControl → AcadosNmpcSolver → generated/acados/px4ctrl_nmpc
    轨迹采样/手动模式       数值配置/求解/恢复       模型与求解器结构
    单位换算/调试消息
```

- `AcadosNmpcSolver(options)`：无 ROS 依赖，直接持有 generated capsule；构造时检查
  参数并配置 OCP，非法配置抛异常，节点在启动阶段退出。
- `step(state, references, elapsed_seconds)`：传入测量状态、N+1 个参考和实际经过时间，
  返回质量归一化推力与机体角速度。返回值可能来自反馈回退，须结合 `diagnostics()`。
- `reset()`：只清空迭代、热启动和历史命令，保留 dt、权重、边界、重力和求解选项。
- `AcadosNmpcControl::configure(options, mass)`：在节点参数初始化后调用。
- `calculate(ref, pose, attitude, measured_body_rate, now_seconds, elapsed_seconds, output)`：FSM 适配入口，
  输出推力为牛顿；时间由 FSM 显式传入，不保存 ROS Node 引用；显式接收 FLU 实测角速度，作为预测状态。
- `setTrajectory`/`clearTrajectory`：使用现有 `OmTrajectoryResult`，更换轨迹时清空热启动。

参考窗口和求解轨迹缓存在配置阶段分配，正常 step 不重建求解器，也不分配动态向量。
状态/输入边界和权重仅配置一次，每帧只更新测量、参考和上一条实际控制指令。

## 调参：修改 YAML 后重启，无需生成或编译

配置在 `config/params.yaml` 的 `nmpc` 段，启动时读取，不支持飞行中热更新。

| 参数 | 含义 |
| --- | --- |
| `horizon` | 预测步数，1..200；通过 generated create_with_discretization 创建 |
| `prediction_dt` | 预测间隔，秒；总预测长度 N×dt，与控制周期独立 |
| `state_weight` | 仍为 10 项，p(3)、v(3)、q_wxyz(4)；新增的角速度状态没有单独状态罚项 |
| `rate_time_constant` | 3 项，内环角速度一阶响应时间常数，秒；默认 `[0.10,0.083,0.25]`，每项须 ≥ prediction_dt |
| `drag_compensation` | ROS 开关，默认 true；预测和参考共用 `aero.kdx/kdy/kdz/kh` 与质量，假定无风 |
| `terminal_weight` | 10 项，终端状态权重 |
| `input_weight` | 4 项，a_T、omega_xyz；必须为正 |
| `command_change_weight` | 4 项，首条指令与上周期实际命令的差，可为零 |
| `body_rate_max` | 机体系三轴角速度上限，rad/s |
| `maximum_iterations` | SQP 迭代上限，1..1000 |
| `tolerance` | KKT 各项容差 |
| `solve_time_budget_ms` | 求解预算，毫秒；超预算结果走反馈回退 |
| `fallback_position_gain` / `fallback_velocity_gain` | 回退位置/速度反馈增益 |
| `fallback_attitude_gain` | 回退姿态反馈增益 |

重力沿用 `gra`，质量沿用 `uav.mass`；质量归一化推力上下界由
`4 × motor.u_min/max / mass` 得到，与内环使用的执行器边界一致。

生成脚本中的 N、dt、g 和数值权重仅用于定义生成模板和独立示例的初值，控制节点会
全部覆盖。改变模型维度、动力学表达式、代价残差结构、约束种类、积分器或 NLP/QP
算法时，才需重新生成。

## 模型和代价

`x=[p_ENU, v_ENU, q_wxyz, omega_actual]` 共 13 个状态，
`u=[a_T, omega_command]` 共 4 个输入；姿态从机体 FLU 转到世界 ENU。
模型使用实际角速度推进姿态，并预测内环响应：

```text
p_dot = v
v_dot = R(q) * (e3*a_T + aero_body) - g*e3
q_dot = 0.5*q⊗[0,omega_actual]
omega_actual_dot = (omega_command - omega_actual) / rate_time_constant
aero_body = -linear_drag .* (R(q)^T*v)
aero_body.z += horizontal_lift * (v_body.x² + v_body.y²)
```

`linear_drag=[kdx,kdy,kdz]/mass`，`horizontal_lift=kh/mass`；关闭 ROS
`drag_compensation` 时二者置零。独立 C++ 接口通过 options 显式传入，默认无阻力。
模型参数共 12 项：g、上一帧命令(4)、时间常数(3)、阻力/质量(3)、kh/质量。
结构变化全部在生成脚本完成，生成 C 不手工修改。

参考姿态和推力补偿阻力，角速度参考由补偿后的姿态序列计算；再用
`omega_command_ref = omega_ref + tau .* omega_ref_dot` 补偿内环响应。
这是预测代价使用的输入参考；输出给内环的 `rate_dot_ref` 仍为零，避免重复补偿。
使用 ERK/RK4 多重射击、完整 SQP、Gauss–Newton、HPIPM。

代价采用明确的**离散累加**：

```text
z = x.head(10)  # 只对位置、速度、四元数计状态代价
J = Σ(k=0..N-1) [||z_k-z_ref,k||²_Q + ||u_k-u_ref,k||²_R]
    + ||z_N-z_ref,N||²_QN + ||u_0-u_applied_previous||²_S
```

所有 cost scaling 显式设置为 1，W=2×权重以抵消 acados 的 1/2。
此前的实现对阶段代价乘 dt、终端不乘，因此此次修正会改变控制响应，需要重新验证
已有调参结果。姿态仍使用四元数分量残差，参考沿预测窗连续并与当前估计对齐。
S 是**相邻控制周期的首条命令平滑项**，不是整个预测窗的 Δu 代价，也不是硬变化率约束。

时间常数是一阶等效响应近似，并非完整的 PID、电机、纯延迟模型。当前仍不预测
推力执行滞后、风和状态估计误差；换内环增益/滤波/电机后应重新辨识并闭环验证。RK4 预测中的
四元数没有硬单位范数约束；测量和热启动插值会归一化，激烈机动仍需闭环验证。

## 热启动、失败与耗时

热启动使用上次成功轨迹，按真实 elapsed/dt 重采样：位置/速度线性插值，姿态 slerp，
输入按分段常数保持，超出窗口用末端值补齐；间隔超过完整预测窗则重新初始化。
这样 100 Hz 控制、40 ms 预测网格不会每 10 ms 错移一个完整预测节点。

求解失败、非有限输入/输出或超时当帧即清空迭代，输出有界的位置/速度/姿态反馈命令，
下一帧重新初始化求解。反馈始终经过与 NMPC 一致的推力和角速度限幅；无有效姿态时
只能输出限幅后的重力推力和零角速度，无法保证稳定。回退不是完整的飞行故障管理器。

`diagnostics()` 包括 success/fallback、原始 status、迭代数、连续失败次数、目标值、
KKT 残差和整个 step 的墙钟耗时。`/debugPx4/ctrl` 每帧发布 `nmpc_active`、
`nmpc_solved/fallback/status/iterations/consecutive_failures/solve_time_ms/objective/residual`。
只有 `nmpc_active=true` 时才解释这些字段；FSM 回退日志仍以 1 秒节流输出。
这些消息原始字段可由现有飞行记录器记录，重建后必须重启消息发布/订阅节点。
状态 `-1/-2/-3` 分别表示输入非法、输出非法、墙钟预算超限，非负值来自 acados。
SQP 在迭代边界检查时间，单次迭代无法抢占，所以时间预算不是硬实时保证。

## 生成、构建和测试

普通构建只需 C++ 的 Eigen、OSQP、acados/HPIPM/BLASFEO、SuiteSparse 和 ROS 依赖，
不需要 Ipopt、NLopt 或 CasADi C++ 库。Python CasADi 仅在生成时使用。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select px4debug_msgs px4ctrl --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select px4ctrl --ctest-args -R '^acados_nmpc_test$' --output-on-failure
```

改变结构后使用 `script/generate_px4ctrl_acados_nmpc.py`，环境准备见
[第三方说明](../../../3rdpart/README.md)。不要直接编辑 generated C/H/JSON。
回归覆盖动态 N、重力、非默认 dt 的重置、权重语义、输入限幅、失败恢复、迭代/时间
限制、无效输入与四元数符号变化、内环/推力滞后下 18/20/40 权重闭环、角速度制动响应，
以及有阻力匀速平衡与 reset。单元测试使用宽松时间预算以避免宿主机调度抖动，
不能替代当前飞行器、内环和仿真模型的闭环评估。

## 2026-09-29 失稳调试

旧 10 状态模型忽略了实际约 100–125 ms 的角速度响应滞后。
仅增加位置权重会进一步提高外环纠偏速度，内环来不及执行，出现振荡。
旧模型在独立无风 MuJoCo 八字复现中 Qp=18、20 均发生失稳。
加入响应模型后，Qp=20 的 3D 位置 RMSE 为约 12.7 cm；继续补偿阻力及参考输入后
约 0.62 cm（seed 42，6×4 m，3 m/s，两圈，评估八字段，不含起飞/末尾悬停）。
这不是实机精度承诺。详细日志、对照指标和边界见工程根目录
`0913/acados_nmpc_debug_20260929.pdf`；实验结果保存在
`datalog/acados_debug_20260929/`。

建议先用当前 Qp=18 或 20。先检查模型/内环响应和求解状态，再逐步调位置权重；
无需为了这条轨迹立即提高 Qp。提高 Qp 时同时观察角速度振荡、输入限幅、求解失败率。
`command_change_weight` 只平滑相邻周期的第一条命令，不能代替正确的内环响应模型。
