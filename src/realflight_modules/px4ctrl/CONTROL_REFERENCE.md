# 控制参考与前馈接口

三个外环统一接收 `px4ctrl::ReferenceWindow`。参考定义及无 ROS 转换函数在
`control_reference.h/.cpp`；不依赖规划器、CSV 或 `OmTrajectoryResult`。
轨迹已重构为连续时间 `Trajectory::evaluate(t)` 接口；详见 [轨迹实现与参数](TRAJECTORIES.md)。没有新增飞行参考发布话题。

## 调用与数据

`stamp` 为窗口首点对应的控制时刻，点 k 对应 `stamp + k*dt`。
调用者每周期提供新窗口；控制器不缓存上一帧预瞄，也不从单点隐式外推。

| 控制器 | 窗口 | 适配后算法入口 |
| --- | --- | --- |
| QuadControl | 1 点，horizon=0 | 原 `Ref_State_t` 控制律 |
| OmMpcControl | horizon+1 点，使用 `mpc.prediction_dt` | 名义状态/输入与 QP 修正 |
| AcadosNmpcControl | horizon+1 点，使用 `nmpc.prediction_dt` | `AcadosNmpcSolver::step` |

真实控制间隔仍用于积分、热启动等，不再改变 OmMPC 的预测网格。
手动模式单独通过 `ControlModeReference` 传递姿态/油门，允许空窗口，不运行轨迹适配。

`ReferencePoint` 有两种权威数据形式：

- `full_state=false`：提供 p/v/a/jerk/snap/yaw/yaw_rate/yaw_acceleration。
  `resolveReference()` 解析构造姿态、名义推力、角速度、角加速度。
  `extrapolateFlatReference()` 用常 snap / 常航向角加速度产生导数一致的短窗口。
- `full_state=true`：提供 p/v/q/thrust_acceleration/body_rate；角加速度可选。
  `angular_acceleration_valid` 区分缺失与有效零。缺失时不启用内环前馈。
  `kinematics_valid=true` 时同时提供解析 p/v/a/jerk/snap（新解析轨迹）；
  为 false 时不声称拥有 jerk/snap（omtraj CSV）。传统控制器使用明确的角速度/角加速度前馈通道。

输入先经过 `resolveReference()`，再提供给控制器。窗口长度、步长、有限性及
单位四元数由控制器边界校验。无效参考在 FSM 中清除源、重置控制器并转入悬停；
不会继续执行上一条窗口。模式切换和更换轨迹仍需重置控制器历史。

坐标使用与反馈一致的局部 z 向上世界系、FLU 机体系；当前 PX4 桥实际是
NED→NWU 的 `[x,-y,-z]`，本次没有改动它。q 为参考机体到世界的旋转。
`thrust_acceleration` 是 N/kg（m/s²），不是油门；输出 `thrust` 是 N。
完整状态参考是无气动补偿的名义运动，a 按 `R e3 a_T - g e3` 恢复；
NMPC 自己负责其模型相关的气动参考补偿。

航向加推力的平坦映射使用正常飞行的局部姿态表示，要求推力朝上，并拒绝奇异点。
倒飞/翻滚应提供完整四元数形式；它仍不保证传统控制器能跟踪任意机动。

## 前馈链路

```text
轨迹/模式参考 → ReferenceWindow → 各控制器适配器
  → Control_Setpoint_t → RatesThrustSetpoint → 角速度环 → 力矩分配
```

`body_rate` 和 `body_acceleration` 是名义参考机体系中的量；下发角加速度前馈前，
用 `R_actual^T R_reference` 转换到当前机体系。它是名义物理角加速度，
不包含反馈修正，也不是跨周期差分最终角速度命令。内环按消息保持这些机体系分量。

输出 `rate_dot_ref_valid=false` 表示禁用前馈。手动、求解回退、调参覆盖时禁用；
内环收到无效标志或指令超时后清零前馈。角速度环保持原来的
`alpha_cmd = PID + rate_dot_ref`、`torque = J alpha_cmd + omega × J omega`，
不重复添加一份轨迹力矩。

NMPC 的一阶内环近似现在包含已知的前馈：

```text
omega_dot = (omega_command - omega) / tau + alpha_ff
omega_command_ref = omega_ref + tau * (alpha_ref - alpha_ff)
```

每个预测区间的 alpha_ff 是参数，不是优化变量；首区间参数与实际下发一致。
有完整前馈且姿态对齐时，输入参考就是 omega_ref；缺失前馈时保留滞后补偿。
求解失败后使用原反馈回退，并禁用前馈。模型仍是一阶近似，不等价于完整 PID、
滤波、电机模型；接通前馈后需要重新检查实际闭环表现。

## 轨迹源与模式边界

`AnalyticTrajectory` 直接返回 `ReferencePoint`，`TrajectoryPlayer` 对齐进入 CMD 的位置/航向，并按真实时间生成预测窗口。没有旧生成器、旧 CMD 宏、`set_2D8_ref()` 或 `LegacyTrajectoryReference`。

`SampledTrajectory` 只服务于离散源；omtraj 的 CSV 在 `omtraj_reference.cpp` 转成通用定时参考后，才交给统一轨迹播放器。它的角加速度仍是差分近似，不伪造 snap。

三个控制器公共入口统一使用 `ReferenceWindow` + `ControlModeReference`；不再接受包含另一份位置参考的 `Ref_State_t`。后者仅留在 FSM 模式参考内部与 QuadControl 私有控制律。

NMPC 对新解析轨迹的气动修正同步解析更新姿态、角速度和角加速度；旋转构造保持倒飞分支。只有缺乏解析导数的离散源采用差分重建。

## 构建和验证

`RatesThrustSetpoint` 新增了前馈有效标志，外环、内环和消息包必须一起重新构建和重启：

```bash
colcon build --packages-select ratectrl_msgs px4ctrl --symlink-install
colcon test --packages-select px4ctrl --ctest-args -R '^(trajectory_test|control_reference_test|controller_adapter_test|acados_nmpc_test)$' --output-on-failure
```

生成模型的参数维度由 12 改为 15，新增三个前馈分量；生成脚本及 C 文件同步更新。
测试覆盖解析导数、坐标变换、缺失/非法参考、预测网格、前馈模型、回退清零和既有求解器回归。
