# 螺旋翻转 OMMPC 调参（2026-09-29）

本页保留上一轮实验记录；当前内环、前馈与模型的联合配置见 [OMMPC_FULL_TUNING.md](OMMPC_FULL_TUNING.md)。

当前分支 `ommpcmodif`。共测试 16 套配置、25 次闭环仿真。最终只把 YAML 的位置权重从 `[150,150,150]` 改为 `[300,300,300]`；轨迹、内环增益、执行器时间常数和预测时域保持原值。

## 先统一仿真时钟

用户使用两个终端分别启动控制器和 MuJoCo。日志 `flight_20260929_213220_277650_e2a653b2.ulg` 中，控制器消息时间约为 1790688745～1790688771 秒，传感器时间却为 1000～1018.27 秒。18.27 秒物理运动花了 22.7879 秒墙钟时间，整体约 0.80 倍实时速度。控制器走墙钟、物理仿真走固定步长，参考推进和动力学时间尺度不一致。GUI CSV 后半段出现失飞；它不应与统一时钟的正常飞行数据混算调参提升。

分别启动时，控制侧必须使用：

```bash
source install/setup.bash
ros2 launch px4ctrl run_ctrl.launch.py use_sim_time:=true
```

也可以使用 `ros2 launch px4ctrl run_sim.launch.py` 一起启动。实机仍使用默认 `use_sim_time=false`。修改参数后需要重启控制节点；当前安装采用符号链接，无需因 YAML 修改重新编译。

## 采用的参数

```yaml
mpc:
  horizon: 16
  prediction_dt: 0.03
  state_weight: [300.0, 300.0, 300.0, 6.0, 6.0, 8.0, 3.0, 3.0, 2.0]
  rate_time_constant: [0.10, 0.083, 0.25]
  thrust_time_constant: 0.033
  input_weight: [0.10, 0.12, 0.12, 0.12]
  command_change_weight: [0.02, 0.08, 0.08, 0.08]
```

## 重复验证

使用真实 C++ OMMPC、自研角速度内环和 MuJoCo/QUAD；从 0.0023 m 地面高度起飞，每次 24 秒物理时间，统一 `/clock`，保留当前螺旋轨迹。无风、精确位置/姿态、IMU 噪声开启。主段窗口为进入 CMD 后 5.35～8.34123 秒；全动作指标包含进出场。三个种子不构成统计鲁棒性证明，ROS 调度和 DDS 到达顺序也会变化。

| 配置 | seed | 主段 RMSE / cm | 全动作 RMSE / cm | 全动作最大误差 / cm | CMD 后持续在空中 |
|---|---:|---:|---:|---:|---|
| baseline | 42 | 1.53 | 1.47 | 3.06 | 是 |
| baseline | 43 | 1.35 | 1.43 | 3.82 | 是 |
| baseline | 44 | 4.17 | 3.34 | 8.59 | 是 |
| position300 | 42 | 1.04 | 1.01 | 2.22 | 是 |
| position300 | 43 | 1.15 | 1.10 | 2.25 | 是 |
| position300 | 44 | 2.32 | 2.70 | 7.42 | 是 |
| position600 | 42 | 0.97 | 0.88 | 1.88 | 是 |
| position600 | 43 | 13.14 | 149.38 | 525.63 | **否** |
| position600 | 44 | 1.61 | 1.32 | 2.52 | 是 |

位置权重 150 → 300：三次主段 RMSE 的算术平均 2.35 → 1.50 cm，全动作 RMSE 的算术平均 2.08 → 1.61 cm。seed44 仍有明显波动，不能宣称每次都达到 1 cm。

权重 600 在 seed43 失飞，虽其他运行精度较高，也不推荐。失飞时求解器没有回退，说明“QP solved”不能等同于闭环稳定。其失飞数据完整保留，不剔除、不与正常飞行混算提升百分比。

### 均匀降速检查

`--real-time-factor 0.8`、seed45；只改变墙钟节流，不改变物理步长和参考速度，控制器仍使用仿真时钟。

| 配置 | 主段 RMSE / cm | 全动作 RMSE / cm | 全动作最大误差 / cm | 持续在空中 |
|---|---:|---:|---:|---|
| baseline | 1.14 | 1.18 | 2.64 | 是 |
| position300 | 1.18 | 1.06 | 2.23 | 是 |
| position600 | 1.07 | 0.92 | 2.34 | 是 |

权重 300 在降速组的主段 RMSE 略高于原参数，全动作 RMSE 和最大误差较低；并非所有指标都改善。

采用组四次测试均无求解回退，控制计算耗时的 99 分位为 0.45～0.58 ms，低于 6 ms 预算。逐电机输出油门下/上限触及样本合计比例为 0.17%～0.20%，不是分配器内部推力裁剪率。

均匀降速检查不等价于真实 GUI 随机卡顿；本轮没有自动运行 GUI 渲染，也未验证实机。统一时钟是本次使用新参数的前提。

## 单次筛选记录

以下每组均为 seed42；仅作筛选，最终决策以上面的重复测试为准。差异相对调参前完整 YAML。

| 配置 | 改动 | 主段 RMSE / cm | 全动作最大误差 / cm |
|---|---|---:|---:|
| attitude1 | `state_weight=[150.0, 150.0, 150.0, 6.0, 6.0, 8.0, 1.0, 1.0, 1.0]` | 1.58 | 2.85 |
| attitude8 | `state_weight=[150.0, 150.0, 150.0, 6.0, 6.0, 8.0, 8.0, 8.0, 4.0]` | 1.79 | 3.97 |
| baseline | 原参数 | 1.53 | 3.06 |
| grid20ms | `horizon=24`；`prediction_dt=0.02` | 1.77 | 3.70 |
| position300 | `state_weight=[300.0, 300.0, 300.0, 6.0, 6.0, 8.0, 3.0, 3.0, 2.0]` | 1.04 | 2.22 |
| position300_velocity12 | `state_weight=[300.0, 300.0, 300.0, 12.0, 12.0, 16.0, 3.0, 3.0, 2.0]` | 1.72 | 2.97 |
| position600 | `state_weight=[600.0, 600.0, 600.0, 6.0, 6.0, 8.0, 3.0, 3.0, 2.0]` | 0.97 | 1.88 |
| preview_long | `horizon=20` | 1.57 | 2.78 |
| preview_short | `horizon=12` | 1.28 | 3.19 |
| previous_q | `state_weight=[60.0, 60.0, 80.0, 6.0, 6.0, 8.0, 3.0, 3.0, 2.0]` | 1.54 | 3.98 |
| rate_fast | `rate_time_constant=[0.07, 0.058, 0.175]` | 2.84 | 6.64 |
| rate_slow | `rate_time_constant=[0.13, 0.108, 0.325]` | 1.52 | 2.69 |
| smooth_rate | `input_weight=[0.1, 0.2, 0.2, 0.2]`；`command_change_weight=[0.02, 0.3, 0.3, 0.3]` | 5.70 | 12.32 |
| thrust_fast | `thrust_time_constant=0.023` | 1.62 | 3.94 |
| thrust_slow | `thrust_time_constant=0.043` | 1.64 | 3.18 |
| velocity12 | `state_weight=[150.0, 150.0, 150.0, 12.0, 12.0, 16.0, 3.0, 3.0, 2.0]` | 1.09 | 2.83 |

较大的命令平滑惩罚、较小的模型角速度时间常数、较高的姿态权重在这次筛选中均变差。加密预测网格增加计算量且未改善误差，因此保留 16 步 / 0.03 秒。

## 数据和复现

数据目录：`datalog/ommpc_tuning_v2/`（被 Git 忽略，需要单独留存）。其中 `baseline.yaml` 为用户调参前原件，`configs/position300.yaml` 为推荐完整配置；每次运行目录包含完整参数、二进制/源码哈希、原始数组、节点日志和 `metrics.json`。`all_metrics.json` 汇总 25 次结果；`variants.json` 记录 16 套差异；`validation.png` 为三个种子对比图；`timing_evidence.json`、`user_gui_before.csv` 和 `gui_clock_mismatch.png` 保留时钟排查证据。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 script/acados_closed_loop/run.py helix_recheck \
  --config datalog/ommpc_tuning_v2/configs/position300.yaml \
  --output-root datalog/ommpc_tuning_v2 --duration 24 \
  --seed 46 --initial-altitude 0.0023 --real-time-factor 0.8
MPLCONFIGDIR=/tmp/ommpc_mpl python3 script/acados_closed_loop/analyze.py helix_recheck \
  --output-root datalog/ommpc_tuning_v2
```

实验名必须唯一。两个 launch 文件现在支持 `params_file:=/绝对路径/完整配置.yaml`，可在重启时对比原配置和候选配置。`run_ctrl.launch.py` 仍需显式附加 `use_sim_time:=true`。Python 语法检查及两个 launch 的参数解析通过；本轮未改控制算法 C++ 源码。
