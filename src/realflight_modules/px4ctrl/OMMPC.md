# OMMPC：流形误差 QP 与本项目执行器模型

控制器宏 `PX4CTRL_PRIMARY_CONTROLLER=1`。参数在 `config/params.yaml → mpc`，内环参数在 `gain.gain_rate_*`，启动时读取，修改后重启两个控制节点。沿用现有角速度 PID 与电机分配链。

当前内外环联合调参及验证见 [OMMPC_FULL_TUNING.md](OMMPC_FULL_TUNING.md)。前一轮外环参数筛选和时钟排查见 [OMMPC_TUNING.md](OMMPC_TUNING.md)。本文末尾首次重构验证数据使用当时的参数，不代表后续配置。

当前 `config/params.yaml` 为八字专项参数，原翻转参数保存在 `config/params_ommpc_flip.yaml`。切轨迹名称不会自动切换内环与预测模型；完整切换表和八字复测结果见上述联合调参记录的“八字专项配置与轨迹切换”一节。

## 与 Lu 等（2023）的关系

依据本地论文《On-Manifold Model Predictive Control for Trajectory Tracking on Robotic Systems》第 IV 节式 (7)–(13)、第 VI-A 节式 (14)–(16)：在参考轨迹每个节点建立局部误差，用 SO(3) 指数/对数处理姿态，凝聚成输入修正量的 QP，只执行第一条命令。继续使用项目已有 OSQP，不引入论文所用 OOQP。

论文的飞行实验使用 8 步预测、100 Hz 外环和 PX4 250 Hz 角速度内环。论文的 9 维模型把角速度当作直接输入，并没有建模本文的自研内环、电机动态或阻力。预测长度和论文权重不能脱离实际执行链直接照搬。本文选择更长预瞄，并保留一次 QP 求解，属于针对本项目的模型扩展，不宣称是原论文参数的逐项复现。

内部世界系 z 向上、机体系 FLU，R 从机体旋转到世界。动力学使用 `-g e3 + aT R e3`；不要照抄论文 FRD 约定下的推力符号。

## 模型与实现

`ommpc_solver.h/.cpp` 不依赖 ROS。`ommpc.h/.cpp` 只负责状态输入、手动模式、牛顿单位推力输出及日志。

状态为 `(p,v,R,omega,aT)`，局部误差维数 13，其中姿态误差仍只有 3 维 `Log(Rd^T R)`。`omega` 来自实测陀螺，`aT` 由最近实际下发命令和推力响应模型传播估计，初值为 g；它不是实测推力，也不是在线质量辨识。

```
p_dot = v
v_dot = -g e3 + R ([0,0,aT] + aero(R^T v))
omega_dot = (omega_cmd - omega) / tau_rate + alpha_ff
aT_dot = (aT_cmd - aT) / tau_thrust
R_dot = R hat(omega)
```

- 执行器采用一阶响应，平移采用中点近似，姿态用指数映射保持 SO(3)。这不是完整电机/力矩模型；内环时间常数是闭环近似，内环或机体改变后应重新验证。
- 气动模型与 MuJoCo 的 `aero.kdx/kdy/kdz/kh` 相同，除以质量得到加速度系数，假定无风。参考与预测共用这些系数，参考沿用现有几何气动补偿函数。
- 参考角加速度经过坐标转换后发送给内环，同时作为已知输入进入预测；不重复用 `tau * alpha` 补偿已经发送的角加速度前馈。CSV 缺失导数时按参考窗口显式重建。
- 名义推力命令含 `tau_thrust * d(aT_ref)/dt`，补偿推力响应；实际推力状态仍为 `aT_ref`。
- 中心差分构造离散误差 Jacobian；保留参考的离散缺陷 `c`，不强行假定连续参考严格满足离散积分器。当前每周期一次线性化、一次 QP，没有迭代 SQP。
- QP 决策维数仍为 `4N`。逐节点累积 Hessian，避免构造大的 Qbar。OSQP 固定矩阵稀疏结构并复用工作区，上一周期绝对命令按 `control_dt/prediction_dt` 插值移动后作为 primal 初值；dual 清零。
- 代价是状态误差、相对名义输入的修正量，加上首条命令与上次实际命令之差。后者是软平滑项，不是硬角加速度约束。
- QP 约束为总推力和角速度边界；成功后第一条命令再次限幅。没有宣称在线逐电机约束或全局稳定性保证；轨迹仍接受公共生成器的名义逐电机审查。

默认 `N=16, dt=0.03 s`，预瞄 0.48 s，外环保持 100 Hz。新的代价适用于扩展模型和较长时域，不能按倍率与旧 80 ms 模型比较。不要用提高位置权重代替辨识内环响应。

## 故障与诊断

无效参考继续走 FSM 既有参考恢复逻辑；无效状态、长时间间断、求解失败或预算耗尽，使用当前位置/速度和姿态反馈生成有界命令，关闭角加速度前馈，清除求解历史，不保持旧翻转角速度。`dt=0` 允许出现，用于墙钟调度周期重复读到同一仿真时钟的情况。反馈回退不是任意翻转状态的安全保证。

`/debugPx4/ctrl` 新增 `ommpc_*`：active、solved、fallback、OSQP status、iterations、consecutive_failures、cycle_time_ms、solve_time_ms、objective、residual。

- 状态 1/2：OSQP solved/solved inaccurate；-1：状态或 dt 无效；-2：模型/QP 数值无效；-3：建模已超预算。
- `cycle_time_ms` 包含参考准备、建模、QP 与回退；`solve_time_ms` 仅 OSQP 求解。
- 求解预算在建模后检查，并把余量交给 OSQP；不是操作系统级硬实时抢占。
- `residual` 是全目标统一缩放后的 OSQP primal/dual 残差最大值；`objective` 还原缩放但不含与决策无关的常数项。

## 编译与仿真

消息定义发生变化，所有消费者需要重建；不要运行旧节点可执行文件搭配新消息库。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to px4ctrl flight_data_recorder quadsim_mujoco \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ctest --test-dir build/px4ctrl --output-on-failure -R 'ommpc_test|trajectory_test|control_reference_test|controller_adapter_test|acados_nmpc_test'
ros2 launch px4ctrl run_ctrl.launch.py use_sim_time:=true
# 另一个已加载工作区的终端：
ros2 launch quadsim_mujoco mujoco.launch.py
```

`run_ctrl.launch.py` 启动两个控制节点与记录器，`mujoco.launch.py` 启动 GUI 仿真，以上命令使控制节点使用 `/clock`。仿真刚体与电机积分均使用固定物理步长，墙钟只用于节流；姿态矩阵使用当步四元数。

实机继续使用 `run_ctrl.launch.py` 默认 `use_sim_time=false`，并关闭源码中的仿真/无遥控开关。不要把仿真时钟带入实机。

无渲染闭环实验复用 `script/acados_closed_loop/`，虽然目录沿用旧名称，但支持 OMMPC/acados，使用真实 C++ 控制节点、自研内环和 MuJoCo 电机/气动模型。分析窗口由当前轨迹生成器的 main_start/main_end 确定，不再硬编码旧八字时长。

```bash
python3 script/acados_closed_loop/run.py vertical_check \
  --output-root datalog/ommpc_modif --duration 22 --controller-label ommpc
MPLCONFIGDIR=/tmp/ommpc_mpl python3 script/acados_closed_loop/analyze.py vertical_check \
  --output-root datalog/ommpc_modif
```

实验名必须唯一。输出保留参数快照、二进制/源码哈希、原始数据、日志和指标。模型为无风、位置/姿态接近真值、带 IMU 噪声；结果不代表实机状态估计延迟、ESC 标定误差或其他机体上的保证。

## 本分支闭环验证（2026-09-29）

测试使用项目真实 C++ OMMPC、自研角速度内环与 MuJoCo/QUAD，无风、IMU 噪声开启。基线与新版本使用相同的固定刚体/电机步长及当前四元数更新；未修改内环增益或实际电机时间常数。

评价仅使用生成器定义的主体时间窗口；另外保存含进出场的 maneuver 指标。

| 实验 | 主体位置 RMSE | 主体最大误差 | CMD 后持续在空中 | 求解回退 |
|---|---:|---:|---|---|
| baseline_vertical_network | 117.72 cm | 217.05 cm | 否 | 未记录 |
| final_vertical42 | 1.24 cm | 2.25 cm | 是 | 0.0 |
| final_vertical43 | 1.46 cm | 2.12 cm | 是 | 0.0 |
| final_ground44 | 1.27 cm | 1.95 cm | 是 | 0.0 |
| final_helix42 | 1.32 cm | 3.74 cm | 是 | 0.0 |
| final_horizontal42 | 0.78 cm | 1.77 cm | 是 | 0.0 |
| final_eight42 | 0.17 cm | 0.36 cm | 是 | 0.0 |

基线发生接地，其 RMSE 包含失飞数据，不能作为同类正常跟踪精度计算提升百分比。基线可执行文件的哈希已保存，不能把留档旧可执行文件与新的消息库混用。

竖直圆三个最终工况分别为 seed42、seed43 悬停起点，以及 seed44 从 GUI 模型地面高度 0.0023 m 起飞。其余轨迹使用 seed42。配置只切换轨迹类型，OMMPC 权重相同。

最终组 CMD 段完整控制计算耗时的 99 分位约 0.37～0.56 ms，外环输出周期中位数 10 ms；仍存在调度长周期。位移拟合的有效滞后约 -5～2.5 ms（2.5 ms 搜索网格），该量不是通信延迟或相位裕度。

翻转动作有约 0.1%～0.63% 的逐电机输出油门样本触及下限，上限未触及；水平圆和八字未触及输出上下限。输出油门触限比例不等同于分配器内部推力裁剪比例。未施加在线逐电机硬约束，当前结果不是完整鲁棒性证明。

编译通过；8 个 OMMPC 单元测试通过，另有 acados、轨迹、参考、控制接口四组回归测试通过。Python 语法、launch 描述解析和新 C++ 文件格式检查通过。GUI 渲染过程未自动实跑；闭环数据来自无渲染适配器。

完整原始数组、参数、指标与节点日志位于项目 `datalog/ommpc_modif/`。总表为 `validated_metrics.json`，竖直圆对照图为 `comparison.png`。实验数据目录被 Git 忽略，源码与本文可随分支保留，原始数据需单独留存。
