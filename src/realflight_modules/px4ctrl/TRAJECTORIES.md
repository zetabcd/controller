# 解析轨迹与统一参考接口

本次按 `0913/轨迹生成/uav_trajectory_design.tex` 实现水平加速圆、竖直俯仰翻转圆、水平轴滚转螺旋，并重新实现水平八字。omtraj 按 `0913/wenzhang0710.docx` 第 III 节重构，使用含动力学模型的 v2 CSV；删除原 minimum-snap、barrel-roll、five-turn、figure-eight 生成器、`set_2D8_ref()` 和 `LegacyTrajectoryReference`。

## 使用与参数

在 `config/params.yaml` 设置 `trajectory.type`，取值为 `horizontal_circle`、`vertical_circle`、`helix`、`figure_eight`、`omtraj`。当前工作区选择为 `helix`，以实际 YAML 为准。三个控制器使用同一选择入口，不再有 CMD 轨迹编译宏。所有参数在启动时读取、生成并检查；修改后重启控制节点，活动轨迹不热切换。

| 参数 | 含义与默认值 |
|---|---|
| `trajectory.takeoff_height/duration` | 相对进入 CMD 位置上升 1.5 m，3 s；九次多项式平滑起飞 |
| `trajectory.settle_duration` | 0.5 s 起飞后稳定段 |
| `horizontal_circle.radius/turns/speed` | 半径 1 m，匀速 2 圈，1.5 m/s |
| `horizontal_circle.ramp_duration` | 各 3 s 加、减速；这些阶段额外贡献绕行角度，结束点不强制等于起点 |
| `vertical_circle.radius/turns/centripetal_g` | 半径 1 m，1 圈，向心加速度 A=1.8g |
| `helix.radius/turns/pitch/centripetal_g` | 半径 1 m，2 圈，每圈轴向推进 0.5 m，A=1.8g |
| `helix.axis_transition_duration` | 各 1 s 独立轴向加、减速，避免滚转连接段混入轴向加速度 |
| 翻转两类的 `entry/exit_duration` | 各 0.85 s；参数名分别为 `entry_duration`、`exit_duration` |
| 翻转两类的 `entry/exit_distance` | 各 1.43 m，沿圆底切线方向连接 |
| 翻转两类的 `connector_height` | 起止悬停点比圆底高 1.1 m；默认主体相对高度为 0.4～2.4 m |
| `figure_eight.length/width/turns/speed` | 全长 2 m，全宽 1.2 m，总计 2 圈，速度上限 1.5 m/s |
| `figure_eight.ramp_duration` | 各 3 s 加减速，计入总圈数；匀速时长自动使总相位闭合 |
| `limits.motor_fraction` | 当前电机静态最大推力的 70% 用作逐电机预算 |
| `limits.angular_acceleration` | 角加速度模长上限 100 rad/s² |
| `limits.minimum_relative_altitude` | 相对轨迹激活点的高度下限 -0.01 m |

圆半径硬性要求 `0 < radius <= 1 m`，因此主体圆直径不超过 2 m。进出连接段占用额外空间：默认竖直圆在局部前向总跨度约 2.86 m；螺旋横向总跨度约 2.86 m。**直径约束不等于完整动作限制在 2 m 盒子内**。螺旋主体每圈推进严格为 0.5 m，进出段也有额外轴向位移。

所有轨迹在局部原点生成。进入 CMD 时整体平移到实测位置、绕 z 轴旋转到实测航向；不使用经纬度或 NED 绝对航点。生成器假设激活时近似悬停，不从当前非零速度重新规划入口。既有 FSM 的模式切换/异常恢复机制未被替换为特技恢复规划器。

## 连续时间实现

`trajectory.h/.cpp` 不依赖 ROS、OSQP、omtraj 或控制器。空间曲线与时间律分离，按完整链式法则解析求 p/v/a/jerk/snap。水平圆使用文档的七阶速度包络 H 与积分 G；八字使用同样的相位规律，按 `sqrt((length/2)^2 + width^2)` 给出速度上界。

竖直圆和螺旋主体保持恒定相位速度 `w=sqrt(A/r)`，其中 A=centripetal_g*g，强制 A>g。减慢翻转会同时减小圆顶推力余量，不能对超限动作无限放慢。默认直径由文档候选 4 m 改为 2 m，同 A 下角速度增加 sqrt(2)，因此重新检查了整个动作。

起飞及所有连接段采用归一化时间九次多项式，两端匹配 p/v/a/jerk/snap，保证位置 C4。参数固定后系数唯一，不称为最小 snap 优化。默认连接参数经全轨迹采样筛选；不宣称对任意用户参数自动优化。螺旋先独立建立轴向速度，再连接圆周运动，退出对称处理；否则较小的轴向加速度也可能通过姿态耦合造成偏航分配超限。

由 a+g*e3 恢复推力和姿态，利用 jerk/snap 解析计算角速度及角加速度。竖直圆固定辅助 y 轴，其余使用辅助 x 轴投影，支持倒置并显式拒绝奇异点。四元数通过预计算符号锚点连续展开；锚点仅决定 q/-q，不插值位置、姿态或导数。主体多圈中不重复启停。

终点和终点之后返回静止悬停：v/a/jerk/snap/omega/alpha 均为零，推力加速度等于 g。首末端与连接段都属于连续性测试范围。

## 统一接口

```text
AnalyticTrajectory / OmReference(omtraj v2 模型积分) / SampledTrajectory
  → Trajectory::evaluate(t) → ReferencePoint
  → TrajectoryPlayer（时间、刚体对齐、预测网格）→ ReferenceWindow
  → QuadControl / OmMpcControl / AcadosNmpcControl
  → Control_Setpoint_t → RatesThrustSetpoint → 内环
```

新生成器直接输出完整 `ReferencePoint`，不创建 `OmTrajectoryResult`，不走线性缓存插值。每个控制周期按真实 ROS 时间和控制器 dt 精确求值 H+1 点。时钟回退被拒绝，随后走 FSM 既有无效参考处理。

控制器的公共入口仅接受 `ReferenceWindow` 和独立的 `ControlModeReference`。模式数据只有状态、手动态度/油门及有效标志，不再附带另一份 p/v/a。FSM 的手动/悬停内部状态和 QuadControl 私有旧控制律仍有 `Ref_State_t`，但它不再作为控制器公共入口，也不用于传递新轨迹。

`kinematics_valid` 表示 p 至 snap 来自同一可微曲线，`angular_acceleration_valid` 区分有效零与缺失角加速度。NMPC 对解析轨迹的气动补偿连同一、二阶姿态导数一起解析计算，保留倒飞姿态分支；不在默认气动补偿打开后又把解析前馈退化为窗口差分。这是基于名义姿态的一次气动力修正，并非耦合气动方程的全局求解。

omtraj 在文件边界封装为 `Trajectory`，优化器类型不进入 FSM 或控制器。`OmReference` 用优化时的动力学和一阶保持（FOH）推力、角速度积分求值；角加速度、推力变化率直接取区间斜率，避免独立插值造成 p/v/q/u 不一致。它不提供 jerk/snap。播放要求首末状态为水平静止悬停，结束后保持末位置。节点动力学残差受独立校验容差限制，不宣称严格 C4。详细实现与参数见下节。

下层 `RatesThrustSetpoint` 的单位与消息布局不变：总推力 N、机体角速度 rad/s、角加速度前馈 rad/s²。角加速度按实际姿态转换到当前机体系。轨迹力矩只用于离线分配检查，不叠加到内环重复计算。

## 检查、导出与对比

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select px4ctrl --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/px4ctrl -R '^(omtraj_test|trajectory_test|control_reference_test|controller_adapter_test|ommpc_test|acados_nmpc_test)$' --output-on-failure
python3 script/analyze_trajectories.py
```

最后一条命令读取实际 YAML，调用无 ROS 副作用的 `trajectory_inspect`，输出到 `datalog/trajectory_refactor/`：四条 CSV、冻结配置、`metrics.json` 和 `trajectories.png`。需要 Python 的 PyYAML、NumPy 和 Matplotlib。也可直接调用 `build/px4ctrl/trajectory_inspect --type helix --radius 1 --pitch 0.5 --turns 5 --output /tmp/helix.csv`；直接调用使用 C++ 默认模型，使用项目 YAML 对比应走 Python 脚本。

2026-09-29 当前参数的 1 ms 全动作采样结果（包括起飞/进出段）：

| 轨迹 | 时长 s | 最大速度 m/s | 最大推力加速度 m/s² | 最大角速度 rad/s | 最大角加速度 rad/s² | 单电机推力范围 N |
|---|---:|---:|---:|---:|---:|---:|
| 水平加速圆 | 17.878 | 1.500 | 11.367 | 0.443 | 1.139 | 1.671～2.305 |
| 竖直翻转圆 | 6.696 | 4.218 | 27.454 | 9.452 | 81.528 | 0.719～5.595 |
| 水平轴螺旋 | 10.191 | 4.231 | 27.454 | 9.452 | 81.528 | 0.820～5.588 |
| 水平八字 | 19.586 | 1.500 | 11.367 | 0.448 | 1.330 | 1.671～2.305 |

旧五圈 QP 参数曾测得峰值速度 7.65 m/s、推力加速度 65.42 m/s²、角速度 10.85 rad/s，首尾角速度约 0.146 rad/s。新方案降低了名义推力峰值，解决了首尾残余角速度和缓存差分前馈，并使直径/每圈推进量可直接解释。但旧方案与新默认圈数、时间律不同，这不是同一任务的闭环性能对照；不能仅据这些峰值宣称实际跟踪更好。

采样检查使用当前质量、惯量、力臂、反扭矩系数以及真实内环电机编号，联合计算 `tau=I*alpha+omega×I*omega` 后逐电机检查上下限。检查通过仅表示无气动名义参考与静态分配可行；没有替代电机迟滞/来流/电池限流和真实闭环仿真，也不是连续时间峰值证明。高度限制是相对激活位置，不是地形避障。原系统的特技中断恢复能力仍需单独验证。


## omtraj：文稿方法、约束与控制器对接

依据 `0913/wenzhang0710.docx` 第 III 节实现；Shen 2024 仅用于航点约束的参考，没有替换为其 13 维状态优化方案。物理目标始终是总时间 T。计算时间通过稀疏实现减少，可跟踪性通过约束限定；没有添加轨迹平滑、控制能量或跟踪误差作为竞争目标。

| 文稿内容 | 实现 |
|---|---|
| x=(p,v,R)，右乘 SO(3) 局部增量 | 每节点 9 维状态增量；四元数仅用于存储和积分，更新 R exp(δθ) |
| t=Tτ，自由终止时间 | ΔT 与状态、输入、航点进度在同一个 SCP 子问题联合优化，无外层时间二分 |
| 线性化动力学、虚拟控制 | 保留名义动力学缺陷、下一节点旋转对数映射的导数及 ΔT 导数；正常步骤将虚拟控制置零；不可行时按需加入 L1 绝对值上图变量恢复可行性 |
| CSTC 航点 | μ(||p−wp||²−r²)≤0；λ₀=1、λN=0、λk+1=λk−μk；不预先锁死通过时刻 |
| 有序通过 | λj≤λj+1；结合非负进度和 CSTC 保证首次通过的顺序，并独立检查实际按序进入航点球 |
| 姿态锥 | 世界竖直与机体 z 轴夹角；全局限制或按航点触发的 `waypoints.tilt_i` |
| 数值稳定 | 状态/进度增量正则、信赖域、实际非线性 merit 回溯、逐步收紧 CSTC 松弛 |

文稿中的旋转方向按 `R: body→world` 统一；动力学线性化使用 ΔT 而非重复累加绝对 T，并保留名义缺陷。这些是使公式可执行的一致性修正。增量正则及虚拟控制罚项属于 SCP 求解机制，不是另加的物理优化指标。保留微小输入、时间增量正则以稳定 QP。

代码拆分为 `omtraj.cpp`（SCP）、内部 `omtraj_qp.h`（稀疏 QP/预消元）、`omtraj_dynamics.cpp`（模型/约束/积分）、`omtraj_io.cpp`（独立校验与文件）、`omtraj_reference.cpp`（控制参考源）。移除额外的进度乘积排序约束和航点辅助松弛变量，保留原始 CSTC 与逐步收紧的 σ。每次 QP 代入固定变量、合并单变量约束，再按当前信赖域筛掉不可能激活的线性行；这些行下一次重新评估，不锁死航点时刻。

精度随非线性残差收紧，避免远离可行域时过度求解。接近可行域时，修正步骤暂不缩短 T，但允许略增 T 以消除缺陷，随后继续时间优化。筛选后的行结构会变化，因此每次对子问题重新分解，避免累积无用稀疏项和复用失配的对偶变量。QP 状态只决定是否尝试更新，不能充当物理可行性证明。

### 一套优化器、一份参数、一个控制接口

唯一参数文件是 [omtraj.yaml](config/omtraj.yaml)，统一设置任务航点/边界、`model` 动力学模型、`limits` 机动约束与余量、`optimizer` 数值参数。launch 只加载这一份文件，没有算法版本或 profile 选择。

```bash
# 优化过程和最终轨迹可视化
ros2 launch px4ctrl omtraj_visualizer.launch.py
# 单次生成，不打开 RViz，完成后退出
ros2 launch px4ctrl omtraj_visualizer.launch.py rviz:=false exit_after_solve:=true
```

每次成功后统一输出 `datalog/omtraj/omtraj_manifold_v2.csv`。`v2` 是包含动力学模型的文件格式版本，不是另一套规划算法。此前 4.32 s、4.50 s、8.08 s 是同一个优化器在不同参数下的结果；旧参数与原始记录保存在 `datalog/omtraj_planning_only/` 和 `datalog/omtraj_fast/final_planning/`，不再保留多个日常启动配置。

纯规划对照曾使用：完整 12 cm 航点半径、4g 推力上限、各轴 14 rad/s 角速度、逐电机静态上下限和一阶推力响应预算。角加速度、推力变化率和速度设置为较宽的数值上限，不代表已识别的硬件极限。全局倾角不受限，任务指定的航点姿态锥仍生效。该组得到 T=4.503641 s，通过独立可行性验收但 80 轮内未满足局部收敛条件。`limits.thrust_time_constant` 改为 0.0 时，对照得到 T=4.323811 s；它是执行器模型近似参数，不是算法开关。

控制器统一通过以下链路接入：

```text
omtraj.yaml → OmTrajectoryOptimizer → omtraj_manifold_v2.csv
  → OmReference → TrajectoryPlayer → ReferenceWindow
  → 已选择的 QuadControl / OMMPC / acados → 内环
```

`OmReference` 按优化模型和 FOH 输入积分求 p/v/R/aT/ω，并提供推力变化率、角加速度前馈。三个控制器共用参考接口，无需各自生成轨迹。解析轨迹 helix/圆/八字是其它参考源，也接入同一个 `Trajectory` 接口，并非 omtraj 的不同版本。

优化入口只生成/显示轨迹。执行时在 `params.yaml` 设置 `trajectory.type: omtraj`，`trajectory.omtraj.file` 指向上述统一文件，再重启控制节点；启动检查仍验证模型与机动范围。当前控制选择保持原样。统一接口不代表任意激进轨迹都能跟踪，纯规划结果可能超出控制器的启动验收范围。当前 `omtraj.yaml` 已恢复为下表通过 5 cm 闭环验收的参数组，飞行时间约 8.08 s。

需要考虑跟踪时，只修改同一文件的 `limits` 约束，物理目标仍然是总时间 T。例如此前通过闭环验收的一组参数为：

| 参数 | 已验证跟踪工况的取值 |
|---|---|
| `thrust_min` / `thrust_max` | 1 / 30 m/s² |
| `rate_max` | 各轴 4 rad/s |
| `thrust_slew_max` | 40 m/s³ |
| `angular_acceleration_max` | 各轴 16 rad/s² |
| `speed_max` / `maximum_tilt` | 6 m/s / π/2 |
| `motor_min` / `motor_max` | 0.5 / 17.972347331460142 N |
| `waypoint_margin` | 0.05 m |

航点余量将 12 cm 任务球收紧为 7 cm 规划球；若实际跟踪误差始终不超过 5 cm，对应时刻仍处于任务球内。这是条件性的几何关系，不是闭环误差证明。可跟踪性目前通过参数收紧与闭环试验校准，优化器没有内嵌整个控制器或任意外扰下的误差上界。

动力学/静态分配按文稿忽略转子惯性，不覆盖电池压降、完整 PID/滤波动态或所有外扰。参数过紧、初值较差或最大时间太短仍可能导致不可行或不收敛。40 区间控制表达能力，12 个 RK4 子步控制积分精度；不能靠放宽验收容差制造成功。`initial_speed` 只影响初始化，不是速度约束。

### 验收、CSV 与播放

优化结束用两倍积分子步复查动力学，检查粗细积分差；积分误差过大时自动倍增 RK4 子步（最多 100），不会增加 QP 维度。并按每区间至少 10 点、间隔不大于 5 ms 的网格检查物理约束。节点间出现超限时，把该区间最差位置加入下一次 QP。这个过程是数值加密校验，不是连续时间峰值的解析证明。动力学分别采用位置/速度/姿态容差；`constraint_tolerance` 应用于归一化的约束行，不能统一解释成米或弧度。

`success` 表示存在通过独立验收的轨迹；`converged` 另外要求原约束通过、σ=0、退出可行性恢复、子问题达到求解精度且局部模型预期改进相对信赖域足够小。采用模型改进作为近似驻点判据，避免仅因冗余输入方向仍变化就耗尽预算。仅 OSQP 达到迭代上限不能获得该标志。达到 SCP 预算时可返回此前最短的已验收轨迹，并注明未收敛；不声称全局最优。

v2 CSV 保存模型、积分子步数和节点 p/v/q/aT/ω；加载时重新检查模型动力学。CSV 本身不保存完整任务约束或优化历史，因此“可加载”不等于对任意新参数文件可行，也不会伪造 converged=true。旧 v1 文件可读取检查，但控制播放器拒绝执行，需要重新规划。保存先写临时文件，成功后原子替换。

统一输出 `datalog/omtraj/omtraj_manifold_v2.csv`。在 `params.yaml` 设置 `trajectory.type: omtraj` 并确认 `trajectory.omtraj.file` 后，重新启动控制节点。播放器按进入 CMD 的位置、偏航整体对齐，CSV 起始高度不是绝对起飞命令。

接口传递 `aerodynamics_included`、模型系数、`thrust_rate`/有效标志和机体角加速度。OMMPC、acados 在模型匹配时保留规划器的气动参考，不再二次补偿；不匹配则拒绝该参考。OMMPC 优先使用解析的区间推力斜率构建执行器输入。已有公共 `ReferenceWindow` 接口和内环前馈通道继续使用，控制器不依赖优化器内部变量。

批量规划示例（在工作区根目录，先构建并 source）：

```bash
ros2 run px4ctrl omtraj_visualizer_node --ros-args \
  --params-file src/realflight_modules/px4ctrl/config/omtraj.yaml \
  -p exit_after_solve:=true
```

### 回归与闭环证据

测试覆盖 QP 消元与矛盾约束、点到点、自由时间、有序航点、低速限制、动力学导数、模型一致性、CSV 篡改拒绝、姿态锥和逐电机限制；同时运行公共参考接口、控制器适配和两种 MPC 回归。

旧版七航点基线在 `datalog/omtraj_refactor/final_planning/`：60 区间、T=8.979448 s、耗时 34.221 s、100 次 SCP、未满足局部终止条件。QP 占 33.846 s，77 个子问题达到 3000 次迭代上限。保守约束不意味着 QP 容易求解：重复约束、辅助松弛以及不必要的高精度都会拖慢收敛。

2026-09-30 跟踪参数组复核（本机单次计时，不能当作实时上界）：

| 配置 | 飞行时间 | 规划耗时 | SCP 轮数 | 局部终止条件 |
|---|---:|---:|---:|---|
| 旧实现、原 60 区间及约束 | 8.979448 s | 34.221 s | 100 | 未满足 |
| 新实现、同一 60 区间及约束 | 9.085473 s | 7.606 s | 100 | 未满足 |
| 跟踪参数组、40 区间及闭环验收约束 | 8.077670 s | 3.322 s | 35 | 满足 |

同网格对比保留旧数值配置，耗时下降约 4.5 倍，但返回的局部可行解略长；不能把它说成同样最优性的加速。跟踪参数组同时调整网格、数值参数和物理范围，整体约快 10.3 倍，飞行时间缩短约 10%。该次运行的 34 个已接受 QP 步耗时中位数 76.5 ms、P95 196.7 ms，日志保留每步 OSQP 迭代数与残差；仍属于秒级离线规划。参数、源码哈希、日志、旧默认备份及最终参考位于 `datalog/omtraj_fast/final_planning/`。

| 最终参考闭环 | 位置 RMSE | 最大位置误差 | 控制回退 | 电机输出触限 |
|---|---:|---:|---:|---:|
| `agile40_v12_noise7` | 1.94 cm | 3.41 cm | 0 | 0 |
| `agile40_v12_noise42` | 1.47 cm | 3.25 cm | 0 | 0 |

两组均按序进入全部 7 个任务航点球（半径 12 cm），并覆盖全程飞行及约 6.77 s 末端保持。实际电机消息分别持续到 1016.9925 / 1016.9900 s，仿真结束于 1017.0 s。详细结果见各目录的 `metrics.json`、`waypoint_audit.json`、`raw.npz` 与日志。当时导出的 CSV 与两组冻结参考逐字节相同。六个测试套件共 49 个测试通过。

当前规划和闭环数据见 `datalog/omtraj_fast/`。闭环使用当前 OMMPC 与 MuJoCo 电机/刚体/气动模型，位置噪声标准差 0.01 m、速度噪声标准差 0.03 m/s。更激进的 6.489869 s 候选最大误差达到 6.25 cm，未满足验收；跟踪配置据此收紧角加速度、推力变化率，并增加电机低端余量。

每组实验冻结参考 CSV、配置、二进制和源码哈希。独立 ROS 域避免实验串扰；分析器拒绝时间倒退或控制数据未覆盖全程的实验。上述结果仅适用于已测工况，不是任意风扰、模型失配或实机下的跟踪保证。输出油门触限统计不等于分配器内部裁剪统计。部分运行在仿真时钟结束后出现既有内环估计器 `XTWX` 奇异异常，需同时核对电机消息覆盖时间，不能据此声称退出流程正常。
