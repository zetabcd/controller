# 解析轨迹与统一参考接口

本次按 `0913/轨迹生成/uav_trajectory_design.tex` 实现水平加速圆、竖直俯仰翻转圆、水平轴滚转螺旋，并重新实现水平八字。omtraj 按 `0913/wenzhang0710.docx` 第 III 节重构，使用含动力学模型的 v2 CSV；删除原 minimum-snap、barrel-roll、five-turn、figure-eight 生成器、`set_2D8_ref()` 和 `LegacyTrajectoryReference`。

## 使用与参数

AUTO_HOVER 首次起飞的目标是本次地面原点上方 0.2 m（当前值在 `takeoff_origin.h` 的 `enterHover()` 中设置）。实机在收到新鲜的未解锁状态及有效位置时更新原点，解锁后固定；因此应在地面启动控制节点、等位置有效后再解锁。原点包括动捕/PX4 的高度偏移，不要求飞机放在世界坐标零点。重新上锁、移动飞机后会更新下一次起飞原点。一次飞行中以非降落请求再次进入 AUTO_HOVER 时保持进入位置（含原有 0.3 s 速度前视），不重复上升或返回起飞点。仿真无解锁状态话题，使用首次有效位置作为原点，重启节点开始新一次实验。

这是参考位置平移，控制器反馈仍使用原世界坐标 NWU；没有重置 PX4 或动捕坐标。CMD 仍从进入 CMD 的当前位置和航向接续，`trajectory.takeoff_height` 仍是相对此处的上升量。启动日志在进入悬停时打印地面原点和目标世界坐标。

AUX2 复用起飞/降落：DOWN→MID（manual→hover）首次原地起飞；MID→UP 执行 CMD；在 CMD 中主动 UP→MID 则进入 hover 内的降落流程，固定当前位置的 x/y 和航向，直立且低速连续 0.5 s 后下降。速度上限 0.3 m/s，距起飞地面高度 0.3 m 内减至 0.1 m/s；参考最低到起飞地面高度下方 0.1 m，不会无限向下延伸。轨迹异常自动退回 hover 仍是悬停，不触发降落。降落中 AUX2 DOWN 取消到 Offboard 手动；UP 不直接重启轨迹。轨迹自然结束仍保持终点，等待 UP→MID。

自动上锁要求新鲜的 `/fmu/out/vehicle_land_detected`、接近起飞高度（±0.15 m）、低速且直立连续 0.5 s，再发送普通上锁请求（不强制上锁），由 `VehicleStatus` 确认后复位本次飞行。需要在同一平地降落，且 PX4 DDS 发布该落地检测话题；缺失或过期时只维持受限降落参考并告警，不凭高度自动停桨。仿真没有落地检测/上锁服务时也不伪造触地确认。实机原点更新在主循环解锁判断之前执行，保证未解锁时能采集地面位置。

在 `config/params.yaml` 设置 `trajectory.type`，取值为 `horizontal_circle`、`vertical_circle`、`helix`、`figure_eight`、`omtraj`。当前工作区选择为 `vertical_circle`，半径 0.9 m、进出距离各 1.35 m，以实际 YAML 为准；下表保留生成器默认参数。三个控制器使用同一选择入口，不再有 CMD 轨迹编译宏。所有参数在启动时读取、生成并检查；修改后重启控制节点，活动轨迹不热切换。

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

## 外部 GCOPTER 动捕穿缝模式（2026-10-04）

当前版本在 **CMD 中等待、接收并执行多次任务**。每段轨迹结束后仍在 CMD 悬停，
由控制台选择继续往返或结束。外部模式关闭旧无遥控流程的自动 CMD/自动执行。

### 启动与任务交互

```bash
source /opt/ros/humble/setup.bash
# 新检出时取得固定提交的 GCOPTER 子模块
# git submodule update --init 3rdpart/src/gcopter
colcon build --packages-up-to gap_planner quadsim_mujoco --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPX4CTRL_SIMULATION=ON
source install/setup.bash
ros2 launch gap_planner gap_flight.launch.py simulation:=true
```

另开终端加载同一工作区，然后运行：

```bash
source install/setup.bash
ros2 run gap_planner gap_console.py
```

| 输入 | 行为 |
|---|---|
| `c` | 仿真从稳定 AUTO_HOVER 进入 CMD 待命；实机用 AUX2 UP 进入 |
| `1`、`2`、`3` | 选择本次配置列表前 N 个窗框并规划；仍在 CMD 悬停 |
| `e` | READY 后明确执行这一段，仿真和实机均适用 |
| `r` | 沿用上次数量，再次规划；在远端时自动按相反顺序穿回 |
| `f` | 在 CMD 待命时结束任务，回 AUTO_HOVER 原地悬停，不自动降落 |
| `q` | 仅关闭控制台，不中断飞机任务 |

例如 `c → 1 → READY → e → COMPLETED → r → READY → e → COMPLETED → f`。
每次完成后也可输入新的数量，改变下一段所用窗框。执行过程中拒绝新的规划、窗框切换和
`f`；等待该段结束再决定下一步。现有遥控手动/安全/降落路径仍保留。
实机 AUX2 从 UP 到 MID 仍沿用原降落语义，持续往返时保持 UP，用控制台选择下一任务。

实机编译选项为 `-DPX4CTRL_SIMULATION=OFF`，launch 使用 `simulation:=false`；两者不一致
会拒绝启动。`/gap/enter_cmd` 只在仿真暴露；实机先由遥控器授权进入 CMD，随后每段用
`/gap/start` 或控制台 `e` 执行。`/gap/finish` 对应 `f`。
控制器种类仍由 `PX4CTRL_PRIMARY_CONTROLLER` 编译选项决定，launch 的
`controller_kind:=mpc|nmpc` 必须匹配；支持 OMMPC 和 acados。

### 固定端点与窗框选择

主配置是 `src/realflight_modules/gap_planner/config/gaps.yaml`，可用
`gap_params:=/绝对路径/gaps.yaml` 替换；控制器配置用 `controller_params:=...` 替换。
修改 YAML 后需要重启 launch，接口更新后也必须退出旧控制器/仿真/控制台进程。

```yaml
mission:
  goal_offset_x: 4.0
  flight_height: 1.0
```

控制器报告其保存的地面起飞原点 `p_takeoff`。两个固定端点分别为：

* 起点上方：`home = p_takeoff + [0, 0, flight_height]`。
* 远端：`far = home + [goal_offset_x, 0, 0]`，方向是世界 NWU 的 +X。

AUTO_HOVER 首次起飞到上述飞行高度。每次规划从当前实际悬停状态出发，选择距离当前状态
较远的那个固定端点作为目标；在 home 附近向 far 飞，在 far 附近向 home 飞。端点不随窗框
数量变化，不再停在最后一个窗框后 0.6 m。返程反转窗框顺序和穿越法线，几何航向仍采用
`planning.heading`，不要求机体先转头 180°。任意中途位置不保证能按全部所选窗框的顺序通过。

按用户要求，**场地边界不参与此模式的轨迹规划或控制器验收**，不读取 `world.lower/upper`，
不使用替代的大场地盒；地面高度也不作为外部轨迹审查下界。实际空间范围由使用者配置。
窗框碰撞约束、动力学和电机可执行性检查保留。

仿真启动时没有可见、可碰撞的窗框。选择数量后，规划器先通过 `/gap/sim/configure`
启用前 N 个窗框，收到确认后才规划；其他窗框不显示、不碰撞、不参与规划。模型内部预分配
窗框几何，切换只改变显隐与碰撞属性，不重载物理模型、不重置飞机位置。
`/gap/sim/selected_count` 发布当前启用数量，启动为 0。RViz 只显示选中的窗框。

实机数量表示本次实际布置、要穿越的前 N 个刚体，软件无法移除真实窗框；应与现场布置对应。
仅检查这些刚体的位姿新鲜度，每次规划重新读取快照。执行期间假设窗框固定；规划/READY
期间移动或过期会作废候选，执行期间失效会报告并保持执行不可变轨迹，未实现穿缝中断恢复规划。

### 刚体与机体参数

* `gate_order`：配置窗框编号。`gates.<id>.topic` 为实机 `geometry_msgs/msg/PoseStamped` 话题。
  已核对 `flight_20261001_203447_613650_bc3a4984.ulg`：原始动捕为 `/sun1/pose`，
  类型即 PoseStamped；这是飞机刚体，不能拿来表示窗框。默认窗框话题按同样的
  `/<刚体名>/pose` 形式填写为 `/gap_1/pose` 等，需改成现场真实窗框刚体名。
  `/fmu/in/vehicle_visual_odometry` 是已转为 NED/FRD、供飞控融合的飞机数据，
  不作为窗框输入，也不需要给窗框复制这一步变换。
* `width/height` 为净开口尺寸，`thickness` 为框体厚度，`frame_width` 为框条宽度，单位 m。
* `opening_offset/opening_quaternion_wxyz` 为刚体坐标到开口坐标的标定变换。
  开口 X 为有向穿越方向，Y 为宽，Z 为高。
* `sim_position/sim_rpy_deg` 为仿真刚体位姿；欧拉角组合为 Rz Ry Rx，同样应用开口偏移。
* `mocap.world_translation/world_quaternion_wxyz` 将动捕世界转换到控制器 `world_nwu`。
  日志逐样本显示飞机桥采用位置 `(x,-y,-z)` 和四元数 `(w,x,-y,-z)`，当前控制器
  输入又将它变回内部约定，因此窗框使用原始 PoseStamped 加实际标定，不重复变号。
  这不证明飞控估计器原点与动捕原点始终相同，仍需核对现场标定。
* `mocap.frame_id` 非空时必须精确匹配；默认空字符串表示锁定第一条有效消息的
  `header.frame_id`，之后所有窗框必须使用同一父坐标系（首条为空也会锁定空标签）。
  原记录器没有保存字符串 `header.frame_id`，不能从旧日志假定它叫 `mocap`。
  自动锁定只识别标签，不求解坐标标定。异常位姿、错误坐标系、零/倒序/过期/超前
  时间戳会拒收并限频告警，规划时也报告最近的拒收原因。
* `vehicle.body_model=polytope` 使用外包顶点，`ellipsoid` 保留论文 5.6 的椭球选项。
  `vehicle.vertices` 为以质心为原点、与控制器机体系一致的 `[x,y,z,x,y,z,...]` 数组，4～64 个
  非共面点，其凸包必须包住机身、桨叶、电池和动捕球，不能仅用电机中心。
* 不提供顶点时使用外包盒，半轴为 `[l+rp,l+rp,height/2]`；力臂和桨半径来自控制器 YAML，
  总高度为用户给出的 `vehicle.height=0.10` m。实际高度关于质心不对称时应提供实测顶点。
  椭球模式默认半轴 `[sqrt(2)*(l+rp),sqrt(2)*(l+rp),height/sqrt(2)]`，可用
  `vehicle.ellipsoid_half_axes` 覆盖为实测外接椭球。
* `planning.margin` 为每侧净空，`optimization_buffer` 为额外数值余量；默认分别 0.02/0.002 m。
* `planning.time_weight` 在总时间与 `∫||jerk/gravity||² dt` 之间权衡；数值越大越倾向于短时间。
  所有速度、角速度、推力等限制仍采用配置中的物理单位。
* `planning.solve_budget` 默认为 5 秒，总任务预算；最后一次独立审查可能带来少量额外耗时。
  超时且没有通过审查的轨迹不会发送给控制器。
* 旧的 `tunnel_length/path_length_weight/backward_weight/penalty` 已退出问题构造；旧配置仍包含
  它们时节点会明确警告并忽略。窗框位置、尺寸、机体顶点和物理限制无需因此重调。

### 稀疏 MINCO 构造与失败诊断

底层直接调用固定提交 `e0444f6d47b84f972ced91746b05feb36ce1fd4f` 的 GCOPTER
`MINCO_S3NU` 和 L-BFGS。与原来的“自由点 + 人工走廊罚函数”不同：

1. 每个缝对应一个**窗平面内的航点**，只优化局部 Y/Z 两个坐标，法向坐标通过参数化严格消元。
   起终点为悬停 P/V/A 边界。相邻任务航点之间仅插入两个自由过渡点，初值沿窗法线进出。
   N 个缝共 `3*(N+1)` 个五次多项式段，`11*N+9` 个决策变量；增加检查采样点不会增加决策变量。
2. 所有多项式系数由 MINCO 带状系统求解，梯度由伴随系统回传到航点与正时间变量。
   不独立优化多项式系数，也不在所有段强加全局 X 单调性或场地盒。
3. 穿窗瞬间检查机体凸包投影能否装进开口，并要求正向穿越；全程碰撞检查的是**四根有限长的
   有厚度框条**，与 MuJoCo 中的几何一致。没有无限墙面，也没有长隧道。
   凸多面体使用面法向和边叉积的完整分离轴集合，顶点支持函数通过平坦映射影响姿态。
   椭球使用真实椭球支持函数和有限候选分离轴，是保守净空证书，可能拒绝部分可行姿态。
4. 代价为归一化 jerk、时间及归一化的平滑约束罚函数。先求一次，再进行 1 ms 稠密独立审查。
   若薄框附近漏采样，把最小净空位置反馈给优化器局部补点并热启动；仅失败时做可行性恢复。
   不再无条件跑四级完整优化，也不随重试把全部段的积分点翻倍。
5. 规划与控制器共用同一空气动力学参考模型；局部自动微分、碰撞梯度和完整 MINCO 目标梯度
   均有数值差分测试。静态单电机推力与角加速度约束使用控制器的惯量、机臂和电机参数，
   由 launch 从同一 controller YAML 派生，无需在窗框 YAML 中重复配置。只有电机约束激活时
   才计算含 snap 的 12 维局部梯度；其他样点使用 9 维梯度。独立验收还直接复用控制器
   `auditTrajectory`，避免“规划成功、控制器拒收”的物理模型缺口。无论 L-BFGS 返回收敛、迭代上限还是线搜索退出，只有独立审查通过才可上传。

这解决了旧构造中的假冲突：例如 X 相隔 0.1 m、上下相隔 1.5 m 的两个有限窗框，并不构成
两面相隔 0.1 m 的无限墙。优化器仍是局部非凸求解器，失败应读作“预算内没找到通过审查的
轨迹”，不能直接证明物理上无解。当前穿窗瞬间的全机体投影约束也比只要求截面穿过更保守。

设计参考：[MINCO/GCOPTER 论文与实现](https://github.com/ZJU-FAST-Lab/GCOPTER)、
[Fast-Racing](https://github.com/ZJU-FAST-Lab/Fast-Racing) 的机体顶点几何表示，以及
[Fast-Perching 实现](https://github.com/ZJU-FAST-Lab/Fast-Perching/blob/master/src/traj_opt/src/traj_opt_perching.cc)
的任务几何参数化、时间变换与事后可行性检查。这里的有限窗框和自适应采样是本项目的实现，
不是声称原样复现 Fast-Racing，也没有 GPU 后端。

离线复现**与启动相同的 YAML**（无需启动 ROS 或仿真）：

```bash
source install/setup.bash
ros2 run gap_planner gap_plan_check --yaml \
  src/realflight_modules/gap_planner/config/gaps.yaml 3 outbound \
  src/realflight_modules/px4ctrl/config/params.yaml
# outbound 换成 return 可检查返程；默认起飞位置与 launch 的 [-1.65,0,0.05] 一致。
```

每次成功或失败，在 `/gap/planner_status`、节点日志和控制台输出墙钟计时：

| 字段 | 含义 |
|---|---|
| `setup_ms` | 几何和求解器初始化 |
| `optimize_ms`、`stages_ms` | 总优化耗时及初次求解/恢复求解耗时 |
| `audit_ms`、`total_ms` | 独立稠密审查、规划线程总耗时；不含传输和控制器验收 |
| `pieces/variables` | 多项式段数、稀疏决策变量数 |
| `evaluations/iterations` | 目标与梯度计算次数、L-BFGS 迭代次数 |
| `lbfgs_status/phase` | 各次退出码及完成/失败阶段；退出码不替代可行性检查 |
| `retry[i]` | 本次补采样/恢复的原因，含最小净空的窗框与时间、动力学峰值 |
| `controller_validation_ms` | 控制器重建与独立验收耗时 |
| `upload_ack_ms` | 发送器上传服务往返时间；ACK 不代表通过验收 |
| `flight_duration` | 飞行时间，区别于求解时间 |
| `path_length/endpoint_distance` | 轨迹弧长与起终点直线距离 |
| `backward_distance` | 沿本段起终点方向累计后退距离；它是统计量，不是约束 |
| `crossing_roll_deg/crossing_tilt_deg` | 计划穿窗时刻的滚转角与总倾角 |

### FSM 与数据流程

```text
AUTO_HOVER（起飞）
  → AUX2 UP / 仿真 c
CMD / WAITING（持续悬停）
  → 数量：配置本次场景 → PLANNING → VALIDATING → READY（仍悬停）
  → e：EXECUTING
  → COMPLETED（仍在 CMD 悬停）
      → r 或新数量：下一段规划，通常反向返回
      → f：AUTO_HOVER 原地悬停
```

AUTO_HOVER 不再接收轨迹。到达 READY 不自动飞行，提前 `e` 被拒绝且不锁存。
验证在独立线程中执行，离开 CMD 或结束任务会丢弃待执行数据及过期验证结果。
开始执行前检查当前反馈、场景、起点位置/速度/姿态，防止候选等待后已失效。

`/gap/polynomial → gap_trajectory_sender → /gap/upload → ExternalTrajectory → TrajectoryPlayer`
构成数据链。发送器默认 5 ms 采样并保留所有分段边界，一次可靠服务上传有界数组；
控制器从 p/v/a 五次 Hermite 重建原 S3 位置曲线，按自己的时钟执行，网络抖动不直接改变采样节奏。
模型、参考坐标、端点静止、动力学及静态电机分配均做检查。S3 首末 jerk 没有强制为零，
完整姿态参考进入/退出的平滑性仍不能称作 C4。离散审查和仿真通过不构成连续时间或实机安全证明。

### 验证记录与复现

本轮保留用户当前场景：第一缝中心 `[0,0,1]`、滚转 50°、开口 0.50×0.25 m；第二缝
中心 `[0.1,0,2.5]`；第三缝 `[1.1,0,1]`。起终点世界 X 相隔 4 m。
旧实现直接读取这份 YAML 时，1/2/3 缝均失败，分别约 0.63/2.55/2.30 秒。

新实现对当前场景、悬停位置扰动 `[8,9,-3]` mm、窗框侧移 10 mm/滚转扰动 1°、以及
原同高 30° 场景，分别测试 1/2/3 缝去返程，**24/24 通过**，包含电机执行性审查。
一次串行运行中，当前场景的离线优化耗时为：

| 缝数 | 去程 | 返程 | 去程路径长度 |
|---|---:|---:|---:|
| 1 | 425 ms | 466 ms | 4.37 m |
| 2 | 1126 ms | 840 ms | 8.17 m |
| 3 | 1760 ms | 2103 ms | 8.87 m |

多缝路线必须升到第二缝 2.5 m 再下降，且几乎同 X 的两个窗都要求正向穿越，因此仍需转弯和
部分 X 回退；没有人为的“只许向前”约束。对原同高 30° 场景，三缝路径约 4.12 m、累计回退
约为数值零。以上是当前机器单次结果，不是全局最优或实时性能保证。

完整 MuJoCo 闭环的三缝去程/改选两缝返程均通过：优化约 1798/719 ms，端点误差
2/9 mm；从实际位置插值确认按顺序、正确方向穿过**有限开口**，未检测到窗框接触。
一缝往返也已通过。检查覆盖启动窗框数 0、AUTO_HOVER 拒绝规划、CMD 待命、READY 等待 `e`、
执行时拒绝重规划和改场景、完成后 CMD 等待、以及 `f` 回 AUTO_HOVER。
实机尚未飞行验证。启动握手偶发 DDS response timeout 的一次测试在规划前超时，换隔离域重试通过；
仿真结束时仍可能打印现有 SIGINT/KeyboardInterrupt 退出信息。

```bash
ctest --test-dir build/gap_planner -R gap_geometry_test --output-on-failure
ctest --test-dir build/px4ctrl \
  -R '^(external_trajectory_test|trajectory_test|control_reference_test|takeoff_origin_test|landing_reference_test)$' \
  --output-on-failure
python3 src/realflight_modules/gap_planner/test/simulation_flow.py --count 1 --return-count 1
python3 src/realflight_modules/gap_planner/test/simulation_flow.py --count 3 --return-count 2
```

离线回归（读取 YAML，不启动 ROS）：

```bash
python3 src/realflight_modules/gap_planner/test/planning_regression.py
```

结果默认写入 `/tmp/gap_planning_regression.json`，保留每个场景的完整成功/失败输出。
`gap_plan_check --yaml gaps.yaml count outbound|return controller.yaml [dx dy dz]` 的最后三个可选
参数用于扰动实际悬停起点（m），固定目标不变。离线 controller YAML 分支默认采用 `mpc` 参数；
`launch` 的 `controller_kind` 则支持选择 `mpc/nmpc`。旧位置参数命令仍保留用于简单合成场景。
