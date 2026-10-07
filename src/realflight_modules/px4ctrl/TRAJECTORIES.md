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

上述解析/omtraj 轨迹在局部原点生成。进入 CMD 时整体平移到实测位置、绕 z 轴旋转到实测航向；不使用经纬度或 NED 绝对航点。下述 `obstacle_trigger` 仅平移位置原点，移动轴固定在世界 X 方向。生成器假设激活时近似悬停，不从当前非零速度重新规划入口。既有 FSM 的模式切换/异常恢复机制未被替换为特技恢复规划器。

## 障碍物接近触发平移：obstacle_trigger

普通 `run_ctrl.launch.py` 入口使用同一套 FSM。将 `trajectory.type` 设为
`obstacle_trigger`：进入 CMD 后按公共 `takeoff_height/takeoff_duration` 上升，
完成 `settle_duration` 后一直保持目标点。新鲜的障碍物位置与飞机 PX4 当前位置的
**三维欧氏距离严格小于** `trigger_distance` 时，执行一次世界 X 轴平移，随后保持终点。
距离是两个位置原点之间的距离，不是到障碍物表面的净距离。

```yaml
trajectory:
  type: obstacle_trigger
  takeoff_height: 0.5
  takeoff_duration: 3.0
  settle_duration: 0.5
  obstacle_trigger:
    topic: /obs/pose
    frame_id: ""
    trigger_distance: 1.0
    move_distance: 1.0
    move_duration: 3.0
    pose_timeout: 0.2
```

`topic` 接收 `geometry_msgs/msg/PoseStamped`。`trigger_distance`、`move_distance`
单位为米，时间参数为秒。正/负 `move_distance` 分别沿世界 +X/−X；以等待悬停目标
为平移起点，目标 Y/Z 和进入 CMD 时的航向保持不变，方向不跟随飞机航向旋转。
移动沿用九次多项式 C4 连接和普通轨迹名义执行器检查，`move_duration` 决定平移速度。
上述数值是可修改的默认值，控制节点启动时读取。

障碍物话题必须已处于控制器世界 NWU 坐标系；不要再次施加飞机 PX4 消息的
`[x,-y,-z]` 转换。`frame_id` 非空时检查完全匹配，空时学习并锁定第一条有效消息
的标签；标签检查不执行坐标变换。消息时间戳须与节点 ROS 时钟同步。缺失、过期、
未来、乱序、非有限位置或坐标系不符的数据不触发移动。订阅只保留最新一条消息，
每次进入 CMD 清除旧观测，起飞阶段的接近事件不缓存为待执行动作。

每次进入 CMD 只触发一次；开始移动后不因障碍物离开或数据断流而中途切换轨迹，
到终点后也不重复触发。重新进入 CMD 会复位。退出 CMD、遥控接管、异常悬停与
UP→MID 降落继续使用原有逻辑。该模式由距离触发固定平移，不做障碍物外形或
平移路径碰撞规划。`gap_flight.launch.py` 会强制 `external`，不适用于此模式。

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
由控制台选择数量、重新规划和明确执行。进入/退出 CMD 恢复原 AUX2 流程：无遥控仿真保留原有合成 AUX2 自动切档，实物使用遥控器；控制台不切换 FSM 模式。进入 CMD 不自动执行轨迹。

### 启动与任务交互

```bash
source /opt/ros/humble/setup.bash
# 新检出时取得固定提交的 GCOPTER 子模块
# git submodule update --init 3rdpart/src/gcopter
colcon build --packages-up-to gap_planner quadsim_mujoco --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch gap_planner gap_flight.launch.py
```

另开终端加载同一工作区，然后运行：

```bash
source install/setup.bash
ros2 run gap_planner gap_console.py
```

| 输入 | 行为 |
|---|---|
| `1`、`2`、`3` | 选择本次配置列表前 N 个窗框并规划；仍在 CMD 悬停 |
| `e` | READY 后明确执行这一段，仿真和实机均适用 |
| `r` | 沿用上次数量，再次规划；在远端时自动按相反顺序穿回 |
| `q` | 仅关闭控制台，不中断飞机任务 |

例如 `原 AUX2 流程进入 CMD → 1 → READY → e → COMPLETED → r → READY → e → COMPLETED`。
每次完成后也可输入新的数量，改变下一段所用窗框。执行过程中拒绝新的规划和窗框切换；等待该段结束再决定下一步。遥控手动/安全/降落仍走原 FSM 路径。
控制台可在 CMD 前或后启动；`/gap/execution` 持续约 10 Hz 发布，晚启动不会错过一次性的切换事件。
不再提供 `c`、`f` 或对应的进入/退出模式服务。`q` 只关闭控制台，不停止飞行。
实机 AUX2 从 UP 到 MID 仍沿用原降落语义，持续往返时保持 UP，用控制台选择下一任务。

实机将 `input.h` 中 `PX4CTRL_SIMULATION` 改为 `0` 并重新编译，将
`gap_flight.launch.py` 顶部 `SIMULATION` 改为 `False`；两者不一致会拒绝启动。实机先由遥控器 AUX2 UP 进入 CMD，仿真由原无遥控逻辑合成 AUX2 UP；随后每段都用 `/gap/start` 或控制台 `e` 执行。
控制器种类仍由 `PX4CTRL_PRIMARY_CONTROLLER` 编译选项决定，launch 的
`CONTROLLER_KIND = 'mpc'` 或 `'nmpc'` 必须匹配；支持 OMMPC 和 acados。

### 穿缝配置入口与覆盖关系

模式开关和启动配置已集中整理；FSM 的模式触发恢复原 AUX2 路径，控制台只管理轨迹任务。MINCO、消息格式和场景新鲜度检查保持原实现。
穿缝只启动 `gap_flight.launch.py`，它已包含两级控制器、规划器、轨迹发送器，以及按开关启动的 MuJoCo 和 RViz。
不要再同时启动 `run_ctrl.launch.py`，否则会重复启动控制节点。

| 修改位置 | 参数与作用 | 生效方式 |
|---|---|---|
| `px4ctrl/include/px4ctrl/input.h` | `PX4CTRL_SIMULATION=1` 仿真；`0` 实机。选择传感器处理分支，同时启用/关闭无遥控仿真流程 | 重新编译 px4ctrl 并重启 |
| `px4ctrl/include/px4ctrl/px4ctrlfsm.h` | `PX4CTRL_PRIMARY_CONTROLLER=1` OMMPC；`2` acados；`0` 原 QuadControl | 重新编译；当前穿缝模型导入按 OMMPC/acados 配置 |
| `gap_flight.launch.py` 顶部 | 以下启动配置 | 重启 launch；非 symlink 安装需重新 build 安装 |
| 选中的控制器 YAML | 原有质量、惯量、电机、气动、控制增益、MPC 和轨迹校验参数 | 重启 |
| 选中的 gaps YAML | 任务端点、窗框几何、动捕、规划参数、采样间隔 | 重启 |

源码模式与 launch 模式是两个明确入口：前者决定编译进去的控制逻辑，后者决定启动哪些节点、窗框数据来源及 ROS 时钟。
launch 不读取头文件来猜模式，也不会通过参数改变已编译的模式。两处必须一致。
以前缓存中的 `PX4CTRL_SIMULATION:BOOL=ON/OFF` 即使还在 CMakeCache.txt，也不再参与编译定义；不用清空整个 build。
以后构建不传 `-DPX4CTRL_SIMULATION`，直接使用上面的构建命令或 `colcon build` 即可。

| launch 顶部变量（当前默认） | 含义 |
|---|---|
| `SIMULATION = True` | 启动 MuJoCo；规划器使用 YAML 框位姿；控制器、规划器、发送器、RViz 统一使用 `/clock`。`False` 时不启动 MuJoCo，规划器订阅框刚体，所有上述节点使用系统时间 |
| `HEADLESS = False` | 是否隐藏 MuJoCo 窗口；True 仍运行仿真，仅在仿真时有效 |
| `RVIZ = True` | 是否显示 RViz |
| `CONTROLLER_KIND = 'mpc'` | 为规划器导入控制器模型时选择 YAML 的 `mpc`（OMMPC）或 `nmpc`（acados）节；不切换控制算法 |
| `CONTROLLER_PARAMS = 'params.yaml'` | px4ctrl/config 下的控制器配置；可以改为 `params_ommpc_flip.yaml` 或绝对路径 |
| `GAP_PARAMS = 'gaps.yaml'` | gap_planner/config 下的任务配置；也可填写绝对路径 |
| `RATECTRL_PARAMS = 'ratectrl_diagnostics.yaml'` | 内环诊断配置；主要飞行器/控制参数仍由内环通过参数服务从外环获取 |
| `SIM_PARAMS = 'params.yaml'` | quadsim_mujoco/config 下的仿真器配置，与控制器的同名文件是不同文件 |
| `RVIZ_CONFIG = 'gaps.rviz'` | gap_planner/config 下的可视化配置 |
| `SIM_INITIAL_POSITION = [-1.65, 0.0, 0.05]` | 仿真机体初始世界 NWU 位置，单位 m |
| `SIM_LOG_ROOT = 'datalog/gap_sim'` | 仿真记录根目录，每次创建时间命名的子目录 |

配置优先级是**节点默认值 < YAML < launch 后置字典**。后置覆盖已在 launch 中明列为
`controller_overrides`、`clock_parameters`、`model` 和 MuJoCo 参数字典：

| 接收节点 | launch 覆盖 YAML 的字段 | 来源/目的 |
|---|---|---|
| 外环 px4ctrl_node | `trajectory.type = external` | 穿缝入口固定选择外部轨迹；即使控制器 YAML 写 helix/omtraj，也不会执行它 |
| 外环 | `trajectory.external.simulation`、`use_sim_time` | 顶部 `SIMULATION`；前者检查与已编译模式是否匹配，后者选择时间来源 |
| 内环、发送器、RViz | `use_sim_time` | 顶部 `SIMULATION` |
| 规划器 | `simulation`、`use_sim_time` | 顶部 `SIMULATION`；因此统一 launch 下仅改 gaps.yaml 的 simulation 无效 |
| 规划器 | 下表的 `vehicle.*` 模型参数 | 从所选控制器 YAML 导出，确保规划与执行使用一致模型 |
| MuJoCo | `initial_position`、`headless`、`log_directory` | 顶部仿真配置；覆盖仿真器 YAML 同名项 |
| MuJoCo | `gap_scene_file` | 使用与规划器相同的 gaps 文件；启动时无激活窗框，选择数量后构建本次场景 |

模型导入具体关系如下；这些都是原版已有的计算，此次没有新增约束：

| 规划器字段 | 控制器 YAML 来源 |
|---|---|
| `vehicle.mass / arm / prop_radius / gravity` | `uav.mass / uav.l / uav.rp / gra` |
| `vehicle.drag_acceleration` | `[aero.kdx, kdy, kdz] / mass`；所选控制器 `drag_compensation=false` 时置零 |
| `vehicle.lift_acceleration` | `aero.kh / mass`；同样受气动补偿开关控制 |
| `vehicle.inertia / arm_angle` | `uav.Jvx/Jvy/Jvz`；`uav.beta_deg` 转弧度 |
| `vehicle.torque_to_thrust` | `2 * rp * motor.Cq_c / motor.Ct_c` |
| `vehicle.motor_min` | `ct * motor.rc2speed_c²`，其中 `ct = motor.Ct_c * 4 * aero.rho * rp⁴ / π²` |
| `vehicle.motor_max` | `ct * (rc2speed_a + rc2speed_b + rc2speed_c)² * trajectory.limits.motor_fraction` |
| `vehicle.execution_rate_max` | 所选 `mpc/nmpc.body_rate_max`，逐轴 rad/s |
| `vehicle.angular_acceleration_max` | `trajectory.limits.angular_acceleration`，rad/s² |

`gaps.yaml` 中仍直接由用户配置的项目如下：

| 参数 | 含义 |
|---|---|
| `simulation` | 单独运行规划器时的默认模式；统一启动时被 launch 覆盖 |
| `mission.goal_offset_x` | 去程终点相对本轮首次接受规划时实际悬停点的世界 X 偏移，默认 4 m；返程回该悬停点 |
| `vehicle.height` | 含桨、电池和动捕球的机体包络总高，当前 0.10 m |
| `vehicle.body_model` | `polytope` 顶点凸包或 `ellipsoid` 外接椭球 |
| `vehicle.vertices` | 可选机体系包络顶点 `[x,y,z,...]`，m；未填时按力臂、桨半径、高度生成盒子八顶点 |
| `vehicle.ellipsoid_half_axes` | 可选椭球三半轴，m；仅椭球模型使用 |
| `planning.heading` | 世界 NWU 中机体 X 方向的航向，rad |
| `planning.margin / optimization_buffer` | 碰撞净空 / 优化额外数值余量，m，默认 0.02 / 0.002 |
| `planning.speed_max / rate_max` | 优化与审计使用的速度、角速度限值，m/s、rad/s |
| `planning.thrust_min / thrust_max` | 单位质量推力范围，m/s²，不是 N |
| `planning.tilt_max` | 倾角上限，rad |
| `planning.time_weight` | 时间成本权重，与归一化 jerk 能量权衡 |
| `planning.solve_budget` | 规划求解总预算，实际墙上耗时的秒数；审计可增加少量开销 |
| `mocap.frame_id` | 要求的输入父坐标系；空串表示学习并锁定首个有效消息的 frame_id（它本身也可能为空） |
| `mocap.timeout` | 实物框位姿新鲜度，默认 0.3 s |
| `mocap.position_tolerance / angle_tolerance` | 对比本次规划快照，框移动超过 0.01 m / 0.02 rad 则场景失效 |
| `mocap.world_translation / world_quaternion_wxyz` | 动捕父系到控制器世界 NWU 系的固定变换；平移 m，四元数顺序 wxyz |
| `gate_order` | 输入 N 时选择列表前 N 个框；返程倒序穿越，每次请求重新取当前框位姿 |
| `gates.<id>.topic` | 该框刚体的 `geometry_msgs/msg/PoseStamped` 话题，不能填飞机刚体的话题 |
| `width / height / thickness / frame_width` | 净开口宽、高，沿穿越轴的厚度，框条宽度，单位 m |
| `opening_offset / opening_quaternion_wxyz` | 刚体系中的开口中心及姿态；开口 X 为穿越方向，Y 为宽，Z 为高 |
| `sim_position / sim_rpy_deg` | 仿真刚体在世界系的位置和姿态；旋转使用 `Rz(yaw) Ry(pitch) Rx(roll)`，之后再复合开口变换 |
| `gap_trajectory_sender.ros__parameters.sample_dt` | 离散采样间隔，默认 0.005 s；整条轨迹一次上传并缓存，不是每 5 ms 发一条 ROS 消息 |

可在控制器 YAML 的 `trajectory.external` 下配置启动容差：`start_position_tolerance` 默认 0.10 m，
`start_speed_tolerance` 默认 0.15 m/s，`start_attitude_tolerance` 默认 0.15 rad，`scene_timeout` 默认 0.5 s。
这些是启动检查，不加入 MINCO 优化。`trajectory.limits` 和控制器模型另用于轨迹动力学校验。
场地长宽没有作为优化边界加入。YAML 参数在构造时读取，修改后应重启，不应假定 `ros2 param set` 会重建内部模型。

### 时间检查为何会立即拒绝

这里说的是**时间来源不一致**，不是 MINCO 优化耗时。`use_sim_time=false` 时 ROS `now()` 是系统时间，
`true` 时取仿真器发布的 `/clock`（通常从接近 0 开始）。例如控制器当前系统时间是约 18 亿秒，
规划器场景消息时间戳是仿真第 12 秒，二者相减会远大于 0.5 秒；消息刚到也会被误判过期。

规划器每 100 ms 的墙上定时回调发布 `/gap/scene`，其中 stamp 使用规划器的 ROS 时钟。
控制器要求场景有效、scene_id 一致，且：

- 收到最后一条已接受场景消息后，控制器 ROS 时间过去不超过 `scene_timeout=0.5 s`。
- 消息 stamp 相对控制器当前 ROS 时间，不能旧于 0.5 s，也不能超前超过 0.05 s。
- 实机规划器另外检查窗框 PoseStamped 的接收时间和 stamp，不得旧于 `mocap.timeout=0.3 s`，stamp 不得超前超过 0.05 s。

`Scene moved, expired or unavailable; replan` 是这些场景检查的共用提示，仅凭此文本无法确定是时钟、消息中断还是框移动。
统一 launch 原来就将控制器和规划器的 `use_sim_time` 设为同一个 simulation 值，正常情况下不会混用时钟。
后来拆分启动，`run_ctrl.launch.py` 默认 `use_sim_time=false`，而规划器可能使用仿真时钟，这条路径能解释立即拒绝；
要证明某次运行的唯一原因仍需要该次节点参数/日志。更早的本地解析轨迹也没有跨节点的场景心跳比较。

此次保留原统一启动和时间检查，不放宽超时。模式为 True 时所有消费时间的节点用 `/clock`，False 时都用系统时间。
实物动捕如果运行在其他电脑，发布的 ROS stamp 与控制电脑系统时间仍需同步；不要把设备自身启动秒数直接填成 ROS 系统时间。
在当前实现中，框失效、场景消息延迟/丢失或时钟重置仍可能触发拒绝，这与参数入口简化是不同问题。
优化日志 `setup_ms/optimize_ms/audit_ms/total_ms` 使用单调时钟统计实际计算耗时，不受 `use_sim_time` 控制。

### 固定端点与窗框选择

主配置是 `src/realflight_modules/gap_planner/config/gaps.yaml`，在 launch 顶部用
`GAP_PARAMS` 选择；控制器配置用 `CONTROLLER_PARAMS` 选择。均支持配置文件名或绝对路径。
修改 YAML 后需要重启 launch，接口更新后也必须退出旧控制器/仿真/控制台进程。

```yaml
mission:
  goal_offset_x: 4.0
```

规划器在本次 CMD 内首次接受规划请求时保存实际悬停位置：

* `home = 首次接受规划时 /gap/execution.pose.position`。
* `far = home + [goal_offset_x, 0, 0]`，方向是世界 NWU 的 +X。

本轮往返和重新规划共用固定的 home/far；每段规划起点仍是当时的实际位置。
退出 CMD 或时钟回退清除 home，下次任务重新记录。规划器不读取起飞高度，也不使用地面原点生成端点。

external 的 AUTO_HOVER 与其他模式一样，首次只给地面原点上方 **0.2 m** 的固定位置目标。
进入 CMD 后才执行前置升高轨迹：从进入时的实际位置和速度出发，保持进入时的 XY 目标和航向，
用与普通解析轨迹共用的九次多项式连接段到达 `p_cmd + [0, 0, trajectory.takeoff_height]`。
高度、执行时间和稳定等待统一使用控制器 YAML 的 `trajectory.takeoff_height`、
`trajectory.takeoff_duration`、`trajectory.settle_duration`，run_ctrl 与 gap_flight 入口一致。
例如 HOVER 在地面上方约 0.2 m，takeoff_height=1.0 时，CMD 前置终点约为地面上方 1.2 m。
gap_flight 不再覆盖高度；旧的 `mission.flight_height` 和 `trajectory.external.takeoff_height` 已移除。
前置轨迹结束后持续保持固定终点，位置误差、速度和姿态满足现有 external 启动容差，并连续稳定
`trajectory.settle_duration`（默认 0.5 s），才进入 WAITING。PREPARING 期间 `hovering=false`，
规划、上传和执行请求被拒绝且不缓存。退出 CMD 丢弃活动和待执行轨迹；每次重入 CMD 都从当时实际状态
重新执行相对升高，不再按旧的绝对高度跳过。takeoff_height=0 时仍执行速度收敛和稳定等待。
CMD 内连续上传执行多段轨迹不会重复前置升高。前置与上传轨迹共用活动轨迹、时钟及播放器，
只有完成后的状态不同（WAITING / COMPLETED）。

每次规划从当前实际悬停状态出发，选择距离当前状态
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
# outbound 换成 return 可检查返程。
# 离线工具没有实时飞机位置；默认测试 home=[-1.65,0,1.05]，不是推算起飞高度。
# 可在控制器 YAML 后追加：起点扰动 dx dy dz，再追加本轮 home 的 x y z。
# 例如追加 0 0 0 -1.65 0 1.85，使用该悬停点测试往返。
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
AUTO_HOVER（0.2 m 起飞）
  → AUX2 UP（实机遥控器 / 无遥控仿真原有自动切档）
CMD / PREPARING（按时间升至任务高度，等待稳定）
  → 前置完成
CMD / WAITING（持续悬停）
  → 数量：配置本次场景 → PLANNING → VALIDATING → READY（仍悬停）
  → e：EXECUTING
  → COMPLETED（仍在 CMD 悬停）
      → r 或新数量：下一段规划，通常反向返回
      → 模式退出/降落：原 AUX2 路径，控制台不切模式
```

AUTO_HOVER 不再接收轨迹。到达 READY 不自动飞行，提前 `e` 被拒绝且不锁存。
验证在独立线程中执行，离开 CMD 会丢弃待执行数据及过期验证结果。
开始执行前检查当前反馈、场景、起点位置/速度/姿态，防止候选等待后已失效。

`/gap/polynomial → gap_trajectory_sender → /gap/upload → ExternalTrajectory → TrajectoryPlayer`
构成数据链。发送器默认 5 ms 采样并保留所有分段边界，一次可靠服务上传有界数组；
控制器从 p/v/a 五次 Hermite 重建原 S3 位置曲线，按自己的时钟执行，网络抖动不直接改变采样节奏。
模型、参考坐标、端点静止、动力学及静态电机分配均做检查。S3 首末 jerk 没有强制为零，
完整姿态参考进入/退出的平滑性仍不能称作 C4。离散审查和仿真通过不构成连续时间或实机安全证明。

### 验证记录与复现

恢复 AUX2 触发后的验证：7 项控制器/规划器相关 CTest 通过；独立 ROS 域、无窗口 MuJoCo
临时单框 30° 场景完成去程和返程，终点误差约 3/8 mm。启动时没有控制台模式请求，
原合成 AUX2 自动进入 CMD 并保持 WAITING；新建晚订阅者收到当前状态，两个模式切换服务均不存在。
选框前无活动窗框、READY 不自动飞行、两段均需明确启动、完成后持续保持 CMD 均已检查。
此测试未修改主 gaps.yaml，不代表当前主场景或实物已经完成同等验证。

以下为此前优化器版本的历史实验记录，场景值和模式接口以当时版本为准：

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
执行时拒绝重规划和改场景、完成后 CMD 等待。以上为此前版本验证记录；其中旧版曾通过控制台进入/退出 CMD，当前已移除这两个服务，模式触发恢复 AUX2。
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
`launch` 顶部的 `CONTROLLER_KIND` 则支持选择 `mpc/nmpc`。旧位置参数命令仍保留用于简单合成场景。
