# 解析轨迹与统一参考接口

本次按 `0913/轨迹生成/uav_trajectory_design.tex` 实现水平加速圆、竖直俯仰翻转圆、水平轴滚转螺旋，并重新实现水平八字。保留 omtraj 优化算法、可视化和 CSV 格式；删除原 minimum-snap、barrel-roll、five-turn、figure-eight 生成器、`set_2D8_ref()` 和 `LegacyTrajectoryReference`。

## 使用与参数

在 `config/params.yaml` 设置 `trajectory.type`，取值为 `horizontal_circle`、`vertical_circle`、`helix`、`figure_eight`、`omtraj`。默认是 `figure_eight`。三个控制器使用同一选择入口，不再有 CMD 轨迹编译宏。所有参数在启动时读取、生成并检查；修改后重启控制节点，活动轨迹不热切换。

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
AnalyticTrajectory / SampledTrajectory(omtraj 文件边界)
  → Trajectory::evaluate(t) → ReferencePoint
  → TrajectoryPlayer（时间、刚体对齐、预测网格）→ ReferenceWindow
  → QuadControl / OmMpcControl / AcadosNmpcControl
  → Control_Setpoint_t → RatesThrustSetpoint → 内环
```

新生成器直接输出完整 `ReferencePoint`，不创建 `OmTrajectoryResult`，不走线性缓存插值。每个控制周期按真实 ROS 时间和控制器 dt 精确求值 H+1 点。时钟回退被拒绝，随后走 FSM 既有无效参考处理。

控制器的公共入口仅接受 `ReferenceWindow` 和独立的 `ControlModeReference`。模式数据只有状态、手动态度/油门及有效标志，不再附带另一份 p/v/a。FSM 的手动/悬停内部状态和 QuadControl 私有旧控制律仍有 `Ref_State_t`，但它不再作为控制器公共入口，也不用于传递新轨迹。

`kinematics_valid` 表示 p 至 snap 来自同一可微曲线，`angular_acceleration_valid` 区分有效零与缺失角加速度。NMPC 对解析轨迹的气动补偿连同一、二阶姿态导数一起解析计算，保留倒飞姿态分支；不在默认气动补偿打开后又把解析前馈退化为窗口差分。这是基于名义姿态的一次气动力修正，并非耦合气动方程的全局求解。

omtraj 仅在文件边界转换为通用 `TimedReference`，优化器结果类型不进入 FSM 或控制器。其内部仍按离散数据插值，并从世界角速度差分补出角加速度；不虚构解析 jerk/snap。终点后也明确悬停，但 CSV 最后一个非悬停输入到 HOLD 不保证 C4；需要平滑终端时应在离线任务中约束它。

下层 `RatesThrustSetpoint` 的单位与消息布局不变：总推力 N、机体角速度 rad/s、角加速度前馈 rad/s²。角加速度按实际姿态转换到当前机体系。轨迹力矩只用于离线分配检查，不叠加到内环重复计算。

## 检查、导出与对比

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select px4ctrl --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/px4ctrl -R '^(trajectory_test|control_reference_test|controller_adapter_test|acados_nmpc_test)$' --output-on-failure
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
