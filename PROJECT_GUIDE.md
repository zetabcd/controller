> OMMPC 分支更新（2026-09-29）：控制器 1 已扩展角速度/推力响应、气动补偿和 OSQP 复用。
> 默认参数与仿真启动方式见 [OMMPC.md](src/realflight_modules/px4ctrl/OMMPC.md)。
> 仿真建议使用 `ros2 launch px4ctrl run_sim.launch.py`，统一 `/clock`；下文旧 OMMPC 参数描述以新说明为准。

**工程模块与使用手册**

轨迹与接口重构更新：2026-09-29；其余内容核对日期：2026-09-28。基于 `trajectory-tracking` 分支、提交 `3a78c7f` 的源码、配置、launch 和本地 Git 历史整理。文中的“当前”指源码配置，不代表已经核实正在运行的节点或旧的 install 构建产物。本次整理没有启动飞行、仿真或修改控制参数。

**1. 工程组成与数据流**

当前分支有 9 个 ROS 包，以及独立 Python 分析脚本、第三方数值库和实机启动脚本。

| 模块 | 位置 | 功能与入口 |
|---|---|---|
| 自研飞行控制 | `src/realflight_modules/px4ctrl` | FSM、3 种顶层控制器、角速度内环、电机分配；`run_ctrl.launch.py` |
| 轨迹生成 | `px4ctrl` 内的 `trajectory.*`、`omtraj.*` | 4 种解析 CMD 轨迹及 omtraj，所有控制器共用；离线优化入口 `omtraj_visualizer.launch.py` |
| PX4 原生位置任务 | `px4ctrl` 内的 `px4_native_position_*` | 向 PX4 发位置航点，由 PX4 自己完成闭环控制 |
| MuJoCo 仿真 | `src/uav_simulator/quadsim_mujoco` | 刚体、电机动态、桨与气动力、风场和传感器噪声；`mujoco.launch.py` |
| ULog 记录 | `src/realflight_modules/flight_data_recorder` | ROS 数值话题逐消息记录；`record.launch.py` |
| 离线飞行分析 | `script/flight_analysis` | 跟踪指标、控制诊断、PDF、GIF、多次实验对比 |
| 旧 CSV 噪声工具 | `script/noise_analys.py` | 三轴噪声均值、方差和波形，需手动配置输入 |
| 实时可视化 | `px4ctrl` 内的 `realflight_trajectory_visualizer_node.cpp` | 实际与参考轨迹、姿态显示 |
| 实机通信辅助 | `script/start_onboard_terminals.sh`、`firmware/README.md` | DDS Agent、动捕桥接启动；AUX5 飞控重启与恢复监测 |
| 消息和工具 | `src/utils` 下的 6 个包 | PX4 消息、调试消息、内环指令、遥控消息、状态机及数学/路径工具 |
| 数值依赖 | `3rdpart` | Eigen、OSQP、acados；CasADi Python 仅用于模型生成 |
| MINCO（未合入） | 仅 `minco` 分支 | 独立算法库、示例与可视化，不在当前源码树中 |

自研控制链路：

```text
FSM / 轨迹生成 → 顶层控制器 px4ctrl_node
                         ↓ /rates_thrust_setpoint
               角速度内环 px4ctrlrate_node
                         ↓ /fmu/in/actuator_motors
                   MuJoCo 或 PX4 实机
                         ↓ 位置、姿态、IMU
                    返回两个控制节点

各节点话题 → flight_data_recorder → .ulg → analyze_flight_log.py
轨迹/状态话题 → 实时 RViz 可视化
```

PX4 原生位置任务是另一条链路：`px4_native_position_node → trajectory_setpoint → PX4 原生控制器`。不要同时启动它与自研控制 launch 向同一台飞行器发控制命令。MuJoCo 当前接收电机指令并依赖 `px4ctrl_node` 握手，不是 PX4 SITL，也不能直接替代原生位置任务所需的 PX4。

**2. 当前源码实际启用了什么**

| 项目 | 当前值 | 位置 |
|---|---|---|
| 顶层控制器 | `PX4CTRL_PRIMARY_CONTROLLER=1`，OMMPC | [px4ctrlfsm.h](src/realflight_modules/px4ctrl/include/px4ctrl/px4ctrlfsm.h) |
| CMD 轨迹 | `trajectory.type=figure_eight`，八字专项参数；翻转配置保存在 `params_ommpc_flip.yaml` | `params.yaml` |
| 仿真编译模式 | `SIMULATION` 已定义 | [input.h](src/realflight_modules/px4ctrl/include/px4ctrl/input.h) |
| 无遥控自动流程 | `USE_WITHOUT_RC` 已定义 | 同上 |
| IMU 来源 | `PX4CTRL_USE_FILTERED_IMU=0`，sensor_combined | 同上 |
| 外环 / 内环频率 | 100 / 400 Hz | [控制参数](src/realflight_modules/px4ctrl/config/params.yaml) |
| 仿真频率 | 400 Hz | [仿真参数](src/uav_simulator/quadsim_mujoco/config/params.yaml) |
| 噪声 | 陀螺、加速度噪声开启；风场关闭 | 同上 |
| 日志启动方式 | `simulation_mode: true`，启动即记录 | [记录配置](src/realflight_modules/flight_data_recorder/config/recorder.yaml) |
| 原生位置任务 | `simulation.enabled: true`，定位就绪后自动发起任务 | [原生任务配置](src/realflight_modules/px4ctrl/config/px4_native_position_mission.yaml) |

无遥控流程按控制循环的启动相对时间合成挡位：约 0.1 s 后请求 AUTO_HOVER，约 2.1 s 后请求 CMD；状态转移仍取决于输入是否就绪。启用调参开关时延后悬停请求且不自动进入 CMD。

本分支控制器默认值为 OMMPC（1），CMD 默认 `figure_eight`；轨迹由 YAML 选择，重启后生效，无需修改轨迹编译宏。八字与翻转的内环和模型参数须成组切换，见 [调参及切换说明](src/realflight_modules/px4ctrl/OMMPC_FULL_TUNING.md)。

**3. 编译、环境与最短仿真流程**

本机存在 ROS 2 Humble。以下命令在工程根目录运行，每个新终端都需要加载环境：

```bash
cd /home/sun/many_controllerV3
source /opt/ros/humble/setup.bash
# 首次构建或依赖包改变时使用 packages-up-to
colcon build --packages-up-to px4ctrl flight_data_recorder quadsim_mujoco --symlink-install
source install/setup.bash
```

第三方库首次安装见 [3rdpart/README.md](3rdpart/README.md)。关键流程为初始化子模块，然后运行 `JOBS=2 ./3rdpart/build_all.sh`；脚本会安装到 `/usr/local` 并在安装时使用 sudo。当前 CMake 只需要 Eigen、OSQP、acados 和 SuiteSparse/UMFPACK；旧 Ipopt/NLopt 控制器已移除，当前编号为 0、1、2（acados）。

MuJoCo 节点还导入 `mujoco`、`glfw`、NumPy、SciPy、`cv_bridge` 等；源码注释记录 MuJoCo 3.2.3。其 package.xml 还声明了本源码树中没有的 `stservo_msgs`，尽管当前仿真节点没有导入该消息包；新机器依赖解析若在这里失败，需要核查已有外部工作区或清理该遗留声明，不能认为当前包清单已完整覆盖所有环境依赖。

三个终端分别运行：

```bash
# 终端 A：外环 + 内环 + ULog 记录器
ros2 launch px4ctrl run_ctrl.launch.py

# 终端 B：MuJoCo 图形仿真
ros2 launch quadsim_mujoco mujoco.launch.py

# 终端 C：可选，实时轨迹对比
ros2 launch px4ctrl realflight_trajectory_visualizer.launch.py
```

两个控制节点和仿真器有握手等待，先启动的一方等待另一方属于正常流程。`run_ctrl.launch.py` 本身不启动 MuJoCo。

有 GNOME 桌面时，`bash script/start_px4ctrl_terminals.sh` 会打开控制和轨迹显示两个标签页，仍需另行启动 MuJoCo。关闭仿真器不会自动结束记录器，应正常退出记录器以完成日志收尾。

**4. 3 种顶层控制器与内环**

修改 [px4ctrlfsm.h](src/realflight_modules/px4ctrl/include/px4ctrl/px4ctrlfsm.h) 的 `PX4CTRL_PRIMARY_CONTROLLER`，然后编译 `px4ctrl` 并重启：

| 值 | 控制器 | 实现文件 | 主要配置 |
|---|---|---|---|
| 0 | 原 QuadControl 串级控制 | `src/controller.cpp` | `gain`、`filter`、`tuning`；含推力映射 RLS |
| 1 | OmMpcControl，流形误差线性 MPC / OSQP | `src/ommpc.cpp` | `mpc.*` |
| 2 | acados NMPC | `src/acados_nmpc.cpp`、`src/acados_nmpc_solver.cpp`、`generated/acados` | `nmpc.*`，数值参数运行时配置 |

这些控制器共用 `px4ctrlrate_node` 的角速度内环和电机分配。内环包含角速度 PID、TVR 角加速度估计、刚体力矩补偿、推力/力矩分配、转速到归一化油门反解，以及限幅与输出回退诊断。

```bash
colcon build --packages-select px4ctrl --symlink-install
source install/setup.bash
```

宏是 C++ 编译开关，不是 `ros2 param set` 参数，也不是现成的 `-D...` CMake 选项。MPC 的预测步长不同于节点调度周期；NMPC 的代价实现也不能直接当作 `mpc.state_weight` 的另一种求解方式。比较后端时需核对各自模型、代价、预测长度和步长。

acados 的 `nmpc.*` 数值参数从 YAML 在启动时读取，修改后重启即可；`ros2 param set` 不会即时重配求解器。接口与配置说明见 [ACADOS_NMPC.md](src/realflight_modules/px4ctrl/ACADOS_NMPC.md)。只在改变模型、代价维度、约束结构或算法类型时，使用 [生成脚本](script/generate_px4ctrl_acados_nmpc.py)，再重新编译；普通构建直接使用仓库已有的生成 C 源码即可。Python 生成环境配置见第三方 README。

**5. 轨迹生成：统一连续时间接口**

完整说明见 [TRAJECTORIES.md](src/realflight_modules/px4ctrl/TRAJECTORIES.md)。依据 `0913/轨迹生成/uav_trajectory_design.tex` 实现三种圆/螺旋动作，并重写水平八字；旧 minimum-snap、barrel-roll、five-turn、旧八字及其适配器已删除。

| `trajectory.type` | 轨迹 | 主要参数 |
|---|---|---|
| `horizontal_circle` | 平滑加速、匀速、减速水平圆 | `radius=1`，`speed=1.5`，匀速 2 圈 |
| `vertical_circle` | 竖直俯仰翻转圆 | `radius=1`，1 圈，`centripetal_g=1.8` |
| `helix` | 水平轴连续滚转螺旋 | `radius=1`，2 圈，`pitch=0.5 m/圈` |
| `figure_eight` | 平滑起飞与水平八字（默认） | 全尺寸 2×1.2 m，1.5 m/s，2 圈 |
| `omtraj` | 保留离线优化 CSV | `trajectory.omtraj.file` |

所有控制器共用选择入口，所有参数在启动时生成/读取并检查，修改 YAML 后重启。圆主体直径不得超过 2 m；连接段需要额外空间。新轨迹九次连接匹配 p/v/a/jerk/snap，直接生成完整统一参考及解析角加速度，结束后明确悬停。启动时执行全动作推力、角速度、角加速度和静态逐电机分配采样检查；这不等同于闭环验证。

轨迹按进入 CMD 的实测位置和航向对齐，假设激活时近似悬停。离线导出和检查：

```bash
python3 script/analyze_trajectories.py
```

输出 `datalog/trajectory_refactor/` 内的 CSV、配置、指标和轨迹图，不发布飞行指令。

**6. 离线 OmTrajectoryOptimizer**

配置：[omtraj.yaml](src/realflight_modules/px4ctrl/config/omtraj.yaml)。实现：[omtraj.cpp](src/realflight_modules/px4ctrl/src/omtraj.cpp)。启动：

```bash
ros2 launch px4ctrl omtraj_visualizer.launch.py
# 不启动 RViz 窗口，但仍运行优化与发布节点
ros2 launch px4ctrl omtraj_visualizer.launch.py rviz:=false
```

任务参数在 `omtraj.yaml`，独立物理范围在 [omtraj_tracking.yaml](src/realflight_modules/px4ctrl/config/omtraj_tracking.yaml)，launch 同时加载。算法按 `0913/wenzhang0710.docx` 第 III 节实现：9 维 SO(3) 局部增量、联合优化总时间 T、L1 虚拟控制、CSTC 有序航点。Shen 文章仅作为航点约束参考。推力/角速度及其变化率、执行器指令余量、速度/倾角和逐电机限制都是硬约束；没有额外轨迹平滑目标或外层时间二分。

QP 预消元、信赖域内冗余行筛除和按需 L1 可行性恢复，非线性回溯，独立加密积分验收并补充节点间约束。success 表示验收通过，converged 另表示局部收敛，不代表全局最优。详细公式对应、参数影响和闭环证据见 [TRAJECTORIES.md](src/realflight_modules/px4ctrl/TRAJECTORIES.md#omtraj文稿方法约束与控制器对接)。

成功时保存 `datalog/omtraj/omtraj_manifold_v2.csv`；v2 包含模型并在加载时复查动力学。旧 v1 不能直接用于新版控制播放。控制侧设置 `trajectory.type: omtraj`、确认 `trajectory.omtraj.file` 并重启后才会执行。优化显示节点本身不控制飞机。

控制器加载时会以 CSV 起点为基准，对齐进入 CMD 的当前位置和偏航，因而 CSV 的初始高度不是一个会直接命令飞机飞到的绝对起飞高度；若任务需要起飞段，要在轨迹中表达。CSV 修改后需重启控制节点重新加载。

**7. 数据选择、滤波与坐标约定**

在 [input.h](src/realflight_modules/px4ctrl/include/px4ctrl/input.h) 设置 `PX4CTRL_USE_FILTERED_IMU`：

| 值 | 外环输入 | 内环输入 |
|---|---|---|
| 0（当前） | `/fmu/out/sensor_combined` 的陀螺/加速度 | 同话题的角速度 |
| 1 | `/fmu/out/vehicle_angular_velocity` + `/fmu/out/vehicle_acceleration` | `vehicle_angular_velocity` |

更改后重新编译。位置与姿态仍来自 `vehicle_local_position` 和 `vehicle_attitude`，该开关不是任意状态估计器切换。当前 MuJoCo 只发布上述模式 0 的 IMU 话题；切到 1 后，必须有对应发布源。

模式 1 的外环分别检查两路话题是否收到及是否超时；一条持续更新不能掩盖另一条断流。TVR 和原有控制器滤波逻辑仍保留，不直接使用 PX4 消息内的角速度导数替代 TVR。

低通频率在 `params.yaml → filter`；TVR 的窗口、正则化、外推参数在 `px4ctrlrate_node.cpp::main()` 中，目前是源码参数。当前窗口为 200 点，不应把它误认为 YAML 可动态配置项。

工程多处注释使用 ENU/FLU 名称，但 PX4 向工程内部的实际向量转换为 `[x,-y,-z]`，四元数为 `[w,x,-y,-z]`，不是通常 NED→ENU 所包含的 x/y 对换。原生位置任务则直接使用 NED；新增轨迹或分析脚本时应遵循所接接口的实际实现。

**8. 参数辨识、标定和响应测试**

| 能力 | 实际状态 | 怎么用 |
|---|---|---|
| 在线推力映射 RLS | 已实现，仅 QuadControl 路径启用 | 控制器设为 0，在 AUTO_HOVER 中更新；观察 `/debugPx4/ctrl` 的 `thr2acc` |
| 电机油门→转速标定模型 | 已有系数与正反计算 | 在 `params.yaml → motor.rc2speed_a/b/c` 配置实验系数 |
| Ct/Cq 随前进比变化模型 | 已有计算模型 | 配置 `motor.Ct_* / Cq_*`，由 `motor_calculate.h` 计算工作点系数 |
| 电机一阶动态模型 | 仿真已实现 | `quadsim_mujoco/config/params.yaml → motor.Tm_a/Tm_b/Jm` |
| 姿态/角速度阶跃调参 | 已实现于 QuadControl | 控制器设为 0，修改 `tuning.attitude_loop` 或 `angular_rate_loop` 及目标值 |
| 传感器噪声统计 | 有旧 CSV 工具与新 ULog 分析 | 见第 10 节及 `noise_analys.py` |
| 质量、惯量、阻力、Ct/Cq、油门曲线全自动联合辨识 | 当前源码中未找到独立完整入口 | 不应把已配置模型或离线模型复算当成已完成的自动辨识工具 |

RLS 实现在 [controller.cpp](src/realflight_modules/px4ctrl/src/controller.cpp)：把约 35–45 ms 前的输出推力与当前 z 轴加速度配对，拟合 `a_z ≈ thr2acc × thrust`。初始值为 `1/mass`，遗忘因子 0.998，结果限制在 `[0.2, 2.0]`；CMD 阶段不继续更新。当前控制器 1 的同名估计函数不更新模型，NMPC 也不启用这项辨识。

这里 `thrust` 是工程推力接口中的量，不能简单解读为 0–1 油门。`thr2acc` 是整个模型/执行链的有效增益，不是直接测量的质量；`1/thr2acc` 不能单独证明飞机质量发生变化。当前 RLS 更新代码也不能仅因在 AUTO_HOVER 调用就视为已经严格筛选了所有稳态样本。

阶跃目标 `euler_des_*_deg` 用度，`rate_des_*_deg` 用度/秒；由代码转换为弧度。它们覆盖正常参考，用于单环响应实验，不会自动求出 PID 最优增益。当前两个调参开关都为 false。

模型使用已有 16 V 标定条件元数据，但没有据此自动执行电压补偿；离线分析也不把实时电压直接乘到推力模型上。当前源码树未找到从台架原始数据一键重新拟合这些系数的专门脚本。

本地另有 [近期推力辨识与转速映射讨论](0913/近10天交互问题汇总_20260927.md)，它是分析记录，且所在目录被 Git 忽略；不能当作已实现功能或随仓库自动分发的文件。

**9. ESC 反馈和内环诊断**

此能力随 `run_ctrl.launch.py` 的内环自动启动，无需额外节点。配置：[ratectrl_diagnostics.yaml](src/realflight_modules/px4ctrl/config/ratectrl_diagnostics.yaml)。

输入 `/fmu/out/esc_status`，输出 `/debugPx4/motor_feedback` 和扩充的 `/debugPx4/ratectrl`。包含转速、电压、电流、电功率、映射与有效性、期望转速误差、模型推力/力矩、PID、分配残差、限幅、TVR 状态和耗时。

默认按 `actuator_function` 对应 `[101,102,103,104]`。仅当驱动不提供该映射时才显式配置已核实的 `esc_slots`；不能把 ESC 数组顺序自然等同于控制器电机顺序。两类默认超时均为 0.25 s。

```bash
ros2 topic echo /debugPx4/motor_feedback --once
ros2 topic echo /debugPx4/ratectrl --once
```

转速反馈来自 ESC；推力、力矩和机械功率为模型估计。若输入无效或缺失，应看 valid 标志和 NaN。当前 MuJoCo 没有发布 EscStatus，因此这条反馈链不会自动提供仿真 ESC 测量；仿真自身 CSV 中另有模型真值/电机数据。

**10. 日志记录与离线分析**

记录器默认随自研控制 launch 启动，也可以独立使用：

```bash
ros2 launch flight_data_recorder record.launch.py
ros2 launch px4ctrl run_ctrl.launch.py record_data:=false
ros2 launch px4ctrl run_ctrl.launch.py log_directory:=datalog/flightlog/experiment_01
```

第一条是独立记录入口，第二条关闭自动记录，第三条自定义目录；不应将它们理解为需要依次运行的三条命令。一般只运行一个记录器。

配置 `simulation_mode=true` 时启动即记录、正常退出结束；false 时以 VehicleStatus 解锁/上锁控制一段记录，默认状态话题为 `/fmu/out/vehicle_status_v1`。普通数值字段和短定长基本类型数组会记录；字符串、变长数组、嵌套消息数组等可能省略，原始 EscStatus.esc[] 不能假定完整保存，电机分析优先用 MotorFeedbackDebug。

当前 YAML 的 `excluded_topics` **包含 `/fmu/out/vehicle_attitude`**，虽然该话题在记录器代码的核心列表内，也会被排除。分析器可尝试使用扩充版 ratectrl 的 `attitude_wxyz` 作为后备；如果日志也没有该字段，则实际姿态分析/姿态回放不可用。需要记录原始姿态话题时，移除这项排除并重启记录器；旧日志不会补齐。此前仅根据 README 列举“支持姿态记录”并不能代表当前配置实际记录了它。

分析入口：

```bash
python3 -m pip install -r script/flight_analysis/requirements.txt
python3 script/analyze_flight_log.py datalog/flightlog/你的日志.ulg
# 使用当次实验的参数快照，并对比上一份结果目录
python3 script/analyze_flight_log.py datalog/flightlog/你的日志.ulg \
  --params datalog/flightlog/你的实验参数.yaml \
  --compare datalog/flightlog/analysis/上一次结果目录
```

分析脚本无需启动 ROS，依赖 NumPy/SciPy/Matplotlib/Pillow/pyulog/PyYAML/fonttools 及中文字体。输入是本项目记录的 ROS ULog，不能直接等同于原生 PX4 SD 卡 ULog，也不再接收旧 CSV 作为主输入。

工作方式：先生成全程预览，按 FSM `state==3` 找 CMD 区间，再逐段询问是否纳入跟踪统计。回车默认不纳入。确认范围是完整 CMD 区间，包含其中起飞、稳定和结束保持，不会自动裁剪成“只有机动部分”。拒绝跟踪统计仍会生成可用的全程曲线与其他诊断。

主要输出到 `<日志目录>/analysis/<本次唯一目录>/`：

| 输出 | 用途 |
|---|---|
| `report.pdf`、`conclusions.md` | 中文图文报告与结论 |
| `flight_replay.gif` | 全过程三维回放 |
| `images/` | 轨迹、误差、控制输出、频谱等图 |
| `summary.json` | 汇总与分段指标、有效性、来源和参数信息 |
| `data/` | 原始 NPZ、信号映射、跟踪与频谱 CSV |
| `comparison.json/csv` | 历次实验比较 |
| `manifest.json` | 完成、失败或中断状态 |

指标覆盖位置/速度/姿态/角速度等跟踪误差、分配误差、输出平滑度、限幅、求解耗时、数据质量、频谱和有数据支持的 ESC 模型比较。时间对齐基于主机接收时间，用因果保持匹配参考；仿真加速或暂停时，频率/时延/积分结果不能直接当作仿真物理时间结果。

旧工具 `python3 script/noise_analys.py` 需要先修改源码中硬编码的 CSV 路径、列名和筛选时间；它做一次 3σ 过滤及均值/方差统计，不是通用命令行分析器。当前“先去掉前 25 s”的筛选随后被 `filtered_df = df` 覆盖，实际又取开头约 12 s，不能照注释认为它已经选中了稳定悬停段。

完整分析约定见 [flight_analysis/README.md](script/flight_analysis/README.md)，记录器细节见 [flight_data_recorder/README.md](src/realflight_modules/flight_data_recorder/README.md)；当前启用值以 YAML 和源码为准。

**11. 实时可视化与 PX4 原生位置任务**

实时对比节点订阅位置、姿态和 `/debugPx4/ctrl`，发布 `/realflight/trajectory_markers`。显示与记录是独立的，记录器排除姿态不会阻止正在运行的可视化节点订阅姿态：

```bash
ros2 launch px4ctrl realflight_trajectory_visualizer.launch.py
ros2 launch px4ctrl realflight_trajectory_visualizer.launch.py rviz:=false
```

第二条只关闭 RViz 窗口，仍运行 marker 发布节点。launch 支持 `debug_topic`、`position_topic` 参数。此入口显示实时跟踪结果；`omtraj_visualizer` 显示离线优化结果。

原生位置任务：

```bash
ros2 launch px4ctrl px4_native_position.launch.py
```

修改 `px4_native_position_mission.yaml` 的点数、展开的 `points_ned`、速度控制、到点阈值、保持时间等。当前为 3 个点、NED 高度 down=-5 m、位置设定值移动速度 0.8 m/s；终点保持，不自动降落。速度控制只是移动发送的位置设定值，不是对真实飞行速度的硬保证。

`simulation.enabled=true` 时等待定位就绪后自动进入 Offboard、解锁并执行；false 时由 AUX1 UP 启动、DOWN 暂停并悬停。该参数与自研控制器的 C++ SIMULATION 宏是两个不同开关。它依赖实际 PX4 或具备对应接口的 PX4 仿真；运行该 launch 不会自动启动 PX4，也不自动启动记录器。

**12. 实机辅助、消息兼容性与飞控重启**

```bash
bash script/start_onboard_terminals.sh
```

此脚本开启 `MicroXRCEAgent udp4 -p 8888`、`vrpn_client_ros sample.launch.py`、`vrpn_client_ros px4_bridge.launch.py`。依赖 `~/nokov_ws/install/setup.bash` 和 `~/px4Msg/install/setup.bash`，相关动捕/桥接实现不在当前仓库中，需要按目标机器调整。它需要 GNOME 图形终端。

自研实机控制需关闭 `input.h` 中的 `SIMULATION` 与 `USE_WITHOUT_RC`，重新编译；否则保留仿真握手和自动流程。记录器实飞解锁触发则另外设置 `simulation_mode=false`。

本仓库提供消息一致性检查：

```bash
source install/setup.bash
python3 script/check_px4_compatibility.py
```

该脚本检查实际加载的 px4_msgs 工作区、消息版本、代码中的话题名、序列化与仿真消息字段，不会连接飞控去证明整条 DDS 链路已可用。

AUX5 的当前行为是请求**整个 PX4 飞控重启**，历史“EKF 重启”的提交标题不代表当前功能。它集成于 `px4ctrl_node`，未解锁、RC/状态新鲜且恢复监测前置条件满足时，拨动 AUX5 发出一次标准命令；之后观察 ACK 与状态/姿态消息中断和恢复。终端绿色 `DDS_RECONNECTED` 表示这两路下行数据稳定恢复，不等于所有话题、上行命令和 EKF 都已验证。详见 [firmware/README.md](firmware/README.md)。

**13. MINCO 与分支中的历史功能**

| 分支 | 当前关系 |
|---|---|
| `px4DataSelect` | 数据源选择和 ESC/内环诊断已包含在当前分支 |
| `data_analyze` | 记录、路径整理、分析功能已包含在当前分支 |
| `controllerOM` / `trajectoryOM` | 停留于共同的较早提交，并非当前功能的完整快照 |
| `minco` | `9efa9a5` 新增独立 MINCO 包，尚未合入当前分支 |
| `simulation` / `trajectory-tracking` | 整理时均基于 `3a78c7f`，含最近的仿真参数调整 |

MINCO 分支具有 `MINCO_S2NU/S3NU/S4NU`，分别提供加速度、jerk、snap 平方积分平滑目标；有命令行示例和独立 RViz 节点。其可视化示例按给定航点与分段时间生成轨迹，并不包含自动避障、时间优化或与当前控制器的完整接入。

下面命令仅适用于已经检出或集成了 `minco` 包的源码树，当前分支不能据此直接编译该包：

```bash
colcon build --packages-select minco
source install/setup.bash
ros2 run minco minco_example
ros2 launch minco minco_visualizer.launch.py
```

可先只读查看其说明：`git show minco:src/realflight_modules/minco/README.md`。本次解析轨迹重构也没有引入 MINCO 包。

**14. 修改入口、数据路径与生效规则速查**

| 想做什么 | 改哪里 | 生效方式 |
|---|---|---|
| 换顶层控制器 | `px4ctrlfsm.h` 的 `PX4CTRL_PRIMARY_CONTROLLER` | 编译并重启 |
| 换 CMD 轨迹 | `params.yaml → trajectory.type` | 重启 |
| 切实机 / 无遥控 / IMU 来源 | `input.h` 的宏 | 编译并重启 |
| 调模型、PID、MPC/NMPC 参数 | `px4ctrl/config/params.yaml` | 默认按修改配置、重启相关节点使用；不要假定支持即时热更新 |
| 调八字尺度/速度 | `trajectory.figure_eight.*` | 重启后重新生成与检查 |
| 调圆/螺旋 | `trajectory.horizontal_circle/vertical_circle/helix.*` | 重启后重新生成与检查 |
| 改离线优化航点/限制 | `omtraj.yaml`、`omtraj_tracking.yaml` | 重跑优化，成功生成 CSV 后重启控制器 |
| 改噪声/风场/电机时间常数 | `quadsim_mujoco/config/params.yaml` | 重启仿真 |
| 改记录范围和记录触发 | `flight_data_recorder/config/recorder.yaml` | 重启记录器 |
| 改 ESC 映射/诊断超时 | `ratectrl_diagnostics.yaml` | 重启内环 |

launch 默认读取安装空间配置。使用 `--symlink-install` 时应核实配置是否链接到源码；普通安装修改源码 YAML 后需重新构建/安装相关包，避免改了源码却运行旧配置。

| 路径 | 内容 |
|---|---|
| `datalog/flightlog` | ROS ULog |
| `datalog/flightlog/analysis` | 默认离线分析结果 |
| `datalog/quadsimlog` | 仿真自身 CSV |
| `datalog/omtraj` | 优化轨迹 CSV |
| `build` / `install` / `log` | colcon 构建、安装、构建日志 |
| `docs` / `0913` | 本地说明、论文和分析记录，当前 Git 忽略 |

`datalog` 也被 Git 忽略，因此切换/同步分支不会自动带走实验日志或离线优化 CSV。本文放在工程根目录，避免和被忽略的本地 docs 混在一起。本次只新增说明文件，没有提交或推送。
