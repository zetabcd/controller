# ULog 飞行分析

入口：`script/analyze_flight_log.py`。原飞行日志 CSV 分析已替换为 ULog 分析；独立噪声工具 `script/noise_analys.py` 保留原样，新报告另提供噪声观察。只读日志，不启动 ROS 节点，不改变记录器、控制器、DDS 或消息定义。

## 使用

```bash
# 在工程根目录运行，不要求启动 ROS
python3 script/analyze_flight_log.py datalog/flightlog/flight_xxx.ulg

# 用本次飞行实际使用的参数补充离线模型，并与以往结果对比
python3 script/analyze_flight_log.py datalog/flightlog/flight_xxx.ulg \
  --params src/realflight_modules/px4ctrl/config/params.yaml \
  --compare datalog/flightlog/analysis/previous_run
```

分析器作为独立 Python 工具运行，不安装到 ROS 包中，无需 ROS 环境或 colcon 编译。
Python 依赖见本目录 `requirements.txt`；还需要中文 TTF/TTC 字体（可用 `--font` 指定）。输入支持本项目逐话题 ROS ULog，**不将原生 PX4 SD 卡 ULog 当作同一数据结构**。

首次使用若缺 Python 依赖：`python3 -m pip install -r script/flight_analysis/requirements.txt`。其中 pyulog 通过该依赖文件安装。

## 自动识别 CMD，每段由人决定是否统计

1. 程序根据 FSM `state == 3` 自动识别 CMD 区间，生成 `images/00_flight_preview.png`，橙色标出这些区间，同时打印日志时长、结果路径。优先使用 `/debugPx4/ctrl.state`；缺失时使用扩充版 ratectrl 的 `fsm_state`。
2. 对每个自动识别的区间询问“是否生成本段轨迹跟踪指标”，只需回答 y 或 n，**不再输入起止时间，移除 `--trajectory-window` 参数**。多次进入 CMD 可以逐段接受或跳过。
3. 默认否：回车、n 或标准输入结束均不会启用尚未确认区间的指标；已经回答 y 的区间保留。没有自动启用的 `--yes` 开关。
4. **统计覆盖确认的完整 CMD 区间，包含其中的起飞、稳定、机动和结束保持。** 程序不判断飞行是否成功，仍由你决定这段实验是否值得统计；不根据参考速度自动裁剪区间。
5. 位置、速度、姿态、角速度、角加速度、分配误差、ESC 转速/模型推力误差等全部跟踪指标和误差分布，只在确认区间内计算，同时保存汇总和各段指标。全部拒绝时不会生成这些误差图、误差 CSV 或统计。没有 FSM 或没有可用 CMD 区间时，自动跳过跟踪指标，不要求手动补时间。
6. 无论是否生成轨迹指标，都保存全程原始曲线、输出平滑度、求解耗时、限幅标志、传感器波动和频谱、数据质量、PDF 与全程动画。

CMD 区间从首次收到 `state == 3` 开始，到首次收到其他状态结束；采用左闭右开 `[开始, 结束)`，退出 CMD 的样本不计入。状态最多保持 `--max-age`（默认 0.1 s）：状态消息长时间中断会截断并拆分区间，不跨未知状态统计。日志结束时仍处 CMD 的区间截止到日志结束或最后状态的新鲜度上限，取较早者；这不代表实际轨迹已完成。

自动阶段标签仅辅助观察：手动、悬停/移点、CMD、上升/起飞候选、轨迹运动候选、下降候选、PX4 已落地。参考速度阈值为 0.05 m/s；该启发式不参与统计区间选择，也不能判定飞行成功。

## 输出

默认新建 `<日志目录>/analysis/<日志名>_<本次时间>_<唯一标识>/`。`--output-root` 可指定结果根目录，每次都新建子目录，不覆盖历史分析。

- `report.pdf`：中文结论、全部结果 PNG、完整数值/方法附录；动画在 PDF 中提供关键帧。
- `flight_replay.gif`：覆盖整个已记录过程的三维位置/姿态回放、时间曲线与阶段显示。位置/姿态未记录时明确显示缺失，不用参考值冒充实测。
- `images/`：全程概览、轨迹跟踪、控制输出、角速度环/PID/分配器、ESC、限幅/耗时/TVR、传感器、箱线图、频谱、可用的电压关联、多次飞行对比、动画关键帧。
- `summary.json`：机器可读指标、有效样本数、自动识别区间、逐段确认结果、实际统计区间、来源哈希、字段覆盖率、参数来源、结论。缺失值用 `null`，没有伪造的零误差。
- `conclusions.md`：中文结论。
- `data/raw/*.npz`：所有已记录话题的原始数值字段，保留 uint64 时间戳精度，不只保存分析用的字段。
- `data/ulog_metadata.json`：话题、实例、原始字段/类型、记录器元信息及省略字段。
- `data/signal_mapping.json` 与 `data/*.npz`：分析量的来源、坐标系、单位及时间序列。
- `data/tracking_*.csv`：仅确认轨迹段的实际、期望、误差。CSV 只作为分析结果导出，不再作为输入。
- `data/psd_*.csv`：每个连续数据段的频率/功率谱。
- `comparison.json/csv`：指定 `--compare` 时的历次指标表。
- `manifest.json`：完成/失败/中断状态；中断留下的文件不标作完成报告。

GIF 默认 10 fps、期望 1 倍速、最多 240 帧。长日志自动提高播放倍速以覆盖全程，画面和 JSON 中标注实际倍速。可调整 `--animation-fps`、`--animation-speed`、`--animation-max-frames`。PDF 本身不能播放 GIF。

## 数据与计算约定

### 时间和有效性

- 主时间轴使用 `recorder_monotonic_ns`，缺失时用 ULog `timestamp`；先以整数减去首条接收时间，再转换为秒，避免大整数精度损失。
- 不混用 PX4 源时钟、ROS 时钟和主机单调时钟。源时间戳保留用于回跳、重复等检查，不将不同钟域直接相减当作传输延迟。
- 仿真加速或暂停时，本报告的频率、相关滞后、能量积分和数值导数仍基于主机接收时间，不能直接当作仿真时间下的物理量。
- 跟踪比较在实际量的原始采样时刻，用**因果零阶保持**匹配最近已接收参考；`--max-age` 默认 0.1 s，过旧则无效。这样不会用未来消息“改善”误差，但包含参考采样保持和接收抖动。
- 各分析量分别检查有效性，位置缺失不阻止角速度分析。统计使用全部有效样本；画图可减少显示点数。
- PSD 和相关滞后只在有效连续片段内均匀重采样，间隔取中位数；不跨大记录缺口拼接。相关滞后正值表示实际落后期望，不等于通信延迟。低激励、低相关、搜索边界结果不作为可靠延迟。
- 统计是按样本而非按时间加权；可用样本数和有效比例随指标保存。不同采样率和缺失情况必须考虑。单桨/各轴对应 `1..4` / `1..3`，三轴即 x/y/z，姿态即 roll/pitch/yaw。

### 坐标和力矩

- 与 `input.cpp` 保持一致：PX4 的位置、速度和 FRD 向量转为 `[x,-y,-z]`，四元数转为 `[w,x,-y,-z]`。这里是本工程的世界系约定，不宣称是标准 ENU 轴交换。
- 旧 `Px4ratectrlDebug` 的角加速度/力矩标量是 FRD，转换为 FLU；schema 2 数组本来就是 FLU，不能再翻转一次。
- `Px4ctrlDebug` 的参考位置/速度/四元数/角速度也使用历史翻转约定，分析时同样还原。`fb_a` 当前未被控制器赋值，MPC 的 `ref_a` 也可能一直为默认零；实际世界系加速度使用 PX4 local_position 的速度导数，参考加速度用 `ref_v` 分段差分，明确属于数值估计。
- 四元数先归一化；整体误差为 `2 acos(abs(q_actual dot q_desired))`。欧拉角误差包装到 ±180°；翻滚时优先看四元数角距。
- 跟踪误差统一为实际/估计减期望。分配残差保留控制器习惯，单独标为期望减分配。
- `tau_sol/cur_tau` 是分配模型值，`motor_thrust` 是分配解，期望 RPM 不是 ESC 反馈。最终 actuator 是发送到 PX4 的控制指令，不能称为实测电机输出。

### RPM 反算、参数来源

兼容 `px4DataSelect` 的 `MotorFeedbackDebug` 和扩充的 ratectrl 消息，优先使用记录的有效标志、机械 RPM、映射后的电机顺序和每周期模型参数。ESC 重复缓存按 `esc_sequence` 去重，避免把控制循环频率当作 ESC 反馈频率。

离线独立复算使用 `motor_calculate.h` 的公式：

```text
omega = mechanical_RPM * 2*pi/60
J = clip(pi * v_air_body_z / (omega*R + 1e-8), 0, 1e10)
CT = clip(Ct_a*J² + Ct_b*J + Ct_c, Ct_c/10, Ct_c)
CQ = clip(Cq_a*J² + Cq_b*J + Cq_c, Cq_c/10, Cq_c)
ct = 4*rho*R^4*CT/pi²
cm = 8*rho*R^5*CQ/pi²
T_i = ct_i*omega_i²
Q_i = cm_i*omega_i²
T_total = sum(T_i)
tau_x = l*sin(beta)*(T1-T2-T3+T4)
tau_y = l*cos(beta)*(-T1-T2+T3+T4)
tau_z = Q1-Q2+Q3-Q4
P_electrical = V*I
P_aerodynamic_model = Q*omega
```

使用记录的来流；必要时由实际姿态和实际速度计算，假设风速为零。缺姿态时不默认用期望姿态或零来流。优先使用日志内系数，缺少时仅采用用户显式传入 `--params` 的参数，记录文件 SHA256 和参数内容，不静默使用当前 checkout 的默认参数。

16 V 是已有转速—油门标定的条件，不凭空进行电压推力修正。推力/力矩和机械功率必须标注模型估计。逆动力学 `J*alpha + omega×(J*omega)` 是含扰动的净力矩估计，不能等同于桨实测力矩。

旧日志可用分配推力和期望 RPM 反推控制器有效 `ct=T/omega²`，单独标为 reconstructed；不冒充直接记录的工作点，更不能只靠转速凭空辨识 CQ。

### 覆盖限制与缺失提示

当前 `data_analyze` 的历史样例没有实际姿态和 ESC 反馈；这些分析会生成明确的缺失说明，其余分析继续。

本分支记录器省略非基本类型数组，因此原始 `EscStatus.esc[]` 可能被省略。只有 EscStatus 顶层状态不足以恢复 RPM。需要日志中真正存在 `MotorFeedbackDebug` 的基本类型数组，不能靠分析端补出没记录的数据。

扩充 ratectrl 后可分析 PID、TVR、ct/cm、分配饱和、积分限幅、回退和完整周期耗时；旧版只支持已记录的子集。当前实现不修改记录范围或生产消息。

噪声观察保留原脚本的一次 3σ 筛选、均值/标准差/样本方差，但明确标为“运动+噪声波动”，整次飞行不能替代静止标定。频谱主峰同样可能来自参考运动，工具不自动断言故障。

## 文件分工

- `ulog_data.py`：读取、字段映射、坐标/有效性处理、原始数据导出。
- `models.py`：分配残差、逆动力学、电功率、ESC 推力/反扭矩离线复算。
- `metrics.py`：阶段候选、人工范围内跟踪统计、相关滞后、PSD、输出/约束/计时/噪声/质量统计。
- `report.py`：图片、结论、数据导出、多次报告对比、PDF 和 GIF。
- `cli.py`：交互确认、唯一结果目录和完整工作流。
