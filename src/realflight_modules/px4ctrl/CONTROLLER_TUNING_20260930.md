# 传统控制器与 Acados NMPC 参数调试（2026-09-30）

本轮比较 **26 组控制参数、74 次闭环运行**，包含被排除的振荡候选。最终选择是本轮所测条件下的折中，不是全局最优或实机稳定性证明。

- QuadControl：`config/params_quad_tracking.yaml`，八字/水平圆共用一组参数，默认八字；编译控制器 **0**。
- Acados NMPC：`config/params_acados_flip.yaml`，垂直圆/螺旋共用一组参数，默认螺旋；编译控制器 **2**。
- 每个修改参数旁的 `# 旧:` 分别对照本轮开始时的 `params.yaml` 和 `params_ommpc_flip.yaml`。新文件为完整配置。
- 当前工程原有 OMMPC 默认和用户已有配置改动保留；加载 YAML 不会自动切换控制算法。

## 对照结果

全机动段三维位置误差，单位 mm。表中 RMSE 为 seed71/72 两次 RMS 的均值，最大误差为两次的最大值；不是只取最好的一次。轨迹尺寸、速度、圈数、物理质量/惯量/电机/阻力和噪声设置相同。

| 控制器 / 轨迹 | 原参数 RMS | 新参数 RMS | 原参数最大 | 新参数最大 |
|---|---:|---:|---:|---:|
| QuadControl / 八字 | 91.09 | 5.75 | 153.68 | 10.58 |
| QuadControl / 水平圆 | 96.98 | 5.01 | 156.32 | 12.17 |
| Acados / 垂直圆翻转 | 33.04 | 10.03 | 63.96 | 24.26 |
| Acados / 螺旋翻转 | 26.86 | 10.64 | 66.83 | 25.31 |

传统控制器的表格使用**修正加速度坐标系后的旧参数**作为基线，从而隔离参数变化的收益。修正之前，seed71 八字/水平圆分别为 153.51/177.50 mm RMS；仅修正传感器后为 90.93/95.22 mm。不能把这部分收益全归给调参。Acados 不使用加速度计反馈，本次该修正不改变其控制输入。

## 最终参数

### QuadControl

| 层级 | 新值 xyz |
|---|---|
| 位置 P | `[4,4,2]` |
| 速度 P / I / D | `[4,4,3]` / `[0.8,0.8,0.5]` / `[0,0,0]` |
| 姿态四元数增益 | `[24,24,4]` |
| 角速度 P / I / D | `[20,24,8]` / `[3,3,0.5]` / `[0.3,0.3,0]` |
| 角加速度前馈系数 | `[1.3,1.3,1]` |
| 外环 / 内环 | `100 / 400 Hz` |

原速度控制律近似为 `a_cmd = Kv*(v_ref + Kp*(p_ref-p)-v) + I - D*a_measured + a_ref`。速度 D 没有参考端补偿；即使理想跟踪，动态项也是 `(1-D)*a_ref`，会抵消一部分机动前馈。关闭**速度环 D**后仍保留速度 P 提供的阻尼和独立的角速度环 D。

只加大位置/速度 P 的 `quad_p4`、`quad_p6` 分别出现约 0.15/0.39 m RMS，并在末端持续振荡。最终先提升姿态/角速度环响应，再使用适度外环增益。八字和水平圆不需要分别换 PID。

### Acados NMPC

| 项目 | 新值 |
|---|---|
| N / prediction_dt | `12 / 0.04 s`，预测 0.48 s |
| rate_time_constant | `[0.08, 0.0666666667, 0.125] s` |
| state_weight：p / v / quaternion | `[3200,3200,3200] / [6,6,6] / [5,5,5,5]` |
| terminal_weight：p / v / quaternion | `[6400,6400,6400] / [12,12,12] / [10,10,10,10]` |
| input_weight | `[0.10,0.50,0.50,0.20]`，相对参考输入的偏差惩罚 |
| command_change_weight | `[0.02,0.12,0.12,0.12]` |
| 内环 P / I / D | `[20,24,8]` / `[3,3,0.5]` / `[0.6,0.6,0]` |
| 角加速度前馈系数 | `[1.6,1.6,1]` |
| 求解预算 / 最大迭代 / 容差 | `8 ms / 60 / 1e-5` |

先按内环组合匹配时间常数，再比较 Qp=100、200、400、800、1600、3200、6400，及网格、输入权重、命令平滑、内环 P、模型时间常数、前馈系数。时间常数的简化匹配 `(1+D)/P` 只是初值依据，并非完整执行链辨识。

Qp=6400 出现强烈振荡，垂直圆/螺旋角速度峰值约 14.9/16.3 rad/s，并出现少量求解回退；因此停止继续增大权重。降低输入惩罚、提高命令平滑惩罚都有部分位置指标变好，但角速度响应明显变差，未选用。Qp=1600 在螺旋上较好、垂直圆上较差。Qp=3200 的原输入权重组合在 seed71/72 位置指标很好，但 seed73 垂直圆复测出现俯仰角速度 RMS 6.78 rad/s、姿态 RMS 8.23°，因此没有直接采用。最终增加相对参考角速度的输入偏差惩罚到 `[0.5,0.5,0.2]`，保留 Qp=3200 和原命令变化惩罚。

最终组 seed73 的 60 s 垂直圆复测：位置 RMS 10.03 mm、俯仰角速度 RMS 1.28 rad/s、姿态 RMS 2.33°、实际角速度峰值 9.72 rad/s。相对前一候选略微牺牲部分位置指标，换取更一致的姿态和角速度响应。增加输入偏差惩罚与增加命令变化惩罚含义不同：前者在整个预测窗口约束偏离规划输入，后者在当前实现中只约束首个控制量相对上次命令的变化。

另测了提高姿态权重、减小内环 D/前馈、24 节点 × 0.02 s 网格。细网格候选反而达到约 168 mm RMS 并出现求解回退，未采用；本实现阶段代价按离散求和，不随 dt 缩放，增加节点也改变了成本总量，不能把这个结果归结为采样周期一个因素。姿态误差指物理姿态与控制器发布的期望姿态的夹角，角速度误差指实际角速度与保持的控制命令的差，不是纯延迟测量。

## 复测范围与结果

机动 RMS/峰值及末端悬停最大误差单位 mm；角速度为物理机体三轴分量绝对值的最大值，rad/s。悬停统计从轨迹结束 3 s 后开始，不含过渡段；所有原始样本保留，可重算。长测试总时长 60 s，包含轨迹结束后至少 30 s 的继续控制。

| 控制器 / 轨迹 | 条件 | 机动 RMS | 机动最大 | 悬停最大 | 角速度峰值 |
|---|---|---:|---:|---:|---:|
| Acados / 螺旋翻转 | 60 s 长测试 | 8.00 | 23.00 | 1.37 | 9.79 |
| Acados / 螺旋翻转 | IMU 标准差 ×2 | 7.60 | 21.50 | 2.53 | 9.70 |
| Acados / 螺旋翻转 | 1 cm / 0.03 m/s 位速观测噪声 | 8.79 | 20.47 | 9.93 | 9.63 |
| Acados / 螺旋翻转 | 地面启动 | 8.13 | 23.60 | 0.87 | 9.81 |
| Acados / 垂直圆翻转 | seed73 复测 | 9.70 | 21.25 | 1.01 | 9.69 |
| Acados / 垂直圆翻转 | 60 s 长测试 | 10.03 | 23.13 | 1.01 | 9.72 |
| Acados / 垂直圆翻转 | 1 m/s 均值风场 | 10.21 | 21.68 | 6.25 | 9.76 |
| Acados / 垂直圆翻转 | 1 cm / 0.03 m/s 位速观测噪声 | 10.60 | 25.08 | 10.24 | 9.54 |
| Quad / 八字 | 60 s 长测试 | 6.20 | 10.51 | 1.81 | 0.54 |
| Quad / 八字 | IMU 标准差 ×2 | 6.23 | 10.10 | 4.67 | 0.59 |
| Quad / 八字 | 1 cm / 0.03 m/s 位速观测噪声 | 7.61 | 13.91 | 6.68 | 0.80 |
| Quad / 八字 | 地面启动 | 6.47 | 10.79 | 1.90 | 0.53 |
| Quad / 水平圆 | 60 s 长测试 | 6.08 | 12.14 | 4.46 | 0.50 |
| Quad / 水平圆 | 1 m/s 均值风场 | 7.38 | 15.81 | 8.71 | 0.50 |
| Quad / 水平圆 | 1 cm / 0.03 m/s 位速观测噪声 | 8.56 | 17.05 | 8.81 | 0.72 |

最终组共 23 次运行，均完成轨迹、状态和输出有限、未出现 CMD 异常退出；Acados 在机动与末端悬停均无求解回退。本轮使用统一仿真时钟，这些结果不代表此前时钟异常或模式退出问题已被修复。

IMU 工况将现有陀螺仪和加速度计标准差都加倍；风扰使用原 Dryden 模型、均值参数 1 m/s、风场 seed42，风速未提供给控制器。位置/速度观测噪声为每轴独立高斯标准差 0.01 m / 0.03 m/s，使用独立 RNG，不改变物理状态或记录真值。地面启动高度为 0.0023 m；普通筛选从 0.5 m 开始，二者明确区分。测试脚本沿用初始电机转速为悬停值的设置，因此地面测试验证的是起始位置变化，不是实机从电机静止、解锁到起飞的完整流程。

## 仿真接口修正与实物含义

`quadsim_node.py` 的 SensorCombined 曾把世界系比力 `state.acc` 当作机体系量发布，控制器会再次旋转，倾斜时产生错误反馈。现改为 MuJoCo 加速度计原始机体系比力 `state.acc_B`，再做 FLU→FRD 符号转换。没有更改刚体/电机/气动力参数，也没有减小噪声。用水平、倾斜、倒置三种姿态抵消重力的 MuJoCo 测试验证：旧版本两个倾斜案例失败，修正后三个案例均通过。

控制器调用和输入输出契约不变：QuadControl 使用位置、速度、姿态、陀螺仪和加速度计；Acados 使用位置、速度、姿态、陀螺仪与参考窗口。输出仍为牛顿制总推力、机体系角速度和角加速度前馈。前馈来自规划轨迹的解析导数，不需要真实扰动力、真实风速或真实电机推力，也没有新增外部真值输入。

普通仿真仍使用真值位置/速度/姿态并共享物理参数；增加位速观测噪声仅是一种补充验证，不能代替 EKF、姿态估计误差、通信时延、参数失配和实机测试。位速白噪声也不能代表有偏、相关或丢失的估计误差。高位置权重不能原样当成实机精度保证。当前执行链仍是项目自研 400 Hz 角速度内环，不能把这里的闭环增益直接等同于 PX4 原生内环。

## 使用与切换

从工程根目录执行。先停止上一组控制节点；两种控制器二选一。以下通过 CMake 编译宏选择后端，**不需要同时编辑头文件**；之后继续用同一方式切换，避免缓存的编译宏覆盖头文件。

```bash
source install/setup.bash
# 传统控制器：八字，或在此 YAML 中把 type 改成 horizontal_circle
colcon build --packages-select px4ctrl --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-DPX4CTRL_PRIMARY_CONTROLLER=0
source install/setup.bash
ros2 launch px4ctrl run_ctrl.launch.py use_sim_time:=true \
  params_file:="$PWD/src/realflight_modules/px4ctrl/config/params_quad_tracking.yaml"
```

```bash
# Acados：螺旋，或在此 YAML 中把 type 改成 vertical_circle
colcon build --packages-select px4ctrl --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-DPX4CTRL_PRIMARY_CONTROLLER=2
source install/setup.bash
ros2 launch px4ctrl run_ctrl.launch.py use_sim_time:=true \
  params_file:="$PWD/src/realflight_modules/px4ctrl/config/params_acados_flip.yaml"
```

MuJoCo 可按原方式单独启动；需要同时启动仿真时，把 `run_ctrl.launch.py` 改为 `run_sim.launch.py`（该入口已统一 `/clock`）。配置改动后重启内外环；同一控制器内仅换轨迹无需重新编译。恢复 OMMPC 用相同 build 命令将编译宏改为 1，再加载对应 OMMPC 参数文件。

本轮保持原有轨迹规格：八字 2×1.2 m、1.5 m/s、2 圈、3 s 加减速；水平圆半径 1 m、巡航 2 圈、1.5 m/s、3 s 加减速；垂直圆/螺旋半径 1 m、向心加速度 1.8g、分别 1/2 圈、螺距 0.5 m。改变尺寸、速度或翻转几何后需要重新验证。

## 代码验证

`colcon build --packages-select px4ctrl` 成功；`ommpc_test`、`acados_nmpc_test`、`trajectory_test`、`control_reference_test`、`controller_adapter_test` 五个 CTest 目标全部通过。加速度坐标系的三个 pytest 用例全部通过，实验脚本语法检查通过。两份最终配置经 YAML 解析后与闭环测试输入一致，且实际安装的仿真代码与已验证源码一致。

## 数据与复现

完整记录在 `datalog/controller_tuning_20260930/`：每次都有原始 `raw.npz`、完整 YAML、二进制/源码哈希、运行条件、日志和指标。`validation_summary.json` 汇总最终组，`all_metrics.json` 和 `comparison.png` 保留整体对比，被排除的候选也保留。该目录按工程原规则被 Git 忽略；需要交接实验数据时应另行备份。

通用仿真脚本新增 `--controller-executable` 支持独立后端二进制，及 `--position-noise-std` / `--velocity-noise-std` 支持观测误差测试；未替换当前安装的 OMMPC 程序。示例：

```bash
python3 script/acados_closed_loop/run.py reproduce_acados \
  --controller-executable "$PWD/build/px4ctrl-interface-2/px4ctrl_node" \
  --controller-label 'Acados NMPC' --duration 60 --seed 73 \
  --config "$PWD/src/realflight_modules/px4ctrl/config/params_acados_flip.yaml" \
  --output-root "$PWD/datalog/controller_tuning_reproduce"
```

独立二进制来自同一源码，分别以 `-DPX4CTRL_PRIMARY_CONTROLLER=0/2` 编译。若清理了独立 build 目录，可使用正常编译后的安装程序作为 `--controller-executable`；必须检查实际后端与参数文件匹配。
