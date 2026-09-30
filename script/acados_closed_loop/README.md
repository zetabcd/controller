# OMMPC / acados / MuJoCo 闭环检查

运行真实的 px4ctrl_node、px4ctrlrate_node、MuJoCo/QUAD 电机和气动模型。
控制器不会连接默认 ROS 域：固定 ROS_DOMAIN_ID=72，只用本机 UDP 回环。
实验启动前请确保此域没有其他实验。脚本只结束自己启动的进程。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 script/acados_closed_loop/run.py q18_seed42
python3 script/acados_closed_loop/analyze.py q18_seed42
```

`--config path/to/params.yaml`、`--seed 7`、`--duration 43`、`--output-root /path`
可以指定参数、噪声种子、运行时间和结果目录。默认结果在 datalog/acados_closed_loop；
每个运行名必须唯一，避免覆盖证据。每次保存配置、二进制 SHA256、原始数组、节点日志。
分析器的 `--output-root` 必须与运行脚本一致；生成 metrics.json 和 comparison.png/json。

`--initial-altitude 0.0023` 可从 GUI 模型的地面高度起飞。
`--real-time-factor 0.8` 让 1 秒物理运动用 1.25 秒墙钟时间执行，用于检查仿真变慢时的表现；
控制器仍使用 `/clock`，不改变轨迹的物理速度。默认值为 1，允许范围为 `(0, 1]`。
这种均匀降速不等价于 GUI 渲染造成的随机卡顿。

`--stall-ms 10 --stall-period 0.25` 每 0.25 秒物理时间注入一次 10 ms 墙钟停顿，
然后按原 deadline 追赶，检查停顿和集中补步时的控制表现；默认不注入。
`--simulator-config path/to/simulator.yaml` 可以传入噪声或扰动工况，实际配置会复制到实验目录。
不要在控制参数对比中同时改变机体或电机物理参数。

轨迹由传入 YAML 的 `trajectory.type` 选择。分析器使用参数快照调用 `trajectory_inspect`，
按生成器的 main_start/main_end 评价圆/螺旋/八字主体，另报含进出场的 maneuver 指标。
当前分析器支持四种解析轨迹；omtraj CSV 需要另行指定评价窗口，不会套用八字指标。
同一套参数下仅改变 Qp 时也不能承诺同一实时执行序列；调度和 DDS 到达顺序会改变噪声使用。

适配器用固定 deadline 的实时 400 Hz 步进及 /clock，控制节点 use_sim_time=true，
初始高度 0.5 m 对齐 FSM 悬停点。电机和刚体均按同一个固定物理步积分，
姿态矩阵取当步四元数；保留单回调派发、噪声和力学方程。它不是 GUI/viewer 调度测试。
默认模拟配置无风、IMU 噪声开启、精确位置/姿态。电机时间常数和内环 PID 不由本脚本修改。

OMMPC 结果与启动方式见 `src/realflight_modules/px4ctrl/OMMPC.md`。
`--controller-label` 用于标注实验，控制器选择仍由编译宏决定。
指标中的有效滞后是轨迹平移拟合（2.5 ms 网格），不是通信延迟；接地组不报告该指标。
输出油门到限比例不是分配器内部逐电机推力裁剪比例。
`maneuver_rate_rmse_rad_s` 按消息时间保持外环输出命令，与真实机体角速度比较，三轴为 FLU；
它包含命令预补偿和反馈修正，不等同于名义轨迹角速度误差或纯通信延迟。

仿真循环退出时保存已有原始数据，运行脚本清理自己启动的子进程。初始化失败可能没有原始数组。旧 10 状态基线与首次修正结果见
`datalog/acados_debug_20260929`；旧基线需要对应旧源码/消息版本，不能由当前二进制复现。
