# acados / MuJoCo 闭环检查

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

本脚本固定使用 6×4 m、3 m/s 八字的当前 FSM 路径；若改变轨迹，须核对分析窗口：
当前从 CMD 开始后 3 秒评估到日志中的轨迹结束，排除起飞和末尾悬停。
同一套参数下仅改变 Qp 时也不能承诺同一实时执行序列；调度和 DDS 到达顺序会改变噪声使用。

适配器用固定 deadline 的实时 400 Hz 步进及 /clock，控制节点 use_sim_time=true，
初始高度 0.5 m 对齐 FSM 悬停点。保留原 simulator 的单回调派发、实测电机 dt、
原姿态矩阵更新顺序、噪声和力学方程。它不是 GUI/viewer 调度测试。
默认模拟配置无风、IMU 噪声开启、精确位置/姿态。电机时间常数和内环 PID 不由本脚本修改。

仿真循环退出时保存已有原始数据，运行脚本清理自己启动的子进程。初始化失败可能没有原始数组。旧 10 状态基线与首次修正结果见
`datalog/acados_debug_20260929`；旧基线需要对应旧源码/消息版本，不能由当前二进制复现。
