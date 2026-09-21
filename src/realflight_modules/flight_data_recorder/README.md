# ROS 2 飞行数值 ULog 记录节点

`flight_data_recorder` 按话题消息逐条记录基本类型字段和短定长数组，生成标准 ULog v1 `.ulg` 文件，可通过 PlotJuggler 的 **ULog** 文件加载插件读取。代码默认解锁后记录；当前 YAML 开启了仿真启动即记录。没有 50 Hz 定时抽样、最新值拼接或坐标变换。`run_ctrl.launch.py` 使用这个节点；旧版 CSV 记录节点及其构建目标已删除。

## 已取消的记录内容

- `/parameter_events`：ROS 节点自动发布的参数新增、修改、删除事件，包含节点名、参数名和值。已从核心订阅移除并加入默认排除列表，自动发现也会跳过。
- `.fields/`：旧版用于存放消息数组或字符串数组的元素，例如参数列表中的每个参数。现在不再生成这些子数据集，相关数组整项省略。
- `.chunks/`：旧版用于把字符串、变长基本类型数组和超过128个元素的定长数组分块保存，每块最多128个元素。现在不再生成这些子数据集，相关字段整项省略，不保留分块数据或额外的长度字段。

普通数值、布尔值、源时间戳、128个元素以内的基本类型定长数组继续记录；直接嵌套的消息仍在主数据集中用 `__` 展开其数值字段。例如位置、姿态四元数、角速度和电机 `control[12]` 均保留。图像像素、点云字节、字符串和变长数组的内容不再保存。只有被省略字段的话题仍保留接收时间和序号。

每个主数据集的元信息 `omitted_fields` 说明省略的字段路径和原因，不保存这些字段的值。日志内 `recorder_schema_version` 为2，ULog文件格式仍为v1。旧 `.ulg` 不会随代码更新而变化。已有核查数据移至工程根目录的 `datalog/flightlog/audit/`，由 Git 忽略；其中2026-09-14的核查文件描述的是旧版日志。

## 编译与运行

在工程根目录编译；下面同时安装新包和更新后的控制启动文件：

```bash
source /opt/ros/foxy/setup.bash  # Humble 机器改为 /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select uav_utils flight_data_recorder px4ctrl --symlink-install
source install/setup.bash
ros2 launch flight_data_recorder record.launch.py
```

全新工作空间使用 `colcon build --packages-up-to flight_data_recorder --symlink-install`。常用启动方式：

```bash
# 单独记录已有 ROS 系统；自定义保存路径
ros2 launch flight_data_recorder record.launch.py log_directory:=datalog/flightlog/experiment_01

# 外环、角速度环与 ULog 记录一起启动
ros2 launch px4ctrl run_ctrl.launch.py

# 关闭 run_ctrl 中自动启动的记录节点
ros2 launch px4ctrl run_ctrl.launch.py record_data:=false

# 自定义 VehicleStatus 话题（消息类型仍须为 px4_msgs/msg/VehicleStatus）
ros2 launch flight_data_recorder record.launch.py status_topic:=/fmu/out/vehicle_status
```

默认保存到工程根目录下的 `datalog/flightlog/`。`output_directory` 和启动参数 `log_directory` 的相对路径均相对于工程根目录解析，支持显式绝对路径。路径通过本包的安装目录向上定位包含 `src/realflight_modules/px4ctrl/package.xml` 的工程根目录，因此从任意终端目录启动均使用同一个保存位置。正常安装和 `--symlink-install` 都支持；整个工程移到其他位置后重新构建并加载新的 `install/setup.bash`。若安装目录完全位于工程外，请显式提供绝对路径。

`log_directory` 留空时保留 YAML 的 `output_directory`，不再用空值覆盖配置。目录自动创建，日志、临时数据及核查结果统一位于被 Git 忽略的 `datalog/` 中。完整路径约定见工程根目录的 [数据目录说明](../../../docs/data_paths.md)。不要同时启动多个记录节点，除非确实需要多份记录。`px4_native_position.launch.py` 等其他入口可配合独立的 `record.launch.py` 使用。

## 记录边界

`simulation_mode: false` 时使用以下解锁/上锁规则（代码默认值；当前 YAML 为 `true`）：

- 启动后即建立核心订阅，并持续发现新增话题；未解锁时丢弃收到的业务数据，不缓存解锁前的数据。
- 默认以 `/fmu/out/vehicle_status_v1` 的 `arming_state` 为准。首次收到 `ARMING_STATE_ARMED=2` 即开始，每次解锁创建独立文件；节点在已解锁时启动，也会从首次收到的 ARMED 消息开始记录。
- 收到 `ARMING_STATE_DISARMED=1` 立即停止接收本次记录，工作线程写完该边界之前已排队的消息，整理文件并执行 `fsync`。解锁和上锁的两条状态消息本身都记录。
- 不依赖 Offboard、起飞、油门、外环 FSM 或轨迹状态。状态话题断流会报警，但不会猜测飞机已上锁。
- 当前 MuJoCo 节点没有发布 VehicleStatus，使用下方的 `simulation_mode: true` 即可直接记录。

边界依据本节点处理状态回调的时间。不同 ROS 话题没有统一的全局发布顺序，因此不将源时间戳解释为跨话题的严格先后关系。

## 仿真启动即记录

启动前修改 [config/recorder.yaml](config/recorder.yaml)：

```yaml
flight_data_recorder:
  ros__parameters:
    simulation_mode: true
```

只需修改现有 YAML 中的这一项，保留其他参数。`record.launch.py` 和 `px4ctrl/run_ctrl.launch.py` 都会读取它。使用 `--symlink-install` 构建后，修改源 YAML 再重启节点即可生效；普通安装需重新构建安装该包。

- `true`：节点完成订阅初始化后立即记录，无需 VehicleStatus、解锁或 FSM 条件；收到上锁消息也继续记录。每次启动对应一个日志。
- 按 Ctrl+C 正常退出**记录节点**时，写完已接收的数据并同步成完整 `.ulg`；只关闭模拟器不会结束仍在运行的记录器。飞行中仍持续写入临时文件，异常中断恢复规则不变。
- `false`：恢复上面的解锁开始、上锁结束规则。此参数在节点运行期间只读，修改后需重启。

这个开关独立于 ROS 的 `use_sim_time`。即使启用了仿真时间但 `/clock` 尚未发布或已暂停，记录与话题发现仍可进行；精确接收时间继续使用主机单调时钟。两种模式都使用同一份10项默认排除列表。

也可临时通过节点参数启用，不修改 YAML：

```bash
ros2 run flight_data_recorder flight_data_recorder_node --ros-args -p simulation_mode:=true
```

## 记录内容

以下14个核心话题预先订阅，保留基本类型字段、短定长数组、源时间戳、状态位、无效值和 NaN，字段规则见上文：

| 话题 | 消息类型 | 主要内容 |
| --- | --- | --- |
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` | 解锁、导航模式、故障保护、系统状态 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | 位置、速度、加速度、估计器有效性与复位信息 |
| `/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | 姿态四元数及复位信息 |
| `/fmu/out/sensor_combined` | `px4_msgs/msg/SensorCombined` | 陀螺仪、加速度计及采样信息 |
| `/fmu/out/battery_status` | `px4_msgs/msg/BatteryStatus` | 电压、电流、电量、温度与电池状态 |
| `/fmu/out/manual_control_setpoint` | `px4_msgs/msg/ManualControlSetpoint` | 遥控器输入与有效性 |
| `/fmu/out/vehicle_command_ack` | `px4_msgs/msg/VehicleCommandAck` | 飞控命令应答 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 发送给飞控的命令及参数 |
| `/fmu/in/actuator_motors` | `px4_msgs/msg/ActuatorMotors` | `control[0..11]` 全部 12 路及可逆标志 |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Offboard 各控制层使能 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | 飞控位置、速度、加速度、航向等设定值 |
| `/rates_thrust_setpoint` | `ratectrl_msgs/msg/RatesThrustSetpoint` | 推力、三轴角速度及角加速度参考 |
| `/debugPx4/ctrl` | `px4debug_msgs/msg/Px4ctrlDebug` | 外环状态、参考量、期望量、电压、`thr2acc` |
| `/debugPx4/ratectrl` | `px4debug_msgs/msg/Px4ratectrlDebug` | 角速度环全部调试数据 |

这不是白名单。默认每 0.5 秒检查 ROS graph，只要有发布者、当前环境可加载其 ROS 消息类型且不在排除列表，就自动加入记录。额外传感器、规划器等新话题也按同样的字段规则处理。ROS 服务调用本身不属于话题；Action 的可见话题会按相同规则发现。

默认精确排除以下10个话题，不做前缀排除：

```text
/joyStick
/realflight/trajectory_markers
/omtraj/markers
/minco/markers
/minco/trajectory
/tf
/tf_static
/clock
/rosout
/parameter_events
```

记录范围是**成功订阅后、记录期间实际收到的消息中的受支持字段**。新话题在发现并完成 DDS 匹配前的样本无法补录；启动前的数据同样无法回溯。核心话题预先创建订阅以缩短这个窗口。飞行前先启动记录器，并检查 `Recording source registered` 日志。

## thr2acc

原样记录 `/debugPx4/ctrl.thr2acc`，在 PlotJuggler 搜索 `msg_thr2acc`。它来自控制器实际发布的调试消息。

工程的 `QuadControl` 在 `controller.cpp` 中更新、发布该值。当前 OM-MPC 和 NMPC 实现没有执行同样的 `thr2acc` 在线估计，所以切换到这些控制器时，记录值可能一直为默认值 0；记录器无法从一个未计算、未发布的内部量中获得变化。此改动不引入新的估计算法。

## ULog 字段与时间轴

- 话题 `/debugPx4/ctrl` 对应数据集 `ros/debugPx4/ctrl`。原字段加 `msg_` 前缀，例如 `msg_thr2acc`、`msg_timestamp`。嵌套消息用 `__` 展平。
- 128 个元素以内的基本类型定长数组直接保留为 ULog 数组，例如 pyulog 中的 `msg_control[11]`；本机 PlotJuggler 3.17.2 将它显示为 `msg_control.11`。更长数组、变长数组、非基本类型数组和字符串省略。
- 每个话题/类型只生成一个主数据集，不再生成 `.fields/`、`.chunks/` 或 `recorder_index_N`。`recorder_sequence` 是记录器为该来源分配的消息序号。
- 主时间轴 `timestamp` 是主机单调时钟微秒。`recorder_monotonic_ns` 保存精确接收时间，`recorder_ros_time_ns` 保存 ROS 时钟接收时间；原消息的时间戳保持不变。即使 ROS 时间回跳，曲线仍可按接收顺序分析。
- 同一数据集内，微秒时间戳重合的消息会顺延 1 微秒以兼容绘图；精确接收时间与序号不变。
- ULog 元信息保存源话题、ROS 类型及递归字段定义、字段映射和省略列表、部分配置、每话题接收/写入/丢弃计数和错误。64 位整数在 ULog 内保持原类型与精度；PlotJuggler 数值曲线使用 double，超过其精确整数范围的数值可用 pyulog 读取原整数。

文件遵循 [PX4 ULog 格式](https://docs.px4.io/main/en/dev_log/ulog_file_format)，数据集是本工程的 ROS 消息映射。通过 PlotJuggler 的“加载数据文件”选择 `.ulg` 和 ULog 插件，不需要 CSV 转换或自定义 PlotJuggler 插件。

## 落盘与异常

记录期间，后台工作线程持续写入 `flight_日期_时间_唯一标识.ulg.pending/` 内的 `definitions.bin` 和 `data.bin`，默认每秒 flush/fsync，无需把整个会话留在内存中。默认模式下上锁、仿真模式下正常退出，都会立即排队结束本次记录；完成排队数据写入、合并及磁盘同步后，才产生最终 `.ulg`。完成时间取决于文件大小和磁盘速度，不承诺零耗时。

分开暂存格式定义与数据，是为了保证动态话题的定义也出现在最终文件数据段之前，兼容 pyulog 和 PlotJuggler。整理时需要约一份额外日志大小的磁盘空间。最终文件通过原子链接发布，不覆盖已有日志。

- 正常上锁完成，或仿真模式正常退出：`.ulg`。
- 按上述字段规则主动省略的内容不算丢包，也不会使日志标记为不完整；完整性检查针对记录范围内的数据。
- 队列溢出、已知 DDS 丢包、无法加载/编码消息、或默认模式下解锁时退出节点：告警并标记 `.incomplete.ulg`，元信息包含原因与计数。仿真模式也保留数据丢失和错误检查。
- 磁盘写入失败：明确报错，保留尚可恢复的 `.ulg.pending`，不报告成功。
- 进程被强制终止或断电：已同步的临时数据可恢复；最后一次同步后的数据可能丢失。

恢复**已停止运行**的记录会话：

```bash
ros2 run flight_data_recorder recover_ulog /path/to/flight_....ulg.pending
```

生成 `.recovered.ulg`，只保留格式完整、可解析的数据包并添加恢复标记。运行中的文件被锁保护，不能被恢复命令提前移走。

默认队列字节上限 64 MiB、订阅深度 4096。超限时逐话题计数并标记不完整。DDS 使用 volatile，避免重放上锁前的保留样本；可确定全部发布者提供 reliable 时请求 reliable，其余使用兼容 PX4 的 best effort。DDS 事件只反映中间件能报告的丢失，无法证明网络中从未发生丢包。字段筛选在后台解码后执行，省略图像等变长字段不会消除其订阅、原始消息排队和解码开销；不需要的话题应加入排除列表。

参数见 [config/recorder.yaml](config/recorder.yaml)：`output_directory`、`simulation_mode`、`status_topic`、`auto_discover`、`discovery_interval_s`、`subscription_depth`、`max_queue_bytes`、`sync_interval_s`、`excluded_topics`。

## Foxy / Humble 订阅事件兼容

- Foxy 的 `SubscriptionEventCallbacks` 没有 `message_lost` 参数。节点按当前接口检测能力，不向旧版传入这个参数；Humble 支持时继续启用丢包事件监测。
- `message_lost` 和 `incompatible_qos` 分别尝试注册。若当前 RMW 不支持其中一个事件，只禁用该事件，其他监测和原始消息订阅继续工作。
- 缺少监测能力输出一次 warning，不再输出 `Cannot record`，也不把它当作实际丢包而误标 `.incomplete.ulg`。真正收到丢包或 QoS 不兼容事件时，仍报告错误并标记记录期间的异常。
- ULog 的 `recorder_metadata.subscription_events` 保存开始记录时各订阅的监测能力；`recorder_summary.subscription_events` 保存结束时的能力，包含飞行途中新增的话题。每项包含 `enabled`，禁用时还包含 `reason`。
- `Recording source registered` 只表示订阅创建成功，不表示已经收到消息；无法监测 DDS 丢包也不代表没有丢包。

## 验证

运行时不依赖 pyulog；测试需要 `pytest`、`numpy`、`pyulog`，缺少时用 `python3 -m pip install -r src/realflight_modules/flight_data_recorder/test/requirements.txt` 安装。独立解析器测试覆盖核心消息和定长数组保留、字符串/长数组/消息数组省略、直接嵌套数值字段展开、64位整数、时间回跳、状态边界、重复解锁、异常与恢复：

```bash
source install/setup.bash
python3 -m pytest -q src/realflight_modules/flight_data_recorder/test
```

实际 ROS 话题测试默认跳过，可在隔离域中启用；它不启动控制器或给真实飞控发命令：

```bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 FLIGHT_RECORDER_ROS_TEST=1 \
  python3 -m pytest -q -s src/realflight_modules/flight_data_recorder/test/test_ros_integration.py
```

该测试发送400 Hz外环调试与400 Hz电机消息，对比每个 `thr2acc` 样本和全部12路电机值，并验证新增话题、包含 `/parameter_events` 的10项排除、无子数据集、两次飞行分别生成文件。仿真测试通过 YAML 启用模式，在没有 VehicleStatus 和 `/clock` 发布者时验证 ROS 话题记录，并单独启动记录进程检查 Ctrl+C 后的完整 ULog。话题测试在同一进程内运行两个 ROS 节点；跨进程 DDS 发现还取决于运行环境的网络配置。`test/plotjuggler_probe.cpp` 可链接本机 PlotJuggler 和 Qt 库，通过未修改的 `libDataLoadULog.so` 读取测试输出，进一步核对曲线样本和值。
