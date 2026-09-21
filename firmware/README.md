# AUX5 通过 ROS2 DDS 重启 PX4

`px4ctrl_node` 在主循环中检测 AUX5 挡位变化，通过
`/fmu/in/vehicle_command` 发送标准 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)`，
请求重启整个 PX4 飞控。伴随计算机继续运行。

## 命令和 ROS1 示例的对应关系

| ROS2 `px4_msgs::msg::VehicleCommand` 字段 | 值与含义 |
| --- | --- |
| `command` | `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN`，即 246 |
| `param1` | `1.0`，重启 autopilot |
| `param2` | `0.0`，不重启 onboard computer |
| `param3` 到 `param7` | `0` |
| `target_system / target_component` | `1 / 1`，定向发送，对应 ROS1 示例的 `broadcast=false` |
| `source_system / source_component` | `1 / 191`，用于匹配应答目标 |
| `confirmation` | `0`，首次发送；该字段是重发标记，不是“是否要求应答”的布尔开关 |
| `from_external` | `true` |

实现使用 `rclcpp` 发布消息并订阅 `/fmu/out/vehicle_command_ack`，
不使用 ROS1 的 `mavros_msgs::CommandLong` 或 `reboot_FCU_srv`。
标准 ACK 按命令号、目标 system/component 匹配，不使用旧私有协议的序号回显。

## 触发和应答行为

- 只有遥控、飞控状态新鲜，并且 PX4 为 `ARMING_STATE_DISARMED` 时才发送。
  检查位于控制 FSM、kill-switch 和解锁门控之前，未解锁时也能处理 AUX5。
- 首个有限 AUX5 样本只建立初始挡位；同挡位和 NaN 不触发。
  启动等待期间及不满足条件时的变化被消耗，不在之后补发。
- 一次主循环收到多个挡位变化时只发送一条重启命令；没有请求队列。
- 发送前还要观察到状态、姿态两路各至少两次不同时间戳，且最近一次更新距今不足 1 秒，
  确保能从正常收数状态开始监测重启。条件不满足时本次 AUX5 变化被消耗，需要重新拨动。
- 5 秒内没有 ACK 时打印 `ACK_TIMEOUT`，继续监测数据恢复。ACK 可能随重启断链而丢失，
  不把没有 ACK 当作重启失败，也不自动重发命令。
- ACCEPTED 只表示命令被接受。收到拒绝应答时结束恢复监测，仍保留发送后至少 5 秒的防重复间隔。
- 从发出命令到恢复确认期间的 AUX5 变化全部丢弃，不排队、不补发。
  恢复确认后也保留发送后至少 5 秒的间隔；间隔结束的那次循环仍不处理新的开关变化。

## DDS 恢复指示

在运行 `px4ctrl_node` 的终端查看 `[FCU_REBOOT]` 日志：

| 标记 | 条件与含义 |
| --- | --- |
| `REQUESTED` | 已发布一次重启命令，等待数据中断 |
| `ACCEPTED` | 收到 PX4 接受命令的 ACK，还未确认恢复 |
| `DISCONNECTED` | 发令至少 1 秒后，状态、姿态两路都至少 1 秒没有新时间戳，开始等待恢复 |
| `VERIFYING` | 中断后两路都已重新更新，开始连续 2 秒稳定性检查 |
| `UNSTABLE` | 检查期间任一路间隔达到 1 秒或源时间戳倒退，重新进行稳定性检查 |
| **`DDS_RECONNECTED`（绿色）** | 两路恢复后持续更新至少 2 秒，且各自在检查起点后又更新至少两次，确认 DDS 下行数据恢复 |
| `NOT_CONFIRMED` | 发令后 10 秒仍未观察到两路共同中断，不能确认重启／重连；结束本次监测，不自动重发 |
| `RECONNECT_TIMEOUT` | 观察到中断后 60 秒仍未确认稳定恢复，打印两路数据的新鲜状态；继续监测迟到的恢复 |
| `REJECTED` | PX4 明确拒绝或执行失败 |

判断直接复用主节点已订阅的 `/fmu/out/vehicle_status_v1` 和
`/fmu/out/vehicle_attitude`，不新增订阅或 ROS 消息类型。
每次主循环只比较两个缓存的源 `timestamp` 是否变化；非零且变化才记为一次更新，
重复读取缓存或重复时间戳不刷新存活时间。中断、超时和稳定时间使用
`std::chrono::steady_clock`，不依赖 ROS 时间跳变或重启后 PX4 时间戳是否归零。
时间戳倒退可以发生在重启／时间同步后，允许恢复观察，但会重开稳定性检查。

这个绿色指示确认的是**观察到中断后，两路 PX4 → ROS2 数据持续恢复**。
它不是飞控启动身份认证，不能单凭它证明一定发生了重启；普通网络断开再恢复也能呈现同样现象。
它也不确认 ROS2 → PX4 指令方向、其他所有话题、EKF 收敛或可解锁状态。
短于检测阈值的中断可能无法识别，此时保守地报告未确认，而不会仅根据 ACK 显示成功。

若以太网／DDS 没有恢复，客户端只提示超时，不会自动重启 Agent、改网络设置或重复重启飞控。
保持控制节点运行，后续手动处理网络或给飞控重新上电后，只要两路数据稳定恢复，仍会给出绿色指示。
监测期间只有固定数量的比较与计时，没有等待循环、阻塞操作或持续打印。

## PX4 端配置

本机 PX4 1.16 的 `src/modules/commander/Commander.cpp` 已有标准命令处理：
`param1=1`、未解锁且硬件支持 `CONFIG_BOARDCTL_RESET` 时，调用
`px4_reboot_request(REBOOT_REQUEST, 400_ms)`，发送 ACCEPTED 后等待重启。
此功能使用 PX4 已有实现，旧 EKF 专用 patch 已从项目移除。

需要已有 DDS 配置导出以下接口：

- `/fmu/in/vehicle_command`
- `/fmu/out/vehicle_command_ack`
- `/fmu/out/manual_control_setpoint`
- `/fmu/out/vehicle_status_v1`（本项目当前订阅的实际版本名）
- `/fmu/out/vehicle_attitude`（用于恢复确认）

PX4 的 `dds_topics.yaml` 中 `vehicle_status` 使用无版本基础名称，DDS 生成代码
根据消息 `MESSAGE_VERSION` 添加后缀；不要直接照上述实际话题名修改 YAML。
本机 `/home/sun/PX4-Autopilot16` 已包含这些接口。
PX4 的 system/component ID 应与上表目标一致；`RC_MAP_AUX5` 应映射到实际开关通道。
不支持硬件重启的 SITL 构建可能返回 DENIED。

## 有哪些重启方式，当前命令是否正确

本机 PX4 1.16 的 Commander 对命令 246 的处理如下；实际机载固件支持情况仍以其版本和板级配置为准：

| `command=246` 的 `param1` | 动作 | 本机 PX4 实现条件 |
| --- | --- | --- |
| `0` | 无动作 | 返回 ACCEPTED，不表示真的重启了 |
| `1` | 正常重启整个飞控，当前项目使用此项 | 未解锁，且支持 `CONFIG_BOARDCTL_RESET` |
| `2` | 关机 | 未解锁，且支持 `BOARD_HAS_POWER_CONTROL`；不会自动重新上电 |
| `3` | 重启进入 bootloader | 未解锁，且支持硬件重启；用于固件维护，不能作为恢复正常 DDS 的替代项 |

当前 MAVLink 规范另有动作 `4`（上电）和 `5`（重启进入 USB 大容量存储模式），
但本机 PX4 1.16 这个处理分支没有实现它们，不能仅凭 MAVLink 枚举认为机载固件支持。
本机该分支只按 `param1` 分派动作；把 `param2` 改成 `1` 并不会自动重启 Manifold，
伴随计算机动作还需要有对应的接收处理程序。

PX4 命令行的 `reboot` 和 `246 / param1=1` 最终都走
`px4_reboot_request(REBOOT_REQUEST, ...)`，差别包括后者延迟 400 ms 以便应答。
`reboot -b` 进入 bootloader；只有支持 ISP bootloader 的板子才支持 `reboot -i`。
单独停止并按原启动参数重新启动 `uxrce_dds_client` 只重启 DDS 客户端；
在 Manifold 上重启 MicroXRCEAgent 只重启代理进程，都不是重启整个飞控。
给飞控断电再上电会经历供电域复位，不等同于软件重启；上述命令没有通用的“全部硬件断电再上电”选项。

因此 `command=246, param1=1, param2=0, confirmation=0` 的选择是正确的。
`confirmation` 是首次发送／重发标记，不是 ROS1 示例中看起来像布尔值的“要求 ACK”。
QGC 能连接而 ROS2 没有数据说明需继续区分 Ethernet、PX4 DDS 客户端、Agent 会话和 ROS2 端状态，
现有信息不能确定是哪一层未恢复，不能靠改成 bootloader 或 shutdown 来解决。

## 项目编译及使用

实物版本需要在 `src/realflight_modules/px4ctrl/include/px4ctrl/input.h` 中关闭
`SIMULATION` 和 `USE_WITHOUT_RC` 后重新编译。当前仓库仍保留原仿真宏配置，
本次功能替换没有切换整个项目的运行模式。

在 ROS2 及工作区依赖环境已加载时：

```bash
colcon build --packages-select px4ctrl
source install/setup.bash
```

启动既有控制流程，确保节点已完成启动等待并进入主循环，再在飞控未解锁时切换 AUX5。
重启期间 PX4 状态与估计输出会中断，重新启动后应重新确认状态和参考坐标。

## 文件位置

- `src/realflight_modules/px4ctrl/include/px4ctrl/fcu_reboot.h`：客户端声明、恢复阶段和两路数据的观测状态。
- `src/realflight_modules/px4ctrl/src/fcu_reboot.cpp`：DDS 命令、应答、恢复检测、超时提示与防重复触发。
- `src/realflight_modules/px4ctrl/src/px4ctrl_node.cpp`：主循环 AUX5 接入，并传入已有状态／姿态缓存时间戳。
- `firmware/README.md`：本说明，包括指示判据、限制与命令选项。

本次恢复指示只修改上述四个文件。原有 CMake 源文件注册无需调整。

命令定义：[MAVLink MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN](https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)。
PX4 参考：[v1.16 Commander](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/commander/Commander.cpp)、
[reboot 命令](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/systemcmds/reboot/reboot.cpp)。
