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
- 发送后的 5 秒为应答等待及防重复触发间隔。期间的开关变化全部丢弃，
  收到 ACCEPTED 或拒绝应答也不会提前结束该间隔。
- ACCEPTED 只打印“PX4 已接受重启命令，等待重新连接”，不声称重启已完成。
  PX4 重启可能使 DDS 在 ACK 到达前断开；5 秒没有 ACK 时报告结果未知，不自动重发。
- 间隔结束后仍需一次新的、满足未解锁和输入新鲜条件的 AUX5 变化才能再次发送。
  本客户端不自动判断 PX4 是否已经完成启动、EKF 是否已经收敛。

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

PX4 的 `dds_topics.yaml` 中 `vehicle_status` 使用无版本基础名称，DDS 生成代码
根据消息 `MESSAGE_VERSION` 添加后缀；不要直接照上述实际话题名修改 YAML。
本机 `/home/sun/PX4-Autopilot16` 已包含这四类接口。
PX4 的 system/component ID 应与上表目标一致；`RC_MAP_AUX5` 应映射到实际开关通道。
不支持硬件重启的 SITL 构建可能返回 DENIED。

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

- `src/realflight_modules/px4ctrl/include/px4ctrl/fcu_reboot.h`：客户端声明。
- `src/realflight_modules/px4ctrl/src/fcu_reboot.cpp`：DDS 命令、应答与防重复触发。
- `src/realflight_modules/px4ctrl/src/px4ctrl_node.cpp`：主循环 AUX5 接入。
- `src/realflight_modules/px4ctrl/CMakeLists.txt`：编译源文件名称。

命令定义：[MAVLink MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN](https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)。
