# AUX5 EKF2 restart over DDS

`px4ctrl_node` checks AUX5 in the main `while` loop, before the kill-switch and
arming checks. The control FSM, including `manual_on`, is unchanged.

- Every received DOWN/MID/UP transition queues one restart while RC and vehicle
  status are fresh and PX4 reports `ARMING_STATE_DISARMED`.
- The first finite AUX5 sample establishes the baseline. NaN and changes within
  the same gear do not trigger a restart.
- Edges received during startup or while armed are discarded. Arming or losing
  fresh feedback also discards queued requests; disarming never replays them.
- Requests are serialized and published once. A rejected command or a missing
  reply does not cause an automatic retry. A 35-second timeout reports an unknown
  outcome rather than claiming success.

The current `input.h` enables `SIMULATION` and `USE_WITHOUT_RC`; that build does
not send EKF restart requests. Use the project's real-flight build (disable both
macros and rebuild) for physical RC/PX4 input.

## PX4 firmware

`px4-v1.16-ekf-restart.patch` targets `/home/sun/PX4-Autopilot16` (PX4 1.16).
It adds a Commander worker operation that invokes `ekf2 stop`, then `ekf2 start`.
Commander remains responsive, rejects requests while armed, and prevents arming
during the restart using its existing calibration guard. The final ACK is sent
after the worker completes. Duplicate copies of the most recent accepted
request receive the same result without repeating the restart.

Multi-instance EKF2 does not create new instances while armed. Restarting also
interrupts estimator outputs and requires estimator initialization afterward;
an accepted command means the module restarted, not that its estimates have
already converged.

Apply to an unpatched PX4 1.16 checkout:

```bash
git -C /path/to/PX4-Autopilot16 apply --check /path/to/many_controllerV3/firmware/px4-v1.16-ekf-restart.patch
git -C /path/to/PX4-Autopilot16 apply /path/to/many_controllerV3/firmware/px4-v1.16-ekf-restart.patch
```

Compile and flash for the actual flight-controller board using the normal PX4
workflow. Building the SITL target checks compilation/linking; it does not create
or flash a hardware image. Stock firmware reports the new command as unsupported.

No DDS message fields or topic definitions change. The firmware must expose the
existing `/fmu/in/vehicle_command` and `/fmu/out/vehicle_command_ack` topics, which
are already present in this checkout's DDS configuration.

## Project-private protocol

| Field | Value |
| --- | --- |
| `VehicleCommand.command` | `100010`, a private 32-bit DDS command |
| `param1` | `4541254` (ASCII `EKF` signature) |
| `param2` | Integer request sequence, `1..16777215` |
| Source system / component | `1 / 191` |
| Target system / component | `1 / 1` |
| ACK `result_param2` | Echo of the request sequence |

The command is independent of `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. It never
requests a reboot of the autopilot or companion computer. Keep the private
command number and signature synchronized between both source trees.

## Verification

Build the controller with the ROS environment and workspace dependencies sourced:

```bash
cmake --build build/px4ctrl --target px4ctrl_node
```

PX4 reference: [EKF2 module commands](https://docs.px4.io/main/en/modules/modules_estimator#ekf2).
