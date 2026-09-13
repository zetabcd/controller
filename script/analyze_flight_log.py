#!/usr/bin/env python3
"""Analyze and visualize a CSV produced by flight_data_recorder_node."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Dict, Iterable, List, Sequence, Tuple


Row = Dict[str, str]


def finite(value: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def load_rows(path: Path, state: int | None) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("CSV中没有数据")
    required = {
        "elapsed_s", "state", "position_error_norm", "velocity_error_norm",
        "actual_q_w", "actual_q_x", "actual_q_y", "actual_q_z",
        "des_q_w", "des_q_x", "des_q_y", "des_q_z",
        "gyro_x", "gyro_y", "gyro_z",
        "des_rate_x", "des_rate_y", "des_rate_z", "des_thrust_n",
    }
    missing = required.difference(rows[0])
    if missing:
        raise ValueError(f"CSV缺少字段: {', '.join(sorted(missing))}")
    if state is not None:
        selected = [row for row in rows if finite(row["state"]) == state]
        if not selected:
            available = ", ".join(sorted({row["state"] for row in rows}))
            raise ValueError(
                f"CSV共{len(rows)}行，没有state={state}的记录"
                f"（现有状态: {available}）；可使用 --state all 分析其他状态"
            )
        rows = selected
    valid_rows = [
        row for row in rows
        if math.isfinite(finite(row["position_error_norm"]))
        and row.get("position_valid", "1") == "1"
    ]
    if not valid_rows:
        invalid_position = sum(
            row.get("position_valid", "1") != "1" for row in rows
        )
        invalid_error = sum(
            not math.isfinite(finite(row["position_error_norm"])) for row in rows
        )
        scope = "全部状态" if state is None else f"state={state}"
        raise ValueError(
            f"{scope}共{len(rows)}行，但没有有效位置数据："
            f"position_valid不为1的有{invalid_position}行，"
            f"position_error_norm缺失或非有限值的有{invalid_error}行"
            "（两项可能重叠）。--state all 只取消状态筛选，"
            "不会跳过数据有效性检查；请检查日志的位置有效标志和记录来源"
        )
    return valid_rows


def series(rows: Sequence[Row], name: str) -> List[float]:
    return [finite(row.get(name, "nan")) for row in rows]


def norm_series(rows: Sequence[Row], names: Sequence[str]) -> List[float]:
    result = []
    for row in rows:
        values = [finite(row.get(name, "nan")) for name in names]
        result.append(
            math.sqrt(sum(value * value for value in values))
            if all(math.isfinite(value) for value in values) else math.nan
        )
    return result


def metric(values: Iterable[float]) -> Dict[str, float]:
    valid = [value for value in values if math.isfinite(value)]
    if not valid:
        return {
            "rmse": math.nan, "mean": math.nan, "max": math.nan,
            "p95": math.nan,
        }
    ordered = sorted(valid)
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "rmse": math.sqrt(fmean(value * value for value in valid)),
        "mean": fmean(valid),
        "max": max(valid),
        "p95": ordered[p95_index],
    }


def error_metric(values: Iterable[float]) -> Dict[str, float]:
    valid = [value for value in values if math.isfinite(value)]
    if not valid:
        return {
            "rmse": math.nan, "bias": math.nan, "mean_abs": math.nan,
            "max_abs": math.nan, "p95_abs": math.nan,
        }
    absolute = sorted(abs(value) for value in valid)
    p95_index = min(len(absolute) - 1, math.ceil(0.95 * len(absolute)) - 1)
    return {
        "rmse": math.sqrt(fmean(value * value for value in valid)),
        "bias": fmean(valid),
        "mean_abs": fmean(absolute),
        "max_abs": absolute[-1],
        "p95_abs": absolute[p95_index],
    }


def range_metric(values: Iterable[float]) -> Dict[str, float]:
    valid = [value for value in values if math.isfinite(value)]
    if not valid:
        return {
            "mean": math.nan, "min": math.nan, "max": math.nan,
            "peak_to_peak": math.nan,
        }
    return {
        "mean": fmean(valid), "min": min(valid), "max": max(valid),
        "peak_to_peak": max(valid) - min(valid),
    }


def normalize_quaternion(values: Sequence[float]) -> Tuple[float, ...] | None:
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        return None
    length = math.sqrt(sum(value * value for value in values))
    if length < 1.0e-9:
        return None
    return tuple(value / length for value in values)


def quaternion_to_euler(
    quaternion: Sequence[float]
) -> Tuple[float, float, float]:
    w, x, y, z = quaternion
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sine)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def attitude_series(rows: Sequence[Row]) -> Dict[str, List[float]]:
    result = {
        "actual_roll_deg": [], "actual_pitch_deg": [], "actual_yaw_deg": [],
        "desired_roll_deg": [], "desired_pitch_deg": [], "desired_yaw_deg": [],
        "error_roll_deg": [], "error_pitch_deg": [], "error_yaw_deg": [],
        "error_angle_deg": [],
    }
    actual_names = ("actual_q_w", "actual_q_x", "actual_q_y", "actual_q_z")
    desired_names = ("des_q_w", "des_q_x", "des_q_y", "des_q_z")
    for row in rows:
        actual = normalize_quaternion(
            [finite(row.get(name, "nan")) for name in actual_names]
        )
        desired = normalize_quaternion(
            [finite(row.get(name, "nan")) for name in desired_names]
        )
        if actual is None or desired is None:
            for values in result.values():
                values.append(math.nan)
            continue
        actual_euler = quaternion_to_euler(actual)
        desired_euler = quaternion_to_euler(desired)
        euler_error = [
            wrap_angle(actual_euler[index] - desired_euler[index])
            for index in range(3)
        ]
        dot = min(1.0, abs(sum(a * b for a, b in zip(actual, desired))))
        angle_error = 2.0 * math.acos(dot)
        for axis, index in zip(("roll", "pitch", "yaw"), range(3)):
            result[f"actual_{axis}_deg"].append(
                math.degrees(actual_euler[index]))
            result[f"desired_{axis}_deg"].append(
                math.degrees(desired_euler[index]))
            result[f"error_{axis}_deg"].append(
                math.degrees(euler_error[index]))
        result["error_angle_deg"].append(math.degrees(angle_error))
    return result


def subtract_series(lhs: Sequence[float], rhs: Sequence[float]) -> List[float]:
    return [
        left - right
        if math.isfinite(left) and math.isfinite(right) else math.nan
        for left, right in zip(lhs, rhs)
    ]


def vector_error(
    rows: Sequence[Row], actual_prefix: str, desired_prefix: str
) -> Dict[str, List[float]]:
    result = {}
    for axis in "xyz":
        result[axis] = subtract_series(
            series(rows, f"{actual_prefix}{axis}"),
            series(rows, f"{desired_prefix}{axis}"),
        )
    result["norm"] = [
        math.sqrt(sum(result[axis][index] ** 2 for axis in "xyz"))
        if all(math.isfinite(result[axis][index]) for axis in "xyz")
        else math.nan
        for index in range(len(rows))
    ]
    return result


def differences(values: Sequence[float]) -> List[float]:
    result = [math.nan]
    for current, previous in zip(values[1:], values[:-1]):
        result.append(
            current - previous
            if math.isfinite(current) and math.isfinite(previous) else math.nan
        )
    return result


def slew_rate(values: Sequence[float], times: Sequence[float]) -> List[float]:
    delta = differences(values)
    result = [math.nan]
    for index in range(1, len(values)):
        dt = times[index] - times[index - 1]
        result.append(delta[index] / dt if dt > 1.0e-6 else math.nan)
    return result


def total_variation(values: Sequence[float]) -> float:
    delta = differences(values)
    return sum(abs(value) for value in delta if math.isfinite(value))


def normalized_differences(values: Sequence[float]) -> List[float]:
    valid = clean(values)
    if not valid:
        return []
    scale = max(valid) - min(valid)
    if scale < 1.0e-12:
        return [0.0 for _ in values[1:]]
    return [value / scale for value in clean(differences(values))]


def summarize(
    rows: Sequence[Row], source: Path, state: int | None,
    attitude: Dict[str, List[float]], rate_error: Dict[str, List[float]],
) -> Dict:
    times = series(rows, "elapsed_s")
    axis_position = {
        axis: error_metric(series(rows, f"position_error_{axis}"))
        for axis in "xyz"
    }
    axis_velocity = {
        axis: error_metric(series(rows, f"velocity_error_{axis}"))
        for axis in "xyz"
    }
    attitude_error = {
        axis: error_metric(attitude[f"error_{axis}_deg"])
        for axis in ("roll", "pitch", "yaw")
    }
    attitude_error["three_dimensional_angle"] = metric(
        attitude["error_angle_deg"])
    body_rate_error = {
        axis: error_metric(rate_error[axis]) for axis in "xyz"
    }
    body_rate_error["norm"] = metric(rate_error["norm"])

    outputs = {
        "thrust_n": series(rows, "des_thrust_n"),
        "body_rate_x_radps": series(rows, "des_rate_x"),
        "body_rate_y_radps": series(rows, "des_rate_y"),
        "body_rate_z_radps": series(rows, "des_rate_z"),
    }
    for index in range(1, 5):
        outputs[f"motor_thrust_{index}_n"] = series(
            rows, f"motor_thrust_{index}")
        outputs[f"motor_rpm_{index}"] = series(rows, f"motor_rpm_{index}")
        outputs[f"actuator_{index}"] = series(rows, f"actuator_{index}")

    controller_output = {
        name: range_metric(values) for name, values in outputs.items()
    }
    control_variation = {}
    for name, values in outputs.items():
        delta = differences(values)
        slew = slew_rate(values, times)
        control_variation[name] = {
            "sample_delta": error_metric(delta),
            "slew_rate": error_metric(slew),
            "total_variation": total_variation(values),
        }

    duration = max(times) - min(times)
    age_fields = (
        "position_age_s", "attitude_age_s", "imu_age_s", "debug_age_s"
    )
    data_quality = {
        name: metric(series(rows, name)) for name in age_fields
    }
    data_quality["effective_record_rate_hz"] = (
        (len(rows) - 1) / duration if duration > 0.0 else math.nan)
    return {
        "source": str(source.resolve()),
        "state_filter": "all" if state is None else state,
        "samples": len(rows),
        "duration_s": duration,
        "position_error_m": {
            "norm": metric(series(rows, "position_error_norm")),
            "axes": axis_position,
        },
        "velocity_error_mps": {
            "norm": metric(series(rows, "velocity_error_norm")),
            "axes": axis_velocity,
        },
        "attitude_error_deg": attitude_error,
        "body_rate_error_radps": body_rate_error,
        "controller_output": controller_output,
        "control_variation": control_variation,
        "solver_time_ms": metric(series(rows, "rate_solve_time_ms")),
        "data_quality": data_quality,
    }


def prepare_time(rows: Sequence[Row]) -> List[float]:
    values = series(rows, "elapsed_s")
    return [value - values[0] for value in values]


def plot_overview(
    rows: Sequence[Row], attitude: Dict[str, List[float]],
    rate_error: Dict[str, List[float]], output: Path,
) -> None:
    import matplotlib.pyplot as plt

    time = prepare_time(rows)
    actual = {axis: series(rows, f"actual_p_{axis}") for axis in "xyz"}
    reference = {axis: series(rows, f"ref_p_{axis}") for axis in "xyz"}
    colors = {"x": "tab:blue", "y": "tab:orange", "z": "tab:green"}
    figure = plt.figure(figsize=(16, 15), constrained_layout=True)
    grid = figure.add_gridspec(4, 2)

    trajectory = figure.add_subplot(grid[0, 0], projection="3d")
    trajectory.plot(reference["x"], reference["y"], reference["z"],
                    "r--", label="reference")
    trajectory.plot(actual["x"], actual["y"], actual["z"],
                    "g", label="actual")
    trajectory.set(xlabel="x forward [m]", ylabel="y left [m]",
                   zlabel="z up [m]", title="3D trajectory")
    trajectory.legend()

    position_plot = figure.add_subplot(grid[0, 1])
    for axis in "xyz":
        position_plot.plot(time, actual[axis], color=colors[axis],
                           label=f"actual {axis}")
        position_plot.plot(time, reference[axis], "--", color=colors[axis],
                           alpha=0.7, label=f"ref {axis}")
    position_plot.set(xlabel="time [s]", ylabel="position [m]",
                      title="Position tracking")
    position_plot.legend(ncol=2, fontsize=8)

    position_error_plot = figure.add_subplot(grid[1, 0])
    for axis in "xyz":
        position_error_plot.plot(
            time, series(rows, f"position_error_{axis}"),
            color=colors[axis], label=f"e{axis}")
    position_error_plot.plot(time, series(rows, "position_error_norm"),
                             "k", linewidth=1.5, label="norm")
    position_error_plot.set(xlabel="time [s]", ylabel="error [m]",
                            title="Position error")
    position_error_plot.legend()

    attitude_tracking_plot = figure.add_subplot(grid[1, 1])
    for axis in ("roll", "pitch", "yaw"):
        attitude_tracking_plot.plot(
            time, attitude[f"actual_{axis}_deg"], label=f"actual {axis}")
        attitude_tracking_plot.plot(
            time, attitude[f"desired_{axis}_deg"], "--",
            label=f"desired {axis}")
    attitude_tracking_plot.set(
        xlabel="time [s]", ylabel="angle [deg]", title="Attitude tracking")
    attitude_tracking_plot.legend(ncol=2, fontsize=8)

    attitude_plot = figure.add_subplot(grid[2, 0])
    for axis in ("roll", "pitch", "yaw"):
        attitude_plot.plot(time, attitude[f"error_{axis}_deg"], label=axis)
    attitude_plot.plot(time, attitude["error_angle_deg"], "k",
                       linewidth=1.4, label="3D angle")
    attitude_plot.set(xlabel="time [s]", ylabel="error [deg]",
                      title="Attitude tracking error")
    attitude_plot.legend()

    rate_tracking_plot = figure.add_subplot(grid[2, 1])
    for axis in "xyz":
        rate_tracking_plot.plot(
            time, series(rows, f"gyro_{axis}"), label=f"actual {axis}")
        rate_tracking_plot.plot(
            time, series(rows, f"des_rate_{axis}"), "--",
            label=f"desired {axis}")
    rate_tracking_plot.set(
        xlabel="time [s]", ylabel="rate [rad/s]",
        title="Body-rate tracking")
    rate_tracking_plot.legend(ncol=2, fontsize=8)

    rate_plot = figure.add_subplot(grid[3, 0])
    for axis in "xyz":
        rate_plot.plot(time, rate_error[axis], color=colors[axis],
                       label=f"e{axis}")
    rate_plot.plot(time, rate_error["norm"], "k", linewidth=1.4,
                   label="norm")
    rate_plot.set(xlabel="time [s]", ylabel="error [rad/s]",
                  title="Body-rate tracking error")
    rate_plot.legend()

    quality_plot = figure.add_subplot(grid[3, 1])
    for name, label in (
        ("position_age_s", "position"), ("attitude_age_s", "attitude"),
        ("imu_age_s", "IMU"), ("debug_age_s", "controller debug"),
    ):
        quality_plot.plot(time, series(rows, name), label=label)
    quality_plot.set(xlabel="time [s]", ylabel="message age [s]",
                     title="Input freshness")
    quality_plot.legend()
    for axes in figure.axes:
        axes.grid(True, alpha=0.3)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_controls(rows: Sequence[Row], output: Path) -> None:
    import matplotlib.pyplot as plt

    time = prepare_time(rows)
    thrust = series(rows, "des_thrust_n")
    rates = {axis: series(rows, f"des_rate_{axis}") for axis in "xyz"}
    figure, axes = plt.subplots(
        4, 2, figsize=(16, 15), constrained_layout=True)

    axes[0, 0].plot(time, thrust, color="tab:purple")
    axes[0, 0].set(title="Collective thrust", ylabel="N")
    for axis in "xyz":
        axes[0, 1].plot(time, rates[axis], label=axis)
    axes[0, 1].set(title="Commanded body rates", ylabel="rad/s")
    axes[0, 1].legend()

    for index in range(1, 5):
        axes[1, 0].plot(time, series(rows, f"motor_thrust_{index}"),
                        label=f"motor {index}")
        axes[1, 1].plot(time, series(rows, f"motor_rpm_{index}"),
                        label=f"motor {index}")
    axes[1, 0].set(title="Allocated motor thrust", ylabel="N")
    axes[1, 1].set(title="Desired motor speed", ylabel="RPM")
    axes[1, 0].legend(ncol=2)
    axes[1, 1].legend(ncol=2)

    axes[2, 0].plot(time, differences(thrust), color="tab:purple")
    axes[2, 0].set(title="Thrust sample-to-sample change", ylabel="delta N")
    for axis in "xyz":
        axes[2, 1].plot(
            time, differences(rates[axis]), label=f"delta rate {axis}")
    axes[2, 1].set(
        title="Body-rate sample-to-sample changes", ylabel="delta rad/s")
    axes[2, 1].legend(fontsize=8)
    for index in range(1, 5):
        axes[3, 0].plot(time, series(rows, f"actuator_{index}"),
                        label=f"actuator {index}")
    axes[3, 0].set(title="PX4 actuator outputs", ylabel="normalized output")
    axes[3, 0].legend(ncol=2)
    axes[3, 1].plot(
        time, series(rows, "rate_solve_time_ms"), color="tab:brown")
    axes[3, 1].set(title="Rate-controller solve time", ylabel="ms")
    for plot_axes in axes.flat:
        plot_axes.set_xlabel("time [s]")
        plot_axes.grid(True, alpha=0.3)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def clean(values: Iterable[float]) -> List[float]:
    return [value for value in values if math.isfinite(value)]


def plot_boxplots(
    rows: Sequence[Row], attitude: Dict[str, List[float]],
    rate_error: Dict[str, List[float]], output: Path,
) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(
        2, 2, figsize=(14, 10), constrained_layout=True)
    axes[0, 0].boxplot(
        [clean(series(rows, f"position_error_{axis}")) for axis in "xyz"] +
        [clean(series(rows, "position_error_norm"))],
        labels=["x", "y", "z", "norm"], showfliers=True)
    axes[0, 0].set(title="Position error distribution", ylabel="m")
    axes[0, 1].boxplot(
        [clean(attitude[f"error_{axis}_deg"])
         for axis in ("roll", "pitch", "yaw")] +
        [clean(attitude["error_angle_deg"])],
        labels=["roll", "pitch", "yaw", "3D angle"], showfliers=True)
    axes[0, 1].set(title="Attitude error distribution", ylabel="deg")
    axes[1, 0].boxplot(
        [clean(rate_error[axis]) for axis in "xyz"] +
        [clean(rate_error["norm"])],
        labels=["x", "y", "z", "norm"], showfliers=True)
    axes[1, 0].set(title="Body-rate error distribution", ylabel="rad/s")
    axes[1, 1].boxplot(
        [normalized_differences(series(rows, "des_thrust_n"))] +
        [normalized_differences(series(rows, f"des_rate_{axis}"))
         for axis in "xyz"],
        labels=[
            "thrust", "rate x", "rate y", "rate z"
        ],
        showfliers=True)
    axes[1, 1].set(
        title="Normalized command change distribution",
        ylabel="sample delta / observed range")
    for plot_axes in axes.flat:
        plot_axes.grid(True, axis="y", alpha=0.3)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_all(
    rows: Sequence[Row], attitude: Dict[str, List[float]],
    rate_error: Dict[str, List[float]], output: Path, show: bool,
) -> List[Path]:
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError(
            "绘图需要 matplotlib，可使用 --no-plot 只计算指标"
        ) from error
    overview = output
    controls = output.with_name(f"{output.stem}.controls{output.suffix}")
    boxplots = output.with_name(f"{output.stem}.boxplots{output.suffix}")
    plot_overview(rows, attitude, rate_error, overview)
    plot_controls(rows, controls)
    plot_boxplots(rows, attitude, rate_error, boxplots)
    if show:
        for image_path in (overview, controls, boxplots):
            image = plt.imread(image_path)
            plt.figure(figsize=(14, 9))
            plt.imshow(image)
            plt.axis("off")
        plt.show()
    return [overview, controls, boxplots]


def print_key_results(summary: Dict) -> None:
    position = summary["position_error_m"]["norm"]
    attitude = summary["attitude_error_deg"]["three_dimensional_angle"]
    body_rate = summary["body_rate_error_radps"]["norm"]
    print(
        "关键指标: "
        f"位置RMSE={position['rmse']:.4f} m, "
        f"姿态角RMSE={attitude['rmse']:.3f} deg, "
        f"角速度RMSE={body_rate['rmse']:.4f} rad/s"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="全面评估飞行轨迹、姿态、角速度和控制器输出"
    )
    parser.add_argument(
        "csv", type=Path, help="flight_data_recorder_node生成的CSV"
    )
    parser.add_argument(
        "--state", default="3",
        help="只分析该FSM状态；默认3(CMD)，all表示全部状态，仍筛选有效位置数据")
    parser.add_argument("--output", type=Path, help="总览图路径，默认与CSV同名.png")
    parser.add_argument("--no-plot", action="store_true", help="只输出指标，不生成图片")
    parser.add_argument("--show", action="store_true", help="生成图片后打开交互窗口")
    args = parser.parse_args()

    try:
        state = None if args.state.lower() == "all" else int(args.state)
    except ValueError:
        parser.error("--state 必须是整数或 all")
    try:
        rows = load_rows(args.csv, state)
    except (OSError, ValueError) as error:
        parser.exit(1, f"分析失败: {error}\n")
    attitude = attitude_series(rows)
    rate_error = vector_error(rows, "gyro_", "des_rate_")
    summary = summarize(rows, args.csv, state, attitude, rate_error)
    summary_path = args.csv.with_suffix(".summary.json")
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print_key_results(summary)
    print(f"完整指标已保存: {summary_path}")
    if not args.no_plot:
        output = args.output or args.csv.with_suffix(".png")
        images = plot_all(rows, attitude, rate_error, output, args.show)
        for image_path in images:
            print(f"评估图已保存: {image_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
