"""Statistics, phase candidates, gated tracking analysis, and signal diagnostics."""
import warnings
import numpy as np
from scipy.signal import detrend, welch
from scipy.spatial.transform import Rotation
from .ulog_data import Signal, hold, interval_mask

PAIR_SPECS = (
    ('position', 'position_ref', 'position', '位置', 'm'),
    ('velocity', 'velocity_ref', 'velocity', '速度', 'm/s'),
    ('acceleration', 'acceleration_ref', 'acceleration', '世界系加速度', 'm/s²'),
    ('rate', 'rate_des', 'body_rate', '角速度', 'rad/s'),
    ('alpha', 'alpha_des', 'angular_acceleration', '角加速度（估计值）', 'rad/s²'),
    ('tau_sol', 'tau_des', 'allocation_torque', '分配力矩（模型值）', 'N m'),
    ('thrust_sol', 'thrust_des', 'allocation_thrust', '分配总推力（模型值）', 'N'),
    ('rpm_measured', 'rpm_des', 'motor_rpm', 'ESC 实测转速', 'RPM'),
    ('thrust_est_motor', 'motor_thrust', 'motor_thrust', 'ESC 反算单桨推力（模型值）', 'N'),
    ('thrust_est', 'thrust_des', 'estimated_thrust', 'ESC 反算总推力（模型值）', 'N'),
    ('tau_est', 'tau_des', 'estimated_torque', 'ESC 反算力矩（模型值）', 'N m'),
    ('tau_inverse', 'tau_des', 'inverse_dynamics_torque', '逆动力学净力矩（估计值）', 'N m'),
)


def stats(values):
    a = np.asarray(values, dtype=float).ravel()
    x = a[np.isfinite(a)]
    if not len(x):
        return {'samples': 0, 'available': False}
    return {'available': True, 'samples': len(x), 'valid_fraction': len(x) / max(1, len(a)),
            'mean': float(np.mean(x)), 'std': float(np.std(x)),
            'rmse': float(np.sqrt(np.mean(x*x))), 'mean_abs': float(np.mean(abs(x))),
            'min': float(np.min(x)), 'max': float(np.max(x)), 'max_abs': float(np.max(abs(x))),
            'p95': float(np.percentile(x, 95)), 'p95_abs': float(np.percentile(abs(x), 95)),
            'peak_to_peak': float(np.ptp(x))}


def columns(values):
    y = np.asarray(values)
    if y.ndim == 1:
        y = y[:, None]
    result = {str(i + 1): stats(y[:, i]) for i in range(y.shape[1])}
    if y.shape[1] > 1:
        result['norm'] = stats(np.linalg.norm(y, axis=1))
    return result


def contiguous(t, y, max_gap):
    valid = np.isfinite(y).all(axis=1)
    idx = np.flatnonzero(valid)
    if len(idx) < 2:
        return []
    cuts = np.flatnonzero((np.diff(idx) > 1) | (np.diff(t[idx]) > max_gap) | (np.diff(t[idx]) <= 0)) + 1
    return [chunk for chunk in np.split(idx, cuts) if len(chunk) >= 2]


def delay_estimates(t, actual, desired, max_age, max_lag=.5):
    """Positive lag: actual follows desired. Descriptive correlation, not transport delay."""
    results = []
    for axis in range(actual.shape[1]):
        pair = np.column_stack((actual[:, axis], desired[:, axis]))
        entries = []
        for idx in contiguous(t, pair, max_age):
            dt = float(np.median(np.diff(t[idx])))
            if len(idx) < 64 or dt <= 0 or t[idx[-1]] - t[idx[0]] < max(1., 4 * max_lag):
                continue
            grid = np.arange(t[idx[0]], t[idx[-1]], dt)
            y = detrend(np.interp(grid, t[idx], actual[idx, axis]))
            x = detrend(np.interp(grid, t[idx], desired[idx, axis]))
            if min(np.std(x), np.std(y)) < 1e-6:
                continue
            limit = min(int(max_lag / dt), len(grid) // 4)
            if limit < 1:
                continue
            coefficients = []
            for lag in range(-limit, limit + 1):
                xx = x[:len(x)-lag] if lag > 0 else x[-lag:] if lag < 0 else x
                yy = y[lag:] if lag > 0 else y[:len(y)+lag] if lag < 0 else y
                coefficients.append(float(np.corrcoef(xx, yy)[0, 1]) if min(np.std(xx), np.std(yy)) > 1e-9 else -1.)
            peak = int(np.argmax(coefficients))
            lag = peak - limit
            entries.append({'start_s': float(grid[0]), 'end_s': float(grid[-1]),
                            'lag_s': lag * dt, 'correlation': coefficients[peak],
                            'at_search_boundary': abs(lag) == limit,
                            'usable': coefficients[peak] >= .3 and abs(lag) != limit,
                            'samples': len(grid)})
        results.append({'axis': axis + 1, 'segments': entries,
                        'note': '正值表示实际滞后期望；相关性估计受闭环、周期性和接收时间抖动影响，不是通信延迟。'})
    return results


def spectra(signal, windows=None, max_gap=.1):
    y = signal.y.copy()
    if windows is not None:
        y[~interval_mask(signal.t, windows)] = np.nan
    results = []
    for axis in range(y.shape[1]):
        for idx in contiguous(signal.t, y[:, axis:axis+1], max_gap):
            if len(idx) < 64:
                continue
            dt = float(np.median(np.diff(signal.t[idx])))
            if dt <= 0:
                continue
            grid = np.arange(signal.t[idx[0]], signal.t[idx[-1]], dt)
            if len(grid) < 64:
                continue
            values = np.interp(grid, signal.t[idx], y[idx, axis])
            f, power = welch(values, fs=1/dt, nperseg=min(1024, len(values)), detrend='linear')
            positive = (f > 0) & np.isfinite(power)
            peak = np.flatnonzero(positive)[np.argmax(power[positive])] if positive.any() else 0
            results.append({'axis': axis + 1, 'start_s': float(grid[0]), 'end_s': float(grid[-1]),
                            'sample_hz': 1/dt, 'frequency_hz': f, 'psd': power,
                            'peak_hz': float(f[peak]) if power[peak] > 1e-20 else None,
                            'peak_psd': float(power[peak]), 'unit': signal.unit + '²/Hz'})
    return results


def phases(log, max_age):
    s = log.signals
    t = np.linspace(0, log.duration, max(2, int(log.duration * 10) + 1))
    state, _ = hold(s.get('state'), t, max_age)
    reference, _ = hold(s.get('velocity_ref'), t, max_age)
    labels = np.full(len(t), '未知/数据缺失', dtype=object)
    names = {0: '手动接管', 1: '手动控制', 2: '自动悬停/移点', 3: 'CMD（轨迹模式）', 4: '安全模式', 5: '错误状态'}
    for value, name in names.items():
        labels[state[:, 0] == value] = name
    if reference.shape[1] == 3:
        cmd = state[:, 0] == 3
        horizontal = np.linalg.norm(reference[:, :2], axis=1)
        labels[cmd & (horizontal > .05)] = '轨迹运动候选'
        labels[cmd & (horizontal <= .05) & (reference[:, 2] > .05)] = '上升/起飞候选'
        labels[cmd & (horizontal <= .05) & (reference[:, 2] < -.05)] = '下降候选'
    if 'landed' in s:
        landed, _ = hold(s['landed'], t, max(.5, max_age))
        labels[landed[:, 0] == 1] = '已落地（PX4）'
    cuts = np.r_[0, np.flatnonzero(labels[1:] != labels[:-1]) + 1, len(t)]
    segments = [{'start_s': float(t[a]), 'end_s': float(t[b] if b < len(t) else t[-1]), 'label': str(labels[a])}
                for a, b in zip(cuts[:-1], cuts[1:])]
    return {'segments': segments, 'method': 'FSM + 参考速度 + 可用的 PX4 landed，仅用于阶段显示；统计区间由 state == 3 和用户确认决定。'}, t, labels


def topic_quality(log):
    result = {}
    for d in log.datasets:
        t = log.times(d)
        dt = np.diff(t)
        positive = dt[dt > 0]
        entry = {'samples': len(t), 'duration_s': float(t.max() - t.min()),
                 'received_hz': float((len(t)-1) / (t.max()-t.min())) if t.max() > t.min() else None,
                 'receive_interval_s': stats(positive), 'nonpositive_intervals': int(np.sum(dt <= 0))}
        if len(positive):
            entry['gaps_over_3x_median'] = int(np.sum(dt > 3 * np.median(positive)))
        if 'recorder_sequence' in d.data:
            delta = np.diff(d.data['recorder_sequence'].astype(np.int64))
            entry['sequence_missing'] = int(np.maximum(delta - 1, 0).sum())
            entry['sequence_nonincreasing'] = int(np.sum(delta <= 0))
        if 'msg_timestamp' in d.data:
            delta = np.diff(d.data['msg_timestamp'].astype(np.int64))
            entry['source_timestamp_backwards'] = int(np.sum(delta < 0))
            entry['source_timestamp_repeated'] = int(np.sum(delta == 0))
        result[d.name + ':' + str(d.multi_id)] = entry
    return result


def error_pair(actual, desired, windows, max_age, quaternion=False):
    target, ages = hold(desired, actual.t, max_age)
    selected = interval_mask(actual.t, windows)
    measured = actual.y.copy()
    measured[~selected], target[~selected] = np.nan, np.nan
    if quaternion:
        an, dn = np.linalg.norm(measured, axis=1), np.linalg.norm(target, axis=1)
        valid = np.isfinite(measured).all(axis=1) & np.isfinite(target).all(axis=1) & (an > 1e-9) & (dn > 1e-9)
        euler_a, euler_d = np.full((len(measured), 3), np.nan), np.full((len(measured), 3), np.nan)
        angle = np.full(len(measured), np.nan)
        if valid.any():
            qa, qd = measured[valid] / an[valid, None], target[valid] / dn[valid, None]
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', UserWarning)
                euler_a[valid] = Rotation.from_quat(qa[:, [1, 2, 3, 0]]).as_euler('xyz', degrees=True)
                euler_d[valid] = Rotation.from_quat(qd[:, [1, 2, 3, 0]]).as_euler('xyz', degrees=True)
            angle[valid] = np.rad2deg(2 * np.arccos(np.clip(abs(np.sum(qa * qd, axis=1)), 0, 1)))
        measured, target = euler_a, euler_d
        error = np.full_like(measured, np.nan)
        error[valid] = (measured[valid] - target[valid] + 180) % 360 - 180
        metrics = columns(error[selected])
        metrics.pop('norm', None)  # Euler error norm is NOT an SO(3) distance.
        metrics['three_dimensional_angle'] = stats(angle[selected])
    else:
        error = measured - target
        angle = None
        metrics = columns(error[selected])
    return {'t': actual.t, 'actual': measured, 'desired': target, 'error': error,
            'angle': angle, 'metrics': metrics, 'selected_samples': int(selected.sum()),
            'reference_receive_age_s': stats(ages[selected])}


def analyze(log, windows, decision, max_age):
    s = log.signals
    phase_info, phase_t, phase_labels = phases(log, max_age)
    summary = {'source': str(log.path), 'duration_s': log.duration,
               'time_axis': 'recorder monotonic reception seconds since first recorded sample',
               'alignment': {'method': 'causal last value at actual sample time', 'max_age_s': max_age,
                             'note': '使用接收时间对齐；age 为接收间隔，不是端到端延迟。各项独立有效性检查，不依赖位置有效。'},
               'tracking_decision': decision, 'trajectory_windows_s': windows,
               'phases': phase_info, 'tracking': {}, 'coverage': {}, 'outputs': {},
               'smoothness': {}, 'flags': {}, 'timing': {}, 'noise': {}, 'spectra': {},
               'delays': {}, 'electrical': {}, 'topic_quality': topic_quality(log),
               'recorder_summary': log.info.get('recorder_summary', {}), 'notes': log.notes}
    details, spectral_data = {}, {}
    specs = list(PAIR_SPECS) + [('attitude', 'attitude_des', 'attitude', '姿态', 'deg')]
    for actual, desired, key, label, unit in specs:
        missing = [name for name in (actual, desired) if name not in s or not np.isfinite(s[name].y).any()]
        summary['coverage'][key] = {'label': label, 'missing_signals': missing,
                                    'status': 'missing' if missing else 'ready' if windows else 'not_requested'}
        if missing or not windows:
            continue
        pair = error_pair(s[actual], s[desired], windows, max_age, key == 'attitude')
        pair.update(label=label, unit=unit)
        details[key] = pair
        summary['tracking'][key] = {'label': label, 'unit': unit, 'error_definition': 'actual minus desired',
                                    'metrics': pair['metrics'], 'selected_samples': pair['selected_samples'],
                                    'reference_receive_age_s': pair['reference_receive_age_s']}
        if not any(v.get('samples', 0) for v in pair['metrics'].values()):
            summary['coverage'][key]['status'] = 'no_valid_overlap_in_confirmed_window'
        if key in ('position', 'velocity', 'body_rate', 'angular_acceleration', 'motor_rpm'):
            summary['delays'][key] = delay_estimates(pair['t'], pair['actual'], pair['desired'], max_age)
        if key in ('body_rate', 'angular_acceleration', 'motor_rpm'):
            spectral_data[key + '_error'] = spectra(Signal(pair['t'], pair['error'], unit), max_gap=max_age)
        # Retain per-CMD-window metrics as well as the combined confirmed metrics.
        summary['tracking'][key]['segments'] = []
        for start, end in windows:
            subset = error_pair(s[actual], s[desired], [(start, end)], max_age, key == 'attitude')
            summary['tracking'][key]['segments'].append({'start_s': start, 'end_s': end, 'metrics': subset['metrics']})

    output_keys = ('thrust_des', 'rate_des', 'motor_thrust', 'rpm_des', 'actuator', 'throttle_raw',
                   'cts', 'cms', 'cts_reconstructed', 'offline_cts', 'offline_cms', 'p_term', 'i_term', 'd_term', 'thr2acc')
    for key in output_keys:
        if key not in s:
            continue
        signal = s[key]
        y = signal.y.copy()
        if key in ('actuator', 'throttle_raw'):
            y[y < 0] = np.nan  # -1 denotes inactive in this project.
        summary['outputs'][key] = {'unit': signal.unit, 'axes': columns(y)}
        dy, dt = np.diff(y, axis=0), np.diff(signal.t)
        valid = (dt > 1e-9) & (dt <= max_age)
        dy[~valid] = np.nan
        slew = np.divide(dy, dt[:, None], out=np.full_like(dy, np.nan), where=valid[:, None])
        summary['smoothness'][key] = {'sample_delta': columns(dy), 'slew_per_s': columns(slew),
                                      'total_variation': [float(np.nansum(abs(dy[:, i]))) if np.isfinite(dy[:, i]).any() else None for i in range(y.shape[1])]}
    for key in ('thrust_clipped_low', 'thrust_clipped_high', 'i_clipped', 'saturation_positive',
                'saturation_negative', 'actuator_fallback', 'speed_curve_limited', 'deadline_missed',
                'control_updated', 'control_inputs_valid', 'control_result_finite', 'tvr_output_valid',
                'allocation_positive_inferred', 'allocation_negative_inferred'):
        if key in s:
            summary['flags'][key] = columns(s[key].y)
    if 'actuator' in s:
        y = s['actuator'].y
        indicator = np.where(np.isfinite(y) & (y >= 0), (y >= .999).astype(float), np.nan)
        summary['flags']['actuator_near_upper_limit_inferred'] = columns(indicator)
    for key in ('solve_ms', 'work_ms', 'cycle_ms', 'cycle_dt', 'tvr_ms', 'input_age', 'esc_age'):
        if key in s:
            summary['timing'][key] = {'unit': s[key].unit, 'axes': columns(s[key].y)}
    if 'tvr_status' in s:
        summary['timing']['tvr_status_counts'] = [{str(int(x)): int(np.sum(column == x))
            for x in np.unique(column[np.isfinite(column)])} for column in s['tvr_status'].y.T]
    summary['model_consistency'] = {key: {'unit': s[key].unit, 'axes': columns(s[key].y)}
                                    for key in ('pid_identity_residual', 'allocation_condition') if key in s}
    if 'state' in s:
        for key in ('position', 'attitude', 'rate'):
            if key in s:
                _, age = hold(s[key], s['state'].t, max_age)
                summary['timing'][key + '_receive_age'] = stats(age)
    for key in ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter'):
        if key in s:
            summary['topic_quality'][key] = {'counter_changes': int(np.sum(np.diff(s[key].y[:, 0]) != 0))}
    for key in ('rate_raw', 'rate_px4', 'rate_lpf', 'accel_raw', 'accel_px4'):
        if key not in s:
            continue
        summary['noise'][key] = {'unit': s[key].unit, 'scope': '全记录的运动+噪声波动，不能当作静止传感器本底噪声', 'axes': []}
        for x in s[key].y.T:
            x = x[np.isfinite(x)]
            clean = x[abs(x - x.mean()) <= 3*x.std(ddof=1)] if len(x) > 1 else x
            summary['noise'][key]['axes'].append({'raw': stats(x), 'after_single_3sigma': stats(clean),
                                                   'sample_variance': float(np.var(clean, ddof=1)) if len(clean) > 1 else None})
        spectral_data[key] = spectra(s[key], max_gap=max_age)
    for key, entries in spectral_data.items():
        summary['spectra'][key] = [{k: v for k, v in x.items() if k not in ('frequency_hz', 'psd')} for x in entries]
    for key in ('voltage', 'current', 'battery_remaining', 'motor_voltage', 'motor_current',
                'electrical_power', 'battery_power', 'mechanical_power'):
        if key not in s:
            continue
        sig = s[key]
        summary['electrical'][key] = {'unit': sig.unit, 'axes': columns(sig.y)}
        if 'power' in key:
            dt = np.diff(sig.t)
            mid = .5 * (sig.y[1:] + sig.y[:-1])
            valid = (dt > 0) & (dt <= max_age)
            mid[~valid] = np.nan
            summary['electrical'][key]['observed_energy_J'] = [float(np.nansum(mid[:, i] * dt)) if np.isfinite(mid[:, i]).any() else None for i in range(mid.shape[1])]
            summary['electrical'][key]['integrated_duration_s'] = [float(np.sum(dt[np.isfinite(mid[:, i])])) for i in range(mid.shape[1])]
    if windows and 'voltage' in s:
        summary['electrical']['voltage_error_correlations'] = {}
        for key in ('body_rate', 'motor_rpm', 'allocation_torque'):
            if key not in details:
                continue
            pair = details[key]
            voltage, _ = hold(s['voltage'], pair['t'], max_age)
            err = np.linalg.norm(pair['error'], axis=1)
            valid = np.isfinite(err) & np.isfinite(voltage[:, 0]) & (voltage[:, 0] > 0)
            value = None
            if valid.sum() >= 20 and min(np.std(err[valid]), np.std(voltage[valid, 0])) > 1e-9:
                value = float(np.corrcoef(err[valid], voltage[valid, 0])[0, 1])
            summary['electrical']['voltage_error_correlations'][key] = {'pearson_r': value, 'samples': int(valid.sum()), 'note': '仅确认轨迹段；相关不等于因果。'}
    return summary, details, spectral_data, (phase_t, phase_labels)
