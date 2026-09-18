"""Read topics, preserve source data, and expose explicitly framed signals."""
from dataclasses import dataclass
import json
from pathlib import Path
import re
import numpy as np
from pyulog import ULog

FLIP = np.array([1., -1., -1.])

@dataclass
class Signal:
    t: np.ndarray
    y: np.ndarray
    unit: str = ''
    source: str = ''
    frame: str = ''

    def __post_init__(self):
        self.y = np.asarray(self.y, dtype=float)
        if self.y.ndim == 1:
            self.y = self.y[:, None]


def hold(signal, times, max_age=.1):
    """Causal latest sample, with freshness limit; no interpolation across gaps."""
    if signal is None or len(signal.t) == 0:
        return np.full((len(times), 1), np.nan), np.full(len(times), np.nan)
    indices = np.searchsorted(signal.t, times, side='right') - 1
    clipped = np.clip(indices, 0, len(signal.t) - 1)
    age = times - signal.t[clipped]
    valid = (indices >= 0) & (age >= -1e-9) & (age <= max_age)
    result = signal.y[clipped].copy()
    result[~valid] = np.nan
    return result, np.where(indices >= 0, age, np.nan)


def interval_mask(times, windows):
    """Select half-open intervals, excluding the sample that exits CMD."""
    result = np.zeros(len(times), dtype=bool)
    for start, end in windows:
        result |= (times >= start) & (times < end)
    return result


def cmd_windows(state, duration, max_age=.1):
    """Observed CMD intervals [start, end), bounded by state freshness and EOF."""
    if state is None or not len(state.t):
        return []
    # Match hold(): the last received state wins at duplicate timestamps.
    keep = np.r_[state.t[1:] != state.t[:-1], True]
    times, values = state.t[keep], state.y[keep, 0]
    idx = np.flatnonzero(values == 3)
    if not len(idx):
        return []
    cuts = np.flatnonzero((np.diff(idx) > 1) | (np.diff(times[idx]) > max_age)) + 1
    windows = []
    for run in np.split(idx, cuts):
        first, last = run[0], run[-1]
        next_time = times[last + 1] if last + 1 < len(times) else duration
        start = float(times[first])
        end = float(min(next_time, times[last] + max_age, duration))
        if end > start:
            windows.append((start, end))
    return windows


class FlightLog:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.ulog = ULog(str(self.path))
        self.datasets = [d for d in self.ulog.data_list
                         if d.name.startswith('ros/') and len(d.data.get('timestamp', []))]
        if not self.datasets:
            raise ValueError('没有 ros/ 话题数据集；本入口读取项目记录器 ULog，不读取旧 CSV 或原生 PX4 SD 卡日志。')
        self.notes, self.signals = [], {}
        self.info = dict(self.ulog.msg_info_dict)
        for key, groups in self.ulog.msg_info_multiple_dict.items():
            values = [''.join(map(str, chunks)) for chunks in groups]
            if len(values) == 1:
                self.info[key] = values[0]
        for key, value in list(self.info.items()):
            if isinstance(value, str):
                try:
                    self.info[key] = json.loads(value)
                except (ValueError, TypeError):
                    pass
        if self.ulog.file_corruption:
            self.notes.append('pyulog 检测到文件损坏；只能分析成功解析的样本，不能把缺失数据解释为正常飞行。')
        if '.incomplete.' in self.path.name or '.recovered.' in self.path.name or 'recorder_recovered' in self.info:
            self.notes.append('输入是未完整结束或恢复的日志；报告仅覆盖现存数据。')
        if 'recorder_summary' not in self.info:
            self.notes.append('日志缺少记录器结束汇总，无法确认收到的消息是否全部写入。')
        metadata = self.info.get('recorder_metadata', {})
        excluded = metadata.get('excluded_topics', []) if isinstance(metadata, dict) else []
        if '/fmu/out/vehicle_attitude' in excluded:
            self.notes.append('本次记录配置排除了 vehicle_attitude；若无 ratectrl attitude_wxyz，实际姿态分析和姿态回放不可用。')
        for value in self.info.values():
            if isinstance(value, dict) and value.get('omitted_fields'):
                self.notes.append('记录时省略字段：' + value.get('topic', '') + ' ' +
                                  ', '.join('.'.join(f['path']) for f in value['omitted_fields']))
        self.origin_ns = min(int(self.stamps(d).min()) for d in self.datasets)
        self.duration = max((int(self.stamps(d).max()) - self.origin_ns) / 1e9 for d in self.datasets)
        self._extract()

    @staticmethod
    def stamps(dataset):
        if 'recorder_monotonic_ns' in dataset.data:
            return dataset.data['recorder_monotonic_ns'].astype(np.int64)
        return dataset.data['timestamp'].astype(np.int64) * 1000

    def times(self, dataset):
        return (self.stamps(dataset) - self.origin_ns).astype(float) / 1e9

    def topic(self, name):
        found = [d for d in self.datasets if re.sub(r'_v\d+$', '', d.name) == 'ros' + name]
        if len(found) > 1:
            self.notes.append(f'{name} 有多个版本/实例；选样本最多者，原始数据全部保留。')
        return max(found, key=lambda d: len(d.data['timestamp'])) if found else None

    @staticmethod
    def fields(dataset, names):
        if dataset is None:
            return None
        keys = ['msg_' + name for name in names]
        if not all(k in dataset.data for k in keys):
            return None
        return np.column_stack([dataset.data[k] for k in keys]).astype(float)

    def add(self, key, dataset, names, unit='', frame='', scale=1., valid=None):
        values = self.fields(dataset, names)
        if values is None:
            return
        values *= np.asarray(scale)
        if valid is not None:
            flags = self.fields(dataset, valid)
            if flags is not None:
                if flags.shape[1] == values.shape[1]:
                    values[flags != 1] = np.nan
                else:
                    values[~np.all(flags == 1, axis=1)] = np.nan
        times = self.times(dataset)
        order = np.argsort(times, kind='stable')
        self.signals[key] = Signal(times[order], values[order], unit,
                                   dataset.name + ':' + ','.join(names), frame)

    def array(self, key, dataset, field, size, **kwargs):
        self.add(key, dataset, [f'{field}[{i}]' for i in range(size)], **kwargs)

    def xyz(self, key, dataset, prefix, **kwargs):
        self.add(key, dataset, [prefix + a for a in 'xyz'], **kwargs)

    def _extract(self):
        ctrl, rate, motor = [self.topic('/debugPx4/' + x) for x in ('ctrl', 'ratectrl', 'motor_feedback')]
        pose = self.topic('/fmu/out/vehicle_local_position')
        att = self.topic('/fmu/out/vehicle_attitude')
        imu = self.topic('/fmu/out/sensor_combined')
        angular = self.topic('/fmu/out/vehicle_angular_velocity')
        accel = self.topic('/fmu/out/vehicle_acceleration')
        sp = self.topic('/rates_thrust_setpoint')
        self.add('state', ctrl, ['state'])
        self.add('position', pose, list('xyz'), 'm', 'controller world (x,-y,-z)', FLIP, ['xy_valid', 'z_valid'])
        self.add('velocity', pose, ['v' + a for a in 'xyz'], 'm/s', 'controller world', FLIP, ['v_xy_valid', 'v_z_valid'])
        self.add('acceleration', pose, ['a' + a for a in 'xyz'], 'm/s²', 'controller world; PX4 velocity derivative', FLIP, ['v_xy_valid', 'v_z_valid'])
        self.array('attitude', att, 'q', 4, frame='controller body-to-world wxyz', scale=[1., 1., -1., -1.])
        self.array('rate_raw', imu, 'gyro_rad', 3, unit='rad/s', frame='FLU', scale=FLIP)
        self.array('accel_raw', imu, 'accelerometer_m_s2', 3, unit='m/s²', frame='FLU specific force', scale=FLIP)
        self.array('rate_px4', angular, 'xyz', 3, unit='rad/s', frame='FLU', scale=FLIP)
        self.array('alpha_px4', angular, 'xyz_derivative', 3, unit='rad/s²', frame='FLU', scale=FLIP)
        self.array('accel_px4', accel, 'xyz', 3, unit='m/s²', frame='FLU specific force', scale=FLIP)
        self.array('rate_des', sp, 'bodyrates', 3, unit='rad/s', frame='FLU')
        self.add('thrust_des', sp, ['thrust'], 'N')
        for key, prefix, unit in (('position_ref', 'ref_p_', 'm'), ('velocity_ref', 'ref_v_', 'm/s'),
                                  ('debug_acceleration_ref', 'ref_a_', 'm/s²'), ('debug_acceleration', 'fb_a_', 'm/s²'),
                                  ('rate_ref', 'ref_rate_', 'rad/s')):
            self.xyz(key, ctrl, prefix, unit=unit, frame='FLU' if 'rate' in key else 'controller world', scale=FLIP)
        self.add('attitude_des', ctrl, ['des_q_' + a for a in 'wxyz'], frame='controller body-to-world wxyz', scale=[1., 1., -1., -1.])
        if 'rate_des' not in self.signals:
            self.xyz('rate_des', ctrl, 'des_rate_', unit='rad/s', frame='FLU', scale=FLIP)
        if 'thrust_des' not in self.signals:
            self.add('thrust_des', ctrl, ['des_thrust'], 'N')
        self.add('voltage', ctrl, ['voltage'], 'V')
        self.add('thr2acc', ctrl, ['thr2acc'])
        for key, prefix, unit in (('alpha_des', 'des_rate_dot_', 'rad/s²'), ('alpha', 'cur_rate_dot_', 'rad/s²'),
                                  ('tau_des', 'des_tau_', 'N m'), ('tau_sol', 'cur_tau_', 'N m')):
            self.xyz(key, rate, prefix, unit=unit, frame='FLU', scale=FLIP)
        self.array('rpm_des', rate, 'des_motor_rpm', 4, unit='RPM')
        self.add('motor_thrust', rate, [f'des_u_{i}' for i in range(1, 5)], 'N')
        self.add('solve_ms', rate, ['solve_time_ms'], 'ms')
        self.array('actuator', self.topic('/fmu/in/actuator_motors'), 'control', 4, unit='normalized command')
        # Schema 2 arrays are FLU; legacy scalar vectors above are FRD.
        vectors = {
            'rate': ('rate_cur', 3, 'rad/s'), 'rate_lpf': ('rate_cur_lpf', 3, 'rad/s'),
            'rate_des': ('rate_des', 3, 'rad/s'), 'alpha': ('rate_dot_cur', 3, 'rad/s²'),
            'alpha_des': ('ome_dot_des', 3, 'rad/s²'), 'alpha_ff': ('rate_dot_ref', 3, 'rad/s²'),
            'tau_des': ('tau_des', 3, 'N m'), 'tau_sol': ('tau_sol', 3, 'N m'),
            'tau_inverse': ('tau_from_rate_dot', 3, 'N m'), 'tau_residual': ('tau_residual', 3, 'N m'),
            'attitude': ('attitude_wxyz', 4, 'wxyz'), 'airflow': ('va_b', 3, 'm/s'),
            'motor_thrust': ('motor_thrust_sol', 4, 'N'), 'motor_thrust_raw': ('motor_thrust_raw', 4, 'N'),
            'motor_rad': ('motor_rad_sol', 4, 'rad/s'), 'cts': ('cts', 4, 'N/(rad/s)²'),
            'cms': ('cms', 4, 'N m/(rad/s)²'), 'reaction_sol': ('motor_reaction_torque_sol', 4, 'N m'),
            'throttle_raw': ('thro_setpoint_i', 4, 'normalized'), 'actuator': ('actuator_control', 4, 'normalized'),
            'p_term': ('p_term', 3, 'rad/s²'), 'i_term': ('i_term', 3, 'rad/s²'),
            'd_term': ('d_term', 3, 'rad/s²'), 'i_after': ('ome_int_after', 3, 'rad/s²'),
            'input_age': ('input_age_s', 5, 's'), 'inertia': ('inertia_diagonal', 3, 'kg m²'),
            'effectiveness': ('effectiveness', 16, 'mixed'), 'tvr_ms': ('tvr_time_ms', 3, 'ms'),
            'tvr_status': ('tvr_status', 3, 'status'),
        }
        for key, (field, size, unit) in vectors.items():
            self.array(key, rate, field, size, unit=unit, frame='FLU' if size == 3 else '')
        for key, field, unit in (('state', 'fsm_state', ''), ('thrust_des', 'thrust_des', 'N'),
                                 ('thrust_sol', 'thrust_sol', 'N'), ('work_ms', 'work_time_ms', 'ms'),
                                 ('cycle_ms', 'previous_cycle_time_ms', 'ms'), ('cycle_dt', 'cycle_interval_s', 's'),
                                 ('nominal_hz', 'nominal_rate_hz', 'Hz'), ('thrust_min', 'thrust_min', 'N'),
                                 ('thrust_max', 'thrust_max', 'N')):
            if key == 'state' and key in self.signals:
                continue  # Prefer the outer FSM; ratectrl is only a fallback.
            self.add(key, rate, [field], unit)
        for field, size in (('thrust_clipped_low', 4), ('thrust_clipped_high', 4), ('i_clipped', 3),
                            ('saturation_positive', 3), ('saturation_negative', 3), ('actuator_fallback', 4),
                            ('speed_curve_limited', 4), ('tvr_output_valid', 3)):
            self.array(field, rate, field, size)
        for field in ('deadline_missed', 'control_updated', 'control_inputs_valid', 'control_result_finite'):
            self.add(field, rate, [field])
        # Don't score inactive controller snapshots or TVR warmup as real outputs.
        if rate is not None:
            for key, flag_names in (
                ('rate', ['input_valid[0]']), ('attitude', ['input_valid[2]']),
                ('alpha', [f'tvr_output_valid[{i}]' for i in range(3)]),
            ):
                flags = self.fields(rate, flag_names)
                sig = self.signals.get(key)
                if flags is not None and sig is not None and sig.source.startswith(rate.name + ':'):
                    order = np.argsort(self.times(rate), kind='stable')
                    flags = flags[order]
                    if flags.shape[1] == sig.y.shape[1]:
                        sig.y[flags != 1] = np.nan
                    else:
                        sig.y[~np.all(flags == 1, axis=1)] = np.nan
            active = self.fields(rate, ['control_updated', 'control_result_finite'])
            if active is not None:
                valid = np.all(active[np.argsort(self.times(rate), kind='stable')] == 1, axis=1)
                for key in ('alpha_des', 'rate_des', 'tau_des', 'tau_sol', 'tau_residual', 'thrust_des',
                            'thrust_sol', 'motor_thrust', 'motor_thrust_raw', 'motor_rad', 'rpm_des',
                            'p_term', 'i_term', 'd_term', 'alpha_ff', 'reaction_sol', 'throttle_raw'):
                    sig = self.signals.get(key)
                    if sig is not None and sig.source.startswith(rate.name + ':'):
                        sig.y[~valid] = np.nan
        if 'rate' not in self.signals:
            source = self.signals.get('rate_px4', self.signals.get('rate_raw'))
            if source is not None:
                self.signals['rate'] = source
                self.notes.append('rate_cur 未记录：对比使用 PX4 滤波角速度（若有），否则 sensor_combined；不声称它就是本次控制器的输入。')
        if 'alpha' not in self.signals and 'alpha_px4' in self.signals:
            self.signals['alpha'] = self.signals['alpha_px4']
        # Cached ESC feedback must be valid BEFORE resampling and statistics.
        for key, field, unit, valid in (
            ('rpm_measured', 'motor_rpm', 'RPM', 'rpm_valid'),
            ('motor_voltage', 'motor_voltage', 'V', 'electrical_valid'),
            ('motor_current', 'motor_current', 'A', 'electrical_valid'),
            ('electrical_power', 'electrical_power', 'W', 'electrical_valid'),
            ('thrust_est_motor', 'motor_thrust_est', 'N (model)', 'estimate_valid'),
            ('reaction_est', 'motor_reaction_torque_est', 'N m (model)', 'estimate_valid'),
            ('mechanical_power', 'mechanical_power_est', 'W (model)', 'estimate_valid')):
            self.array(key, motor, field, 4, unit=unit, valid=[f'{valid}[{i}]' for i in range(4)])
        self.add('thrust_est', motor, ['thrust_est'], 'N (model)', valid=['total_estimate_valid'])
        self.array('tau_est', motor, 'tau_est', 3, unit='N m (model)', frame='FLU', valid=['total_estimate_valid'])
        self.array('feedback_airflow', motor, 'va_b', 3, unit='m/s', frame='FLU', valid=['airflow_valid'])
        self.add('esc_age', motor, ['esc_receive_age_s'], 's')
        self.add('esc_sequence', motor, ['esc_sequence'])
        for field in ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter'):
            self.add(field, pose, [field])
        battery = self.topic('/fmu/out/battery_status')
        for key, field, unit in (('voltage', 'voltage_v', 'V'), ('current', 'current_a', 'A'),
                                 ('battery_remaining', 'remaining', 'fraction')):
            self.add(key, battery, [field], unit)
        self.add('landed', self.topic('/fmu/out/vehicle_land_detected'), ['landed'])
        self.add('arming_state', self.topic('/fmu/out/vehicle_status'), ['arming_state'])
        if 'voltage' in self.signals:
            self.signals['voltage'].y[self.signals['voltage'].y <= 0] = np.nan
        if 'current' in self.signals:
            self.signals['current'].y[self.signals['current'].y < 0] = np.nan

    def export(self, output):
        raw = output / 'data' / 'raw'
        raw.mkdir(parents=True)
        inventory = []
        for number, d in enumerate(self.datasets):
            filename = f'{number:02d}_' + re.sub(r'[^a-zA-Z0-9_-]', '_', d.name) + f'_{d.multi_id}.npz'
            np.savez_compressed(raw / filename, **d.data)
            inventory.append({'topic': d.name, 'instance': d.multi_id, 'file': 'data/raw/' + filename,
                              'samples': len(d.data['timestamp']), 'fields': {k: str(v.dtype) for k, v in d.data.items()}})
        (output / 'data' / 'ulog_metadata.json').write_text(
            json.dumps({'metadata': self.info, 'datasets': inventory}, ensure_ascii=False,
                       indent=2, default=str), encoding='utf-8')
        return inventory
