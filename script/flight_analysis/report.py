"""Figures, human-readable conclusions, PDF, data exports and full-flight GIF."""
import csv
import json
from pathlib import Path
import tempfile
import unicodedata
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image
from .ulog_data import hold, interval_mask
from .models import rotation_matrices

_FONT_DIRECTORY = None


def configure_font(font=None):
    global _FONT_DIRECTORY
    candidates = [font] if font else [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/arphic/uming.ttc']
    available = next((Path(p) for p in candidates if p and Path(p).is_file()), None)
    if available is None:
        raise ValueError('中文 PDF 需要中文字体；请用 --font 指定 TTF/TTC 字体文件。')
    # Matplotlib 3.5 cannot subset a TTC collection into a type-42 PDF font.
    # Extract its first face into our temporary directory, not the system fonts.
    from fontTools.ttLib import TTFont
    from matplotlib.ft2font import FT2Font
    if not all(ord(c) in FT2Font(str(available)).get_charmap() for c in '飞行数据ABC012'):
        raise ValueError('所选字体需要同时包含中文、英文字母和数字；请用 --font 指定完整字体。')
    _FONT_DIRECTORY = tempfile.TemporaryDirectory(prefix='flight_analysis_font_')
    extracted = Path(_FONT_DIRECTORY.name) / 'report_font.ttf'
    face = TTFont(str(available), fontNumber=0)
    if 'BDF ' in face:
        del face['BDF ']
    for table in face['cmap'].tables:
        if table.isUnicode() and 0x2212 not in table.cmap and ord('-') in table.cmap:
            table.cmap[0x2212] = table.cmap[ord('-')]
    face.save(str(extracted))
    face.close()
    font_manager.fontManager.addfont(str(extracted))
    font_manager.fontManager.ttflist.insert(0, font_manager.fontManager.ttflist.pop())
    name = font_manager.FontProperties(fname=str(extracted)).get_name()
    plt.rcParams.update({'font.family': [name, 'DejaVu Sans'], 'axes.unicode_minus': False,
                         'pdf.fonttype': 42, 'font.size': 9, 'axes.grid': True,
                         'grid.alpha': .25, 'savefig.dpi': 135})
    return str(available)


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean_json(v) for v in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean_json(value), ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def sample_indices(n, limit=18000):
    return np.unique(np.linspace(0, n-1, min(n, limit), dtype=int)) if n else np.array([], dtype=int)


class Report:
    def __init__(self, output, log, max_age):
        self.output, self.log, self.max_age = output, log, max_age
        self.images = []
        (output / 'images').mkdir(parents=True)

    def save(self, figure, name):
        path = self.output / 'images' / (name + '.png')
        figure.savefig(path)
        plt.close(figure)
        if path not in self.images:
            self.images.append(path)
        return path

    def lines(self, ax, keys, title, labels=None):
        count = 0
        for index, key in enumerate(keys):
            sig = self.log.signals.get(key)
            if sig is None or not np.isfinite(sig.y).any():
                continue
            idx = sample_indices(len(sig.t))
            data = sig.y[idx].copy()
            # Plot gaps honestly, even when plot decimation skips many samples.
            for start in np.flatnonzero(np.diff(sig.t) > self.max_age) + 1:
                j = np.searchsorted(idx, start)
                if j < len(data):
                    data[j] = np.nan
            prefix = labels[index] if labels else key
            for axis in range(data.shape[1]):
                name = prefix + (f' {axis+1}' if data.shape[1] > 1 else '')
                ax.plot(sig.t[idx], data[:, axis], '--' if index else '-', linewidth=.7, label=name)
            count += 1
        ax.set(title=title, xlabel='记录接收时间 / s')
        if count:
            ax.legend(fontsize=6, ncol=2)
        else:
            ax.text(.5, .5, '缺少有效记录字段', ha='center', va='center', transform=ax.transAxes)

    def preview(self, cmd_windows):
        fig, axes = plt.subplots(3, 1, figsize=(13, 8), constrained_layout=True, sharex=True)
        self.lines(axes[0], ['position', 'position_ref'], '实际位置与参考位置 / m', ['实际', '参考'])
        self.lines(axes[1], ['velocity_ref'], '参考速度 / m/s（辅助观察内部阶段，不筛选统计区间）')
        self.lines(axes[2], ['state'], 'FSM：0 手动接管，1 手动，2 悬停/移点，3 CMD，4 安全，5 错误')
        for index, (start, end) in enumerate(cmd_windows, 1):
            for ax in axes:
                ax.axvspan(start, end, color='orange', alpha=.12)
            axes[2].text((start + end) / 2, .96, f'CMD {index}', ha='center', va='top',
                         transform=axes[2].get_xaxis_transform(), fontsize=8)
        fig.suptitle('全程预览：橙色为 state == 3 自动识别的 CMD 区间；逐段确认是否统计，无需输入时间')
        return self.save(fig, '00_flight_preview')

    def overview(self):
        fig = plt.figure(figsize=(13, 12), constrained_layout=True)
        ax = fig.add_subplot(321, projection='3d')
        for key, style, label in [('position', '-', '实际位置'), ('position_ref', '--', '参考位置')]:
            sig = self.log.signals.get(key)
            if sig is not None:
                idx = sample_indices(len(sig.t), 10000)
                ax.plot(*sig.y[idx].T, style, linewidth=.8, label=label)
        ax.set(xlabel='x / m', ylabel='y / m', zlabel='z / m', title='全记录轨迹（控制器世界系）')
        if ax.lines:
            ax.legend()
        for pos, keys, title in [(322, ['euler', 'euler_des'], '全程欧拉角 / deg（翻滚时注意奇异性）'),
                                  (323, ['velocity', 'velocity_ref'], '全程速度 / m/s'),
                                  (324, ['rate', 'rate_des'], '全程角速度与指令 / rad/s'),
                                  (325, ['alpha', 'alpha_des'], '全程角加速度 / rad/s²'),
                                  (326, ['thrust_des', 'thrust_sol', 'thrust_est'], '全程总推力 / N（含模型值）')]:
            self.lines(fig.add_subplot(pos), keys, title)
        return self.save(fig, '01_flight_overview')

    def tracking(self, details, windows):
        for key, pair in details.items():
            dims = pair['error'].shape[1]
            extra = int(pair['angle'] is not None)
            fig, axes = plt.subplots(dims + extra, 2, figsize=(13, 2.3*(dims+extra)), squeeze=False, constrained_layout=True)
            for axis in range(dims):
                for start, end in windows:
                    selected = np.flatnonzero(interval_mask(pair['t'], [(start, end)]))
                    idx = selected[sample_indices(len(selected))]
                    axes[axis, 0].plot(pair['t'][idx], pair['actual'][idx, axis], '-', linewidth=.7, color='C0')
                    axes[axis, 0].plot(pair['t'][idx], pair['desired'][idx, axis], '--', linewidth=.7, color='C1')
                    axes[axis, 1].plot(pair['t'][idx], pair['error'][idx, axis], linewidth=.7, color='C2')
                axes[axis, 0].set(title=f'{pair["label"]} 通道 {axis+1}：实际/估计（蓝）与期望（橙）', ylabel=pair['unit'])
                axes[axis, 1].set(title='实际/估计 - 期望', ylabel=pair['unit'])
            if extra:
                for start, end in windows:
                    mask = interval_mask(pair['t'], [(start, end)])
                    axes[-1, 0].plot(pair['t'][mask], pair['angle'][mask], linewidth=.7)
                axes[-1, 0].set(title='整体姿态夹角（四元数，0–180°）', ylabel='deg')
                axes[-1, 1].axis('off')
                axes[-1, 1].text(.05, .5, '翻滚时优先看四元数整体夹角。\n欧拉角在奇异姿态附近会跳变，不能将欧拉角误差范数当作姿态距离。', wrap=True)
            for ax in axes.flat:
                ax.set_xlabel('记录接收时间 / s；仅显示确认统计的 CMD 区间')
            self.save(fig, 'tracking_' + key)

    def diagnostic_figures(self, details, spectra):
        groups = {
            '02_controls': [(['thrust_des'], '期望总推力 / N'), (['rate_des'], '期望角速度 / rad/s'),
                            (['motor_thrust_raw', 'motor_thrust'], '单电机分配推力 / N'), (['rpm_des'], '期望转速 / RPM'),
                            (['thrust_delta'], '相邻采样推力变化 / N'), (['rate_delta'], '相邻采样角速度指令变化 / rad/s'),
                            (['actuator', 'throttle_raw'], '最终电机指令与反解油门（负值表示未激活）'),
                            (['solve_ms'], '角速度控制计算耗时 / ms')],
            '03_rate_controller': [(['alpha', 'alpha_des', 'alpha_ff'], '角加速度 / rad/s²'),
                                   (['p_term', 'i_term', 'd_term'], 'PID 分量 / rad/s²'),
                                   (['tau_des', 'tau_sol', 'tau_est', 'tau_inverse'], '力矩：期望、分配、ESC 模型、逆动力学 / N m'),
                                   (['tau_residual'], '分配残差：期望 - 分配 / N m'),
                                   (['cts', 'cts_reconstructed', 'offline_cts'], 'ct：推力系数 / N/(rad/s)²'),
                                   (['cms', 'offline_cms'], 'cm/cq：反扭矩系数 / N m/(rad/s)²'),
                                   (['i_after', 'i_clipped'], '更新后积分与积分限幅标志'),
                                   (['thrust_residual', 'thr2acc'], '推力分配残差 / N 与 thr2acc'),
                                   (['pid_identity_residual'], 'P+I+D+前馈 与角加速度指令的一致性 / rad/s²'),
                                   (['allocation_condition'], '未归一化分配矩阵条件数（混合单位，仅观察趋势）')],
            '04_motor_feedback': [(['rpm_measured', 'rpm_des'], 'ESC 实测与期望转速 / RPM'),
                                  (['thrust_est_motor', 'motor_thrust'], 'ESC 模型估计与分配单桨推力 / N'),
                                  (['reaction_est', 'reaction_sol'], '反扭矩估计与分配模型 / N m'),
                                  (['motor_voltage', 'voltage'], 'ESC / 电池电压 / V'),
                                  (['motor_current', 'current'], 'ESC / 电池电流 / A'),
                                  (['electrical_power', 'battery_power'], '电功率 / W'),
                                  (['mechanical_power'], '桨气动模型机械功率 / W（非实测轴功率）'),
                                  (['esc_age', 'battery_remaining'], 'ESC 接收年龄 / s 与电池剩余比例')],
            '05_constraints_timing': [(['thrust_clipped_low', 'thrust_clipped_high'], '单桨推力限幅标志'),
                                      (['saturation_positive', 'saturation_negative'], '正/负方向分配饱和标志'),
                                      (['actuator_fallback', 'speed_curve_limited'], '回退与转速曲线限幅'),
                                      (['input_age'], '控制器输入接收年龄 / s'),
                                      (['solve_ms', 'work_ms', 'cycle_ms'], '计算、工作、上一完整周期耗时 / ms'),
                                      (['tvr_ms'], 'TVR 各轴求解耗时 / ms'),
                                      (['tvr_status', 'tvr_output_valid'], 'TVR 状态（0预热、1成功、2–4失败）与有效标志'),
                                      (['deadline_missed', 'control_inputs_valid', 'control_result_finite'], '截止期与控制有效性标志'),
                                      (['position_age', 'attitude_age', 'imu_age', 'debug_age'], '各输入在 FSM 采样时的接收年龄 / s'),
                                      (['allocation_positive_inferred', 'allocation_negative_inferred'], '从分配残差推断方向（阈值 +/-1e-6 N m）')],
            '06_sensor_signals': [(['rate_raw', 'rate_px4', 'rate_lpf'], '陀螺仪与滤波角速度 / rad/s'),
                                  (['accel_raw', 'accel_px4'], '机体系比力 / m/s²（含重力响应，不是世界系运动加速度）')],
        }
        for name, panels in groups.items():
            fig, axes = plt.subplots((len(panels)+1)//2, 2, figsize=(14, 2.8*((len(panels)+1)//2)), squeeze=False, constrained_layout=True)
            for ax, (keys, title) in zip(axes.flat, panels):
                self.lines(ax, keys, title)
            fig.suptitle('全程诊断；模型估计不能视为实测力/力矩')
            self.save(fig, name)
        self.boxplots(details)
        for key, entries in spectra.items():
            if not entries:
                continue
            fig, ax = plt.subplots(figsize=(12, 4), constrained_layout=True)
            for item in entries:
                f, power = item['frequency_hz'], item['psd']
                valid = (f > 0) & (power > 0)
                ax.semilogy(f[valid], power[valid], linewidth=.8,
                            label=f'通道{item["axis"]} {item["start_s"]:.1f}–{item["end_s"]:.1f}s')
            ax.set(title=key + '：Welch 功率谱（运动成分 + 噪声；不跨记录缺口）', xlabel='频率 / Hz', ylabel=entries[0]['unit'])
            if len(entries) <= 12:
                ax.legend(fontsize=7, ncol=3)
            self.save(fig, 'spectrum_' + key)
        if 'voltage' in self.log.signals and 'body_rate' in details:
            pair = details['body_rate']
            voltage, _ = hold(self.log.signals['voltage'], pair['t'], self.max_age)
            error = np.linalg.norm(pair['error'], axis=1)
            valid = np.isfinite(error) & np.isfinite(voltage[:, 0]) & (voltage[:, 0] > 0)
            if valid.any():
                fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
                ax.scatter(voltage[valid, 0], error[valid], s=3, alpha=.25)
                ax.set(xlabel='电压 / V', ylabel='角速度误差范数 / rad/s', title='确认轨迹段：电压与角速度误差（相关不代表因果）')
                self.save(fig, 'voltage_tracking_relation')

    def boxplots(self, details):
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        for ax, key in zip(axes.flat, ('position', 'attitude', 'body_rate')):
            if key not in details:
                ax.text(.5, .5, '未生成：未确认轨迹段或缺少数据', transform=ax.transAxes, ha='center')
                continue
            pair = details[key]
            data = [v[np.isfinite(v)] for v in pair['error'].T]
            fourth = pair['angle'] if pair['angle'] is not None else np.linalg.norm(pair['error'], axis=1)
            data.append(fourth[np.isfinite(fourth)])
            ax.boxplot([v if len(v) else [np.nan] for v in data], labels=['1', '2', '3', '3D angle' if key == 'attitude' else 'norm'])
            ax.set(title=pair['label'] + '误差分布（确认轨迹段）', ylabel=pair['unit'])
        data, labels = [], []
        for key in ('thrust_des', 'rate_des'):
            sig = self.log.signals.get(key)
            if sig is None:
                continue
            for axis, values in enumerate(sig.y.T):
                valid = np.isfinite(values)
                diff = np.diff(values)
                dt = np.diff(sig.t)
                diff[(dt <= 0) | (dt > self.max_age)] = np.nan
                width = np.ptp(values[valid]) if valid.any() else np.nan
                d = diff[np.isfinite(diff)] / width if width > 1e-12 else np.zeros(np.isfinite(diff).sum())
                data.append(d if len(d) else [np.nan])
                labels.append(f'{key} {axis+1}')
        if data:
            axes[1, 1].boxplot(data, labels=labels)
        axes[1, 1].set(title='全程指令相邻变化 / 该指令观测范围', ylabel='normalized delta')
        self.save(fig, '07_distributions')

    def export_data(self, details, spectral_data):
        data = self.output / 'data'
        data.mkdir(exist_ok=True)
        mapping = {}
        for key, sig in self.log.signals.items():
            np.savez_compressed(data / (key + '.npz'), time_s=sig.t, values=sig.y)
            mapping[key] = {'file': 'data/' + key + '.npz', 'unit': sig.unit, 'source': sig.source, 'frame': sig.frame}
        write_json(data / 'signal_mapping.json', mapping)
        for key, pair in details.items():
            columns = [pair['t']]
            names = ['receive_time_s']
            for kind in ('actual', 'desired', 'error'):
                columns.extend(pair[kind].T)
                names.extend(f'{kind}_{i+1}' for i in range(pair[kind].shape[1]))
            if pair['angle'] is not None:
                columns.append(pair['angle'])
                names.append('quaternion_error_deg')
            values = np.column_stack(columns)
            keep = np.isfinite(pair['actual']).any(axis=1) | np.isfinite(pair['desired']).any(axis=1)
            np.savetxt(data / ('tracking_' + key + '.csv'), values[keep], delimiter=',', header=','.join(names), comments='')
        for key, entries in spectral_data.items():
            for i, item in enumerate(entries):
                np.savetxt(data / f'psd_{key}_{i}.csv', np.column_stack((item['frequency_hz'], item['psd'])),
                           delimiter=',', header='frequency_hz,psd', comments='')
        return mapping

    def animation(self, phase_data, windows, fps=10, speed=1., max_frames=240):
        phase_t, phase_labels = phase_data
        count = max(2, min(max_frames, int(np.ceil(self.log.duration * fps / speed)) + 1))
        times = np.linspace(0, self.log.duration, count)
        pos, _ = hold(self.log.signals.get('position'), times, self.max_age)
        ref, _ = hold(self.log.signals.get('position_ref'), times, self.max_age)
        q, _ = hold(self.log.signals.get('attitude'), times, self.max_age)
        matrices = rotation_matrices(q) if q.shape[1] == 4 else None
        if matrices is not None and not np.isfinite(matrices).any():
            matrices = None
        actual_speed = self.log.duration * fps / max(1, count - 1)
        fig = plt.figure(figsize=(10, 6), dpi=90)
        grid = fig.add_gridspec(2, 2, width_ratios=[1.15, 1.])
        ax = fig.add_subplot(grid[:, 0], projection='3d')
        altitude = fig.add_subplot(grid[0, 1])
        rates = fig.add_subplot(grid[1, 1])
        fig.subplots_adjust(left=.025, right=.98, bottom=.11, top=.84, wspace=.28, hspace=.48)
        self.lines(altitude, ['position'], '位置 / m')
        self.lines(rates, ['rate'], '角速度 / rad/s')
        cursors = [altitude.axvline(0, color='black'), rates.axvline(0, color='black')]
        has_position = pos.shape[1] == 3 and np.isfinite(pos).any()
        all_positions = []
        for key, label, color in [('position_ref', '参考', 'C1'), ('position', '实际', 'C0')]:
            sig = self.log.signals.get(key)
            if sig is not None:
                points = sig.y[np.isfinite(sig.y).all(axis=1)]
                if len(points):
                    all_positions.append(points)
                    idx = sample_indices(len(sig.t), 5000)
                    if key == 'position_ref':
                        ax.plot(*sig.y[idx].T, '--', color=color, alpha=.35, linewidth=.6, label=label)
        if all_positions:
            points = np.vstack(all_positions)
            center = (points.min(axis=0) + points.max(axis=0))/2
            half = max(.5, float(np.max(np.ptp(points, axis=0)))/2 * 1.1)
            ax.set(xlim=(center[0]-half, center[0]+half), ylim=(center[1]-half, center[1]+half), zlim=(center[2]-half, center[2]+half))
            arm = max(.08, min(.3, half*.08))
        else:
            arm = .15
        ax.set(xlabel='x / m', ylabel='y / m', zlabel='z / m', title='全程飞行回放')
        ax.set_box_aspect([1, 1, 1])
        trail, = ax.plot([], [], [], color='C0', linewidth=1, label='已飞行')
        marker, = ax.plot([], [], [], 'o', color='C0', markersize=4)
        reference_marker, = ax.plot([], [], [], '*', color='C1', markersize=5)
        trail_t, trail_y = times, pos
        if has_position:
            original = self.log.signals['position']
            idx = sample_indices(len(original.t), 8000)
            trail_t, trail_y = original.t[idx], original.y[idx].copy()
            for start in np.flatnonzero(np.diff(original.t) > self.max_age) + 1:
                j = np.searchsorted(idx, start)
                if j < len(trail_y):
                    trail_y[j] = np.nan
        body = [ax.plot([], [], [], color=color, linewidth=2)[0] for color in ('r', 'g', 'b')]
        status = fig.suptitle('')
        frames, contacts = [], []
        contact_indices = set(np.linspace(0, count-1, min(9, count), dtype=int))
        # Palette conversion keeps long GIFs bounded in memory (one byte per pixel).
        for i, t in enumerate(times):
            label = str(phase_labels[min(len(phase_labels)-1, np.searchsorted(phase_t, t, side='right')-1)])
            if interval_mask(np.array([t]), windows)[0]:
                label = '已确认统计的 CMD 区间'
            suffix = '' if matrices is not None else ' | 实际姿态未记录'
            if matrices is not None and not np.isfinite(matrices[i]).all():
                suffix += ' | 当前姿态无效/过旧'
            if has_position:
                trail.set_data_3d(*trail_y[trail_t <= t].T)
                marker.set_data_3d(*pos[i:i+1].T)
                for axis, line in enumerate(body):
                    if matrices is not None and np.isfinite(pos[i]).all() and np.isfinite(matrices[i]).all():
                        points = np.vstack((pos[i], pos[i] + matrices[i, :, axis] * arm))
                        line.set_data_3d(*points.T)
                    else:
                        line.set_data_3d([], [], [])
                if not np.isfinite(pos[i]).all():
                    suffix += ' | 当前无新鲜位置'
            else:
                suffix += ' | 位置缺失，仅回放时间曲线'
            if ref.shape[1] == 3:
                reference_marker.set_data_3d(*ref[i:i+1].T)
            for cursor in cursors:
                cursor.set_xdata([t, t])
            status.set_text(f't={t:.2f}/{self.log.duration:.2f}s | {label}\n播放速度 {actual_speed:.2f}×{suffix}')
            fig.canvas.draw()
            frame = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
            if i in contact_indices:
                contacts.append((t, frame.copy()))
            palette = getattr(Image, 'Palette', Image)
            frames.append(frame.convert('P', palette=palette.ADAPTIVE, colors=128))
        gif = self.output / 'flight_replay.gif'
        frames[0].save(gif, save_all=True, append_images=frames[1:], duration=round(1000/fps), loop=0, optimize=False)
        plt.close(fig)
        sheet, axes = plt.subplots(3, 3, figsize=(15, 10), constrained_layout=True)
        for ax in axes.flat:
            ax.axis('off')
        for ax, (t, frame) in zip(axes.flat, contacts):
            ax.imshow(frame)
            ax.set_title(f'{t:.2f} s')
        sheet.suptitle('全程动画关键帧（PDF 为静态页面；完整动图见 flight_replay.gif）')
        self.save(sheet, '99_replay_contact_sheet')
        return {'file': gif.name, 'frames': count, 'fps': fps, 'playback_speed': actual_speed,
                'covers_recording_start_to_end': True, 'actual_attitude_available': matrices is not None}

    def comparison(self, summary, paths):
        reports = [('本次', summary)]
        for path in paths:
            path = Path(path)
            if path.is_dir():
                path /= 'summary.json'
            manifest_path = path.parent / 'manifest.json'
            if manifest_path.exists() and json.loads(manifest_path.read_text(encoding='utf-8')).get('status') != 'complete':
                raise ValueError(f'对比结果尚未完成或已失败：{path.parent}')
            other = json.loads(path.read_text(encoding='utf-8'))
            if other.get('format_version') != 1:
                raise ValueError(f'不支持的对比报告格式：{path}')
            reports.append((path.parent.name, other))
        rows = []
        keys = [('position', 'norm'), ('velocity', 'norm'), ('attitude', 'three_dimensional_angle'),
                ('body_rate', 'norm'), ('angular_acceleration', 'norm'), ('motor_rpm', 'norm')]
        fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
        for ax, (key, metric) in zip(axes.flat, keys):
            names, values = [], []
            for label, report in reports:
                entry = report.get('tracking', {}).get(key, {})
                result = entry.get('metrics', {}).get(metric, {})
                rmse = result.get('rmse')
                if rmse is None:
                    continue
                names.append(label[:24])
                values.append(rmse)
                rows.append({'report': label, 'source': report['source'], 'quantity': key,
                             'unit': entry.get('unit'), 'rmse': rmse, 'p95_abs': result.get('p95_abs'),
                             'samples': result.get('samples'), 'trajectory_windows_s': report.get('trajectory_windows_s')})
            if values:
                ax.bar(names, values)
                ax.tick_params(axis='x', rotation=30, labelsize=7)
            else:
                ax.text(.5, .5, '没有可比较的已确认指标', transform=ax.transAxes, ha='center')
            unit = next((report.get('tracking', {}).get(key, {}).get('unit') for _, report in reports
                         if report.get('tracking', {}).get(key, {}).get('unit')), '')
            ax.set(title=key, ylabel='RMSE / ' + unit)
        fig.suptitle('各次人工确认轨迹段的指标对比；不同轨迹、速度、参数和数据覆盖率不能直接判定优劣')
        self.save(fig, 'comparison')
        write_json(self.output / 'comparison.json', rows)
        if rows:
            with (self.output / 'comparison.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        return {'reports': [str(p) for p in paths], 'rows': rows,
                'note': '只比较各报告人工确认的轨迹段；没有成功/失败自动判定或跨任务优劣排名。'}

    def pdf(self, summary, conclusions):
        path = self.output / 'report.pdf'
        with PdfPages(path) as pdf:
            pdf.infodict().update(Title='ULog 飞行分析报告', Author='many_controllerV3 flight_analysis')
            self.text_pages(pdf, ['ULog 飞行分析报告', '', *conclusions], '结论与适用范围')
            for image in self.images:
                picture = plt.imread(image)
                ratio = picture.shape[0] / picture.shape[1]
                fig, ax = plt.subplots(figsize=(11.69, max(5, min(16.5, 11.69*ratio))))
                ax.imshow(picture)
                ax.axis('off')
                fig.subplots_adjust(left=.015, right=.985, top=.98, bottom=.025)
                fig.text(.02, .005, image.name, fontsize=7)
                pdf.savefig(fig)
                plt.close(fig)
            sections = ('tracking_decision', 'trajectory_windows_s', 'coverage', 'phases', 'tracking',
                        'outputs', 'smoothness', 'flags', 'timing', 'noise', 'spectra', 'delays',
                        'electrical', 'topic_quality', 'recorder_summary', 'model', 'model_consistency', 'comparison')
            for name in sections:
                if name in summary:
                    self.text_pages(pdf, list(flatten_lines(summary[name])), '数值与方法附录 / ' + name)
        return path

    @staticmethod
    def text_pages(pdf, lines, title):
        wrapped = []
        for line in lines:
            # CJK glyphs occupy about twice the width of ASCII, including in paths.
            text, width = '', 0
            for char in str(line):
                advance = 2 if unicodedata.east_asian_width(char) in ('W', 'F') else 1
                if width + advance > 106:
                    wrapped.append(text)
                    text, width = '', 0
                text += char
                width += advance
            wrapped.append(text)
        for start in range(0, len(wrapped), 46):
            fig = plt.figure(figsize=(8.27, 11.69))
            fig.text(.07, .96, title, fontsize=13, va='top')
            for i, line in enumerate(wrapped[start:start+46]):
                fig.text(.07, .915-i*.0185, line, fontsize=8.4, va='top')
            pdf.savefig(fig)
            plt.close(fig)


def flatten_lines(value, path=''):
    if isinstance(value, dict):
        if 'samples' in value and ('rmse' in value or value.get('available') is False):
            yield path
            yield '  ' + '; '.join(f'{k}={v:.6g}' if isinstance(v, float) else f'{k}={v}' for k, v in value.items())
        else:
            for key, item in value.items():
                yield from flatten_lines(item, path + '/' + str(key))
    elif isinstance(value, (list, tuple)):
        if not value:
            yield path + ': []'
        for i, item in enumerate(value):
            yield from flatten_lines(item, path + f'[{i}]')
    else:
        yield f'{path}: {value}'


def conclusions_for(summary):
    lines = [f'日志：{summary["source"]}', f'记录时长：{summary["duration_s"]:.3f} s。',
             '时间轴：首条消息的主机单调接收时间归零；原始消息时间戳仍保存在 data/raw 中。',
             '对齐：在实际数据时刻取最近的已接收参考值，超过新鲜度门限即作缺失；不跨大缺口插值。',
             '坐标：严格复现工程 x,-y,-z 和四元数 w,x,-y,-z 转换；不是标准 ENU 的轴交换。',
             '全程图、输出变化、频谱、计算耗时和记录质量始终生成；误差指标仅使用确认统计的 CMD 区间。',
             '统计范围：由 state == 3 自动识别，逐段询问是否生成；包含 CMD 内的起飞、稳定、机动和末点保持。',
             '区间采用左闭右开 [开始, 结束)，退出 CMD 的样本不计入；状态过旧的部分跳过，末段最多截止到日志结束。']
    lines.append('自动识别的 CMD 区间（接收时间秒）：' +
                 str(summary['tracking_decision'].get('detected_windows_s', [])))
    if summary['trajectory_windows_s']:
        lines.append('确认统计的 CMD 区间（接收时间秒）：' + str(summary['trajectory_windows_s']))
        for key, entry in summary['tracking'].items():
            metrics = entry['metrics']
            aggregate = metrics.get('three_dimensional_angle', metrics.get('norm', metrics.get('1', {})))
            if aggregate.get('available'):
                lines.append(f'{entry["label"]}：RMSE={aggregate["rmse"]:.5g} {entry["unit"]}，'
                             f'绝对误差P95={aggregate["p95_abs"]:.5g}，最大={aggregate["max_abs"]:.5g}，有效样本={aggregate["samples"]}。')
            else:
                lines.append(entry['label'] + '：确认区间内没有有效对齐样本，未给出数值结论。')
    else:
        lines.append('本次未生成任何轨迹跟踪误差指标：' + summary['tracking_decision']['reason'])
    for item in summary['coverage'].values():
        if item['missing_signals']:
            lines.append(item['label'] + '分析缺少：' + ', '.join(item['missing_signals']) + '。')
    for topic, quality in summary['topic_quality'].items():
        if 'received_hz' in quality:
            hz = quality['received_hz']
            lines.append(f'{topic}：{quality["samples"]}条，接收均频 {hz:.2f} Hz。' if hz is not None else f'{topic}：样本不足以计算频率。')
            if quality.get('sequence_missing', 0):
                lines.append(f'  序号缺失 {quality["sequence_missing"]} 条；结合记录器计数判断，接收间隙不能单独证明 DDS 丢包。')
    for key, result in summary['flags'].items():
        ratios = [v.get('mean') for k, v in result.items() if k != 'norm' and v.get('available')]
        if ratios:
            lines.append(f'{key} 各通道置位样本比例：' + ', '.join(f'{x:.2%}' for x in ratios) + '。')
    for key, entries in summary['spectra'].items():
        peaks = [f'通道{x["axis"]} {x["peak_hz"]:.2f} Hz' for x in entries if x['peak_hz'] is not None]
        if peaks:
            lines.append(key + '各连续段主峰：' + '; '.join(peaks) + '。主峰可能包含参考运动，不自动认定振荡故障。')
    for key, axes in summary['delays'].items():
        for axis in axes:
            for segment in axis['segments']:
                lines.append(f'{key} 通道{axis["axis"]} {segment["start_s"]:.2f}–{segment["end_s"]:.2f}s：'
                             f'相关滞后 {segment["lag_s"]*1000:.2f} ms，相关系数 {segment["correlation"]:.3f}，'
                             + ('可供观察。' if segment['usable'] else '激励/搜索边界不足，不作为可靠延迟。'))
    record = summary.get('recorder_summary', {})
    if record.get('queue_dropped') or record.get('problems') or record.get('writer_errors'):
        lines.append('记录器报告丢弃或异常：' + json.dumps(record, ensure_ascii=False))
    lines.extend([
        '力/力矩语义：tau_sol 是分配模型结果；ESC RPM 为机械转速；ESC 推力/反扭矩为准稳态模型估计；逆动力学力矩包含扰动，均非力传感器实测。',
        '反算公式：omega=RPM*2*pi/60，J=clip(pi*vz/(omega*R+1e-8),0,1e10)。',
        'CT/CQ=clip(a*J²+b*J+c,c/10,c)，ct=4*rho*R^4*CT/pi²，cm=8*rho*R^5*CQ/pi²。',
        'T_i=ct_i*omega_i²，Q_i=cm_i*omega_i²；tau_x=l*sin(beta)*(T1-T2-T3+T4)，tau_y=l*cos(beta)*(-T1-T2+T3+T4)，tau_z=Q1-Q2+Q3-Q4。',
        '电功率=V*I，模型机械功率=Q*omega；16 V 为标定条件，不把实时电压未经标定地乘进推力。',
        '频谱按连续片段重采样到中位采样间隔并做 Welch 估计；不补大缺口。噪声统计包含运动，不能直接用于辨识传感器本底噪声。',
        '统计为有效样本统计；不同数据源采样率、延迟和覆盖率会影响结果。因果保持对齐包含最多一个参考采样周期的保持误差。',
        '程序不判断本次轨迹是否成功。多次飞行对比需要相同任务、参数、速度和统计范围。',
        '动画覆盖整个已记录时间段；未记录的起飞/结束不能补出。缺实际姿态时只显示位置，不用期望姿态冒充。',
    ])
    lines.extend(summary.get('notes', []))
    return lines
