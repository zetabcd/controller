"""Command-line workflow. Tracking metrics always require an interactive decision."""
import argparse
from datetime import datetime
import hashlib
from pathlib import Path
import sys
import uuid
import numpy as np
from . import VERSION
from .ulog_data import FlightLog, cmd_windows
from .models import derive
from .metrics import analyze
from .report import Report, configure_font, conclusions_for, write_json


def ask_tracking(log, candidates):
    state = log.signals.get('state')
    decision = {'confirmed': False, 'source': 'fsm_state_cmd', 'cmd_state': 3,
                'state_signal_source': state.source if state is not None else None,
                'detected_windows_s': candidates, 'interval_convention': '[start, end)',
                'segments': [], 'success_automatically_inferred': False,
                'scope': 'state == 3 的完整 CMD 区间，包含其中的起飞、稳定、机动和末点保持。'}
    if state is None or not candidates:
        decision['reason'] = ('日志缺少 FSM state，无法自动识别 CMD，跳过跟踪指标。' if state is None else
                              '日志中没有可用的 CMD 时间区间，跳过跟踪指标。')
        print(decision['reason'])
        return [], decision

    print('\n轨迹指标包括位置、速度、姿态、角速度、角加速度、力矩和电机跟踪误差。')
    print('已根据 state == 3 自动识别 CMD 区间，无需输入时间。')
    print('统计包含 CMD 内的起飞、稳定、机动和末点保持；状态记录中断会拆分区间。')
    print('程序不判断飞行是否成功，请逐段决定是否生成指标。')
    windows, input_closed = [], False
    for index, (start, end) in enumerate(candidates, 1):
        print(f'CMD 第 {index}/{len(candidates)} 段：{start:.3f}–{end:.3f} s（{end-start:.3f} s）')
        confirmed = False
        reason = '标准输入结束，本段未确认。'
        while not input_closed:
            try:
                answer = input('是否生成本段轨迹跟踪指标？[y/N] ').strip().lower()
            except EOFError:
                input_closed = True
                break
            if answer in ('', 'n', 'no', '否', '不'):
                reason = '用户选择不生成本段指标。'
                break
            if answer in ('y', 'yes', '是', '好'):
                confirmed, reason = True, '用户确认生成本段指标。'
                windows.append((start, end))
                break
            print('请输入 y 或 n。')
        decision['segments'].append({'index': index, 'window_s': [start, end],
                                     'confirmed': confirmed, 'reason': reason})
    decision['confirmed'] = bool(windows)
    decision['reason'] = (f'自动识别 {len(candidates)} 个 CMD 区间，用户确认其中 {len(windows)} 段。' if windows else
                          '没有获得任何 CMD 区间的统计确认，跳过跟踪指标。')
    if input_closed:
        decision['reason'] += ' 标准输入结束，其余未确认区间已跳过。'
    print(decision['reason'])
    return windows, decision


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description='ULog 全程诊断、自动识别 CMD 并逐段确认指标、PDF 报告与全程 GIF。')
    parser.add_argument('ulog', type=Path, help='本项目记录器生成的 .ulg 文件')
    parser.add_argument('--output-root', type=Path, help='结果根目录；默认 日志目录/analysis，每次新建唯一子目录')
    parser.add_argument('--max-age', type=float, default=.1, help='对齐允许的最大接收年龄/连续分析缺口，秒（默认 0.1）')
    parser.add_argument('--params', type=Path, help='本次飞行参数 YAML，仅用于缺少日志模型参数时的离线推算')
    parser.add_argument('--compare', type=Path, nargs='+', default=[], help='以往本工具生成的结果目录或 summary.json')
    parser.add_argument('--font', type=Path, help='中文 TTF/TTC 字体；默认自动查找系统中文字体')
    parser.add_argument('--animation-fps', type=int, default=10)
    parser.add_argument('--animation-speed', type=float, default=1., help='期望回放倍速；帧数上限可能进一步加速')
    parser.add_argument('--animation-max-frames', type=int, default=240, help='动图最多帧数，始终覆盖全记录，默认 240')
    args = parser.parse_args(argv)
    if not np.isfinite(args.max_age) or args.max_age <= 0:
        parser.error('--max-age 必须为正数')
    if not 1 <= args.animation_fps <= 60 or args.animation_max_frames < 2:
        parser.error('动图 fps 必须为 1–60，最大帧数至少 2')
    if not np.isfinite(args.animation_speed) or args.animation_speed <= 0:
        parser.error('动图倍速必须为正数')
    output = None
    try:
        if args.ulog.suffix.lower() != '.ulg':
            raise ValueError('请提供 .ulg 文件；旧 CSV 分析入口已移除。')
        font = configure_font(args.font)
        print('读取 ULog：', args.ulog, flush=True)
        log = FlightLog(args.ulog)
        root = args.output_root or log.path.parent / 'analysis'
        output = root.resolve() / (log.path.stem + '_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:6])
        output.mkdir(parents=True, exist_ok=False)
        model = derive(log, args.max_age, args.params)
        report = Report(output, log, args.max_age)
        candidates = cmd_windows(log.signals.get('state'), log.duration, args.max_age)
        preview = report.preview(candidates)
        print(f'记录时长：{log.duration:.3f} s；结果目录：{output}')
        print('先查看全程预览：', preview)
        windows, decision = ask_tracking(log, candidates)
        print('计算诊断并导出数据……', flush=True)
        summary, details, spectral_data, phase_data = analyze(log, windows, decision, args.max_age)
        summary.update(format_version=1, analyzer_version=VERSION, generated_at=datetime.now().astimezone().isoformat(),
                       source_sha256=sha256_file(log.path), model=model, font=font,
                       source_bytes=log.path.stat().st_size)
        log.export(output)
        summary['signal_mapping'] = report.export_data(details, spectral_data)
        report.overview()
        report.tracking(details, windows)
        report.diagnostic_figures(details, spectral_data)
        if args.compare:
            summary['comparison'] = report.comparison(summary, args.compare)
        print('生成覆盖整个记录过程的 GIF……', flush=True)
        summary['animation'] = report.animation(phase_data, windows, args.animation_fps,
                                                args.animation_speed, args.animation_max_frames)
        conclusions = conclusions_for(summary)
        summary['conclusions'] = conclusions
        summary['figures'] = [str(p.relative_to(output)) for p in report.images]
        write_json(output / 'summary.json', summary)
        (output / 'conclusions.md').write_text('# 飞行分析结论\n\n' + '\n\n'.join(conclusions) + '\n', encoding='utf-8')
        print('汇总图片和全部数值结论到 PDF……', flush=True)
        pdf = report.pdf(summary, conclusions)
        write_json(output / 'manifest.json', {'status': 'complete', 'pdf': pdf.name, 'gif': summary['animation']['file'],
                                             'summary': 'summary.json', 'figures': summary['figures']})
        print(f'分析完成。\nPDF：{pdf}\n动图：{output / "flight_replay.gif"}\n结论：{output / "conclusions.md"}')
        return 0
    except Exception as error:
        if output is not None:
            write_json(output / 'manifest.json', {'status': 'failed', 'error': str(error)})
        print('分析失败：' + str(error), file=sys.stderr)
        if output is not None:
            print('部分输出保留于：' + str(output), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        if output is not None:
            write_json(output / 'manifest.json', {'status': 'interrupted'})
        print('\n分析已中断。', file=sys.stderr)
        return 130
