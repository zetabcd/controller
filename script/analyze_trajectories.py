"""Export and audit the four analytic references using the actual project YAML."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
TYPES = ('horizontal_circle', 'vertical_circle', 'helix', 'figure_eight')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'src/realflight_modules/px4ctrl/config/params.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'datalog/trajectory_refactor')
    parser.add_argument('--binary', type=Path, default=ROOT / 'build/px4ctrl/trajectory_inspect')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(args.config.read_text())['px4ctrl_node']['ros__parameters']
    tr, uav, motor = config['trajectory'], config['uav'], config['motor']
    ct = motor['Ct_c'] * 4 * config['aero']['rho'] * uav['rp'] ** 4 / math.pi ** 2
    motor_min = ct * motor['rc2speed_c'] ** 2
    motor_max = ct * sum(motor['rc2speed_' + c] for c in 'abc') ** 2
    common = {k: tr[k] for k in ('takeoff_height', 'takeoff_duration', 'settle_duration')}
    common.update(gravity=config['gra'], mass=uav['mass'], ix=uav['Jvx'], iy=uav['Jvy'], iz=uav['Jvz'],
                  arm=uav['l'], arm_angle=math.radians(uav['beta_deg']),
                  torque_to_thrust=2 * uav['rp'] * motor['Cq_c'] / motor['Ct_c'],
                  motor_min=motor_min, motor_max=motor_max * tr['limits']['motor_fraction'],
                  thrust_min=4 * motor_min / uav['mass'],
                  thrust_max=4 * motor_max * tr['limits']['motor_fraction'] / uav['mass'],
                  angular_acceleration=tr['limits']['angular_acceleration'],
                  minimum_relative_altitude=tr['limits']['minimum_relative_altitude'])
    common.update(zip(('rate_x', 'rate_y', 'rate_z'), config['nmpc']['body_rate_max']))
    reports, data = {}, {}
    failed = False
    for kind in TYPES:
        csv = args.output / (kind + '.csv')
        command = [str(args.binary), '--type', kind, '--output', str(csv)]
        for key, value in (common | tr[kind]).items():
            command += ['--' + key, str(value)]
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode not in (0, 2):
            raise RuntimeError(f'{kind}: {result.stderr}')
        reports[kind] = json.loads(result.stdout)
        reports[kind]['failure_reason'] = result.stderr.strip()
        failed |= result.returncode != 0
        data[kind] = np.genfromtxt(csv, delimiter=',', names=True)
    (args.output / 'metrics.json').write_text(json.dumps(reports, indent=2) + '\n')
    (args.output / 'params.yaml').write_text(args.config.read_text())
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/trajectory_matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16, 13), constrained_layout=True)
    for row, kind in enumerate(TYPES):
        d, report = data[kind], reports[kind]
        ax = fig.add_subplot(4, 3, row * 3 + 1, projection='3d')
        ax.plot(d['px'], d['py'], d['pz'], color='gray', lw=1.0)
        main = (d['t'] >= report['main_start']) & (d['t'] <= report['main_end'])
        ax.plot(d['px'][main], d['py'][main], d['pz'][main], color='tab:blue', lw=1.7)
        if np.ptp(d['py']) < .01:
            ax.set_yticks([0])
        ax.set(xlabel='x [m]', ylabel='y [m]', zlabel='z [m]', title=kind)
        ax.set_box_aspect([max(np.ptp(d[c]), .5) for c in ('px', 'py', 'pz')])
        ax = fig.add_subplot(4, 3, row * 3 + 2)
        ax.plot(d['t'], d['thrust_acc'], label='thrust / mass [m/s²]')
        ax.plot(d['t'], np.sqrt(sum(d[c] ** 2 for c in ('vx', 'vy', 'vz'))), label='speed [m/s]')
        ax.set_xlabel('time [s]'); ax.legend(fontsize=8); ax.grid(alpha=.3)
        ax = fig.add_subplot(4, 3, row * 3 + 3)
        ax.plot(d['t'], np.sqrt(sum(d[c] ** 2 for c in ('wx', 'wy', 'wz'))), label='|omega| [rad/s]')
        ax.plot(d['t'], np.sqrt(sum(d[c] ** 2 for c in ('alphax', 'alphay', 'alphaz'))), label='|alpha| [rad/s²]')
        for t in (report['main_start'], report['main_end']):
            ax.axvline(t, color='gray', ls=':', lw=.8)
        ax.set_xlabel('time [s]'); ax.legend(fontsize=8); ax.grid(alpha=.3)
    fig.savefig(args.output / 'trajectories.png', dpi=160)
    plt.close(fig)
    print(json.dumps(reports, indent=2))
    print(args.output / 'trajectories.png')
    if failed:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
