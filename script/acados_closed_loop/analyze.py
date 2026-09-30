"""Compare current analytic trajectories using their generated evaluation bounds."""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]


def trajectory_info(folder):
    p = yaml.safe_load((folder/'params.yaml').read_text())['px4ctrl_node']['ros__parameters']
    t = p['trajectory']
    kind = t['type']
    if kind == 'omtraj':
        raise ValueError('CSV trajectories need explicit evaluation bounds; use an analytic trajectory here')
    fields = {k: t[k] for k in ('takeoff_height', 'takeoff_duration', 'settle_duration') if k in t}
    fields.update(t.get(kind, {}))
    fields['gravity'] = p['gra']
    cmd = [str(ROOT/'build/px4ctrl/trajectory_inspect'), '--type', kind, '--motor_max', '100']
    for key, value in fields.items():
        cmd += ['--'+key, str(value)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode not in (0, 2):
        raise RuntimeError(result.stderr)
    info = json.loads(result.stdout)
    info['kind'] = kind
    info['motion_start'] = t.get('takeoff_duration', 3)+t.get('settle_duration', .5)
    return info


def interpolate(values, stamps, target):
    return np.column_stack([np.interp(target, stamps, values[:, i]) for i in range(values.shape[1])])


def load(folder):
    raw = np.load(folder/'raw.npz')
    s, d, m = (raw[k] for k in ('state', 'debug', 'motors'))
    d = d[d[:, 0] >= s[0, 0]]
    _, indices = np.unique(d[:, 0], return_index=True)
    d = d[indices]
    if not np.any(d[:, 1] == 3):
        raise ValueError(f'{folder}: CMD never activated')
    start = d[d[:, 1] == 3][0, 0]
    info = trajectory_info(folder)
    t = s[:, 0]-start
    ref = interpolate(d[:, 2:5], d[:, 0], s[:, 0])
    err = s[:, 3:6]-ref
    norm = np.linalg.norm(err, axis=1)
    w = (t >= info['main_start']) & (t <= info['main_end'])
    motion = (t >= info['motion_start']) & (t <= info['duration'])
    if not w.any() or t[-1] < info['duration']+2:
        raise ValueError(f'{folder}: incomplete trajectory or final hover')
    flight = t >= 0
    airborne = bool(np.min(s[flight, 5]) > .12)
    lag = []
    for axis in range(3):
        if not airborne or np.ptp(ref[w, axis]) < .05:
            lag.append(None)
            continue
        shifts = np.arange(-.30, .401, .0025)
        scores = [np.mean((s[w, 3+axis]-np.interp(s[w, 0]-shift, d[:, 0], d[:, 2+axis]))**2)
                  for shift in shifts]
        best = int(np.argmin(scores))
        lag.append(float(shifts[best]*1000) if 0 < best < len(shifts)-1 else None)
    dw = (d[:, 0] >= start+info['motion_start']) & (d[:, 0] <= start+info['duration'])
    intervals = np.diff(d[dw, 0])
    settle = (t >= info['duration']+3) & (t <= info['duration']+8)
    metrics = dict(name=folder.name, seed=int(raw['seed']), trajectory=info['kind'],
        main_window_s=[info['main_start'], info['main_end']],
        main_rmse_m=float(np.sqrt(np.mean(norm[w]**2))), main_max_m=float(norm[w].max()),
        maneuver_rmse_m=float(np.sqrt(np.mean(norm[motion]**2))),
        maneuver_max_m=float(norm[motion].max()), effective_lag_xyz_ms=lag,
        final_hover_rmse_m=float(np.sqrt(np.mean(norm[settle]**2))) if settle.any() else None,
        remained_airborne=airborne, minimum_altitude_m=float(s[flight, 5].min()),
        finite=bool(np.isfinite(s).all() and np.isfinite(d).all() and np.isfinite(m).all()),
        outer_interval_ms_percentiles=np.quantile(intervals*1000, [.5,.95,.99,1]).tolist(),
        physics_wall_ratio=float((s[-1, 1]-s[0, 1])/(s[-1, 20]-s[0, 20])))
    mw = (m[:, 0] >= start+info['motion_start']) & (m[:, 0] <= start+info['duration'])
    metrics['output_throttle_min_max'] = [float(m[mw, 1:].min()), float(m[mw, 1:].max())]
    metrics['output_throttle_rail_fraction'] = float(np.mean((m[mw, 1:] <= 0) | (m[mw, 1:] >= .9999)))
    # Held body-rate command versus physical body rate, in FLU. This is an
    # inner-loop diagnostic, not the trajectory's attitude error or latency.
    command_index = np.clip(np.searchsorted(d[:, 0], s[:, 0], side='right')-1, 0, len(d)-1)
    rate_error = s[:, 9:12]-d[command_index, 8:11]
    metrics['maneuver_rate_rmse_rad_s'] = np.sqrt(np.mean(rate_error[motion]**2, axis=0)).tolist()
    metrics['maneuver_position_p95_m'] = float(np.quantile(norm[motion], .95))
    if d.shape[1] >= 16:
        q = d[:, 12:16].copy()
        for k in range(1, len(q)):
            if q[k-1]@q[k] < 0:
                q[k] *= -1
        qr = interpolate(q, d[:, 0], s[:, 0])
        qr /= np.maximum(1e-12, np.linalg.norm(qr, axis=1))[:, None]
        angle = np.degrees(2*np.arccos(np.clip(abs(np.sum(qr*s[:, 12:16], axis=1)), 0, 1)))
        metrics['maneuver_attitude_rmse_deg'] = float(np.sqrt(np.mean(angle[motion]**2)))
        metrics['maneuver_attitude_max_deg'] = float(angle[motion].max())
    if 'solver' in raw and raw['solver'].size:
        q = raw['solver']
        q = q[(q[:, 0] >= start) & (q[:, 0] <= start+info['duration'])]
        metrics['solver_fallback_fraction'] = float(np.mean(q[:, 5]))
        metrics['solver_time_ms_percentiles'] = np.quantile(q[:, 3], [.5,.95,.99,1]).tolist()
        metrics['solver_status_counts'] = {str(int(v)):int(n) for v,n in zip(*np.unique(q[:, 1], return_counts=True))}
    (folder/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    return metrics, (t,s,ref,norm,motion,info)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('names', nargs='+')
    parser.add_argument('--output-root', type=Path, default=ROOT/'datalog/acados_closed_loop')
    args = parser.parse_args()
    results = []
    fig, ax = plt.subplots(1, 2, figsize=(11,4))
    for name in args.names:
        metrics, (t,s,ref,norm,w,info) = load(args.output_root/name)
        results.append(metrics)
        axes = {'vertical_circle': (3,5), 'helix': (4,5)}.get(info['kind'], (3,4))
        ax[0].plot(s[w, axes[0]], s[w, axes[1]], label=name)
        ax[1].plot(t[w], norm[w], label=name)
        print(json.dumps(metrics))
    ax[0].plot(ref[w, axes[0]-3], ref[w, axes[1]-3], 'k--', label='reference')
    ax[0].set_xlabel('xyz'[axes[0]-3]+' [m]')
    ax[0].set_ylabel('xyz'[axes[1]-3]+' [m]')
    ax[0].axis('equal');ax[0].set_title('Trajectory [m]')
    ax[1].set_title('3D position error [m]');ax[1].set_xlabel('Time since CMD [s]')
    for a in ax:
        a.grid();a.legend(fontsize=8)
    fig.tight_layout();fig.savefig(args.output_root/'comparison.png', dpi=160)
    (args.output_root/'comparison.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
