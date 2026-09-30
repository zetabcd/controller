"""Run isolated real-time ROS/MuJoCo experiments; source ROS setup first."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser()
    p.add_argument('name')
    p.add_argument('--duration', default=43, type=float)
    p.add_argument('--seed', default=42, type=int)
    p.add_argument('--controller-label', default='configured controller')
    p.add_argument('--initial-altitude', default=0.5, type=float)
    p.add_argument('--real-time-factor', default=1.0, type=float,
        help='Physics seconds per wall second; <1 emulates a slow GUI with a shared /clock')
    p.add_argument('--stall-ms', default=0.0, type=float,
        help='Periodic wall-clock stall before physics, followed by normal deadline catch-up')
    p.add_argument('--stall-period', default=0.25, type=float,
        help='Physical seconds between injected stalls')
    p.add_argument('--simulator-config', default=str(HERE/'simulator.yaml'))
    p.add_argument('--config', default=str(ROOT/'src/realflight_modules/px4ctrl/config/params.yaml'))
    p.add_argument('--output-root', default=str(ROOT/'datalog/acados_closed_loop'))
    a = p.parse_args()
    if not 0 < a.real_time_factor <= 1:
        p.error('--real-time-factor must be in (0, 1]')
    if not 0 <= a.stall_ms <= 1000 or not 0 < a.stall_period <= 1000:
        p.error('stall-ms must be in [0, 1000] and stall-period in (0, 1000]')
    out = Path(a.output_root).resolve()/a.name
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID='72', ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=str(out/'ros'), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(HERE/'fastdds.xml'))
    config = Path(a.config).resolve()
    # Freeze the exact startup parameters and binary fingerprint for every run.
    (out/'params.yaml').write_bytes(config.read_bytes())
    executable = ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrl_node'
    sources = ['src/realflight_modules/px4ctrl/src/ommpc_solver.cpp',
        'src/realflight_modules/px4ctrl/src/ommpc.cpp',
        'src/realflight_modules/px4ctrl/src/px4ctrlrate_node.cpp',
        'src/uav_simulator/quadsim_mujoco/quadsim_mujoco/quad.py',
        'script/acados_closed_loop/headless.py']
    (out/'simulator.yaml').write_bytes(Path(a.simulator_config).read_bytes())
    (out/'ratectrl.yaml').write_bytes((ROOT/
        'src/realflight_modules/px4ctrl/config/ratectrl_diagnostics.yaml').read_bytes())
    (out/'manifest.json').write_text(json.dumps(dict(
        config=str(config), duration_s=a.duration, seed=a.seed, initial_altitude=a.initial_altitude,
        real_time_factor=a.real_time_factor,
        stall_ms=a.stall_ms, stall_period_s=a.stall_period,
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        inner_executable_sha256=hashlib.sha256((ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrlrate_node').read_bytes()).hexdigest(),
        source_sha256={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
                       for path in sources if (ROOT/path).exists()},
        ros_domain_id=env['ROS_DOMAIN_ID'], controller=a.controller_label,
        timing='fixed physics and motor step, ROS simulation clock'), indent=2)+'\n')
    config = out/'params.yaml'
    commands = [
        [str(ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrl_node'), '--ros-args', '--params-file', str(config), '-p', 'use_sim_time:=true'],
        [str(ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrlrate_node'), '--ros-args', '--params-file',
         str(out/'ratectrl.yaml'), '-p', 'use_sim_time:=true'],
        [sys.executable, str(HERE/'headless.py'), '--output', str(out/'raw.npz'),
         '--duration', str(a.duration), '--seed', str(a.seed),
         '--real-time-factor', str(a.real_time_factor),
         '--stall-ms', str(a.stall_ms), '--stall-period', str(a.stall_period),
         '--initial-altitude', str(a.initial_altitude), '--ros-args',
         '--params-file', str(out/'simulator.yaml')]]
    children, logs = [], []
    try:
        for name, cmd in zip(['outer','inner','sim'], commands):
            log = open(out/(name+'.log'), 'w')
            logs.append(log)
            children.append(subprocess.Popen(cmd, cwd=ROOT, env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        result = children[-1].wait(timeout=a.duration/a.real_time_factor+45)
        if result:
            raise RuntimeError(f'Simulator exit {result}; see {out}/sim.log')
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        for log in logs:
            log.close()
    print(out, flush=True)


if __name__ == '__main__':
    main()
