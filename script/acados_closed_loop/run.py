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
    p.add_argument('--config', default=str(ROOT/'src/realflight_modules/px4ctrl/config/params.yaml'))
    p.add_argument('--output-root', default=str(ROOT/'datalog/acados_closed_loop'))
    a = p.parse_args()
    out = Path(a.output_root).resolve()/a.name
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID='72', ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=str(out/'ros'), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(HERE/'fastdds.xml'))
    config = Path(a.config).resolve()
    # Freeze the exact startup parameters and binary fingerprint for every run.
    (out/'params.yaml').write_bytes(config.read_bytes())
    executable = ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrl_node'
    (out/'manifest.json').write_text(json.dumps(dict(
        config=str(config), duration_s=a.duration, seed=a.seed,
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        ros_domain_id=env['ROS_DOMAIN_ID'], model='13-state rate-lag + body aerodynamics'), indent=2)+'\n')
    config = out/'params.yaml'
    commands = [
        [str(ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrl_node'), '--ros-args', '--params-file', str(config), '-p', 'use_sim_time:=true'],
        [str(ROOT/'install/px4ctrl/lib/px4ctrl/px4ctrlrate_node'), '--ros-args', '--params-file',
         str(ROOT/'src/realflight_modules/px4ctrl/config/ratectrl_diagnostics.yaml'), '-p', 'use_sim_time:=true'],
        [sys.executable, str(HERE/'headless.py'), '--output', str(out/'raw.npz'),
         '--duration', str(a.duration), '--seed', str(a.seed), '--ros-args',
         '--params-file', str(HERE/'simulator.yaml')]]
    children, logs = [], []
    try:
        for name, cmd in zip(['outer','inner','sim'], commands):
            log = open(out/(name+'.log'), 'w')
            logs.append(log)
            children.append(subprocess.Popen(cmd, cwd=ROOT, env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        result = children[-1].wait(timeout=a.duration+45)
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
