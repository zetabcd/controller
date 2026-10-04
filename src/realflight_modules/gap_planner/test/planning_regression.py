#!/usr/bin/env python3
"""Offline regression against the real YAML, without changing user's files."""
import argparse
import copy
import json
from pathlib import Path
import re
import subprocess
import tempfile
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', default='build/gap_planner/gap_plan_check')
    parser.add_argument('--gaps', default='src/realflight_modules/gap_planner/config/gaps.yaml')
    parser.add_argument('--controller', default='src/realflight_modules/px4ctrl/config/params.yaml')
    parser.add_argument('--output', default='/tmp/gap_planning_regression.json')
    args = parser.parse_args()
    source = yaml.safe_load(Path(args.gaps).read_text())
    scenarios = [('current', source, (1, 2, 3), False)]
    scenarios.append(('hover_perturbed', source, (1, 2, 3), True))
    perturbed = copy.deepcopy(source)
    p = perturbed['gap_planner']['ros__parameters']
    for i, name in enumerate(p['gate_order']):
        g = p['gates'][name]
        g['sim_position'][1] += (1 if i % 2 else -1)*0.01
        g['sim_rpy_deg'][0] += (1 if i % 2 else -1)*1.0
    scenarios.append(('gate_perturbed', perturbed, (1, 2, 3), False))
    aligned = copy.deepcopy(source)
    p = aligned['gap_planner']['ros__parameters']
    for i, name in enumerate(p['gate_order']):
        p['gates'][name]['sim_position'] = [-0.9+i, 0, 1]
        p['gates'][name]['sim_rpy_deg'] = [(-30 if i == 1 else 30), 0, 0]
    scenarios.append(('aligned_30deg', aligned, (1, 2, 3), False))
    results = []
    with tempfile.TemporaryDirectory(prefix='gap_regression_') as directory:
        for label, document, counts, noisy in scenarios:
            config = Path(directory)/f'{label}.yaml'
            config.write_text(yaml.safe_dump(document))
            for direction in ('outbound', 'return'):
                for count in counts:
                    command = [args.binary, '--yaml', str(config), str(count), direction, args.controller]
                    if noisy:
                        command += ['0.008', '0.009', '-0.003']
                    completed = subprocess.run(command, text=True, capture_output=True, timeout=20)
                    output = completed.stdout+completed.stderr
                    metrics = {key: float(value) for key, value in re.findall(
                        r'\b(optimize_ms|audit_ms|total_ms|path_length|backward_distance|clearance)=([0-9.eE+-]+)', output.split(' retry[', 1)[0].splitlines()[0] if output else '')}
                    record = dict(scenario=label, count=count, direction=direction,
                                  passed=completed.returncode == 0, metrics=metrics, output=output)
                    results.append(record)
                    print(f'{label} {count} {direction}: {"PASS" if record["passed"] else "FAIL"} {metrics}', flush=True)
    Path(args.output).write_text(json.dumps(results, indent=2))
    failures = [r for r in results if not r['passed']]
    print(f'{len(results)-len(failures)}/{len(results)} passed; full outputs: {args.output}')
    raise SystemExit(bool(failures))


if __name__ == '__main__':
    main()
