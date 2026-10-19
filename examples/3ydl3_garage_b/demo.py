#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run the accepted three-layer planner with only the public garage B inputs."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
R4 = '/home/robot/workspaces/3ydl3_v11_r4_tailflow_i_20261016'
V5 = '/home/robot/workspaces/3yd_v5_r11_20260922'
V1 = '/home/robot/workspaces/3yd_v1_semantic_ack_20260915'
FIXED = '/home/robot/文档/ChatGPT/dddd-任务/output/3yd_fixed10_20260922'
MAP_ID = '3yd_synth_twinwing_b_v1'
QUERY_IDS = [f'Q{i:02}' for i in range(1, 11)]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_files():
    manifest = json.loads((ROOT / 'MANIFEST.json').read_text())
    for name, expected in manifest['files'].items():
        p = ROOT / name
        if not p.is_file() or digest(p) != expected:
            raise ValueError(f'DEMO_FILE_CHANGED: {name}')
    with gzip.open(ROOT / 'map/optemap.pgm.gz', 'rb') as f:
        if hashlib.sha256(f.read()).hexdigest() != manifest['map_pgm_sha256']:
            raise ValueError('DECOMPRESSED_MAP_CHANGED')
    return manifest


def prepare_map(folder):
    fixed = Path(folder)
    inp = fixed / MAP_ID / 'inputs'
    inp.mkdir(parents=True)
    for name in ['map.yaml', 'queries.json', 'semantic.json', 'topology.json', 'parking_access.json']:
        shutil.copy2(ROOT / 'map' / name, inp / name)
    with gzip.open(ROOT / 'map/optemap.pgm.gz', 'rb') as src, (inp / 'optemap.pgm').open('wb') as dst:
        shutil.copyfileobj(src, dst)
    shutil.copy2(ROOT / 'map/binding.json', inp.parent / 'binding.json')
    for name in ['registry.json', 'fixed10_binding.py']:
        shutil.copy2(ROOT / 'map' / name, fixed / name)
    return fixed


def namespace_command(output, fixed, venv, command):
    # The legacy absolute paths exist only inside this private namespace.
    # No source or map from the host home directory is visible to the run.
    return [
        'bwrap', '--die-with-parent', '--new-session', '--unshare-pid',
        '--unshare-ipc', '--unshare-uts', '--clearenv',
        '--setenv', 'PATH', '/usr/bin:/bin:/usr/sbin:/sbin',
        '--setenv', 'LANG', 'C.UTF-8', '--setenv', 'HOME', '/tmp/home',
        '--setenv', 'ROS_LOG_DIR', '/output/ros_logs',
        '--ro-bind', '/usr', '/usr', '--ro-bind', '/lib', '/lib',
        '--ro-bind', '/lib64', '/lib64', '--ro-bind', '/bin', '/bin',
        '--ro-bind', '/sbin', '/sbin', '--ro-bind', '/etc', '/etc',
        '--ro-bind', '/opt/ros', '/opt/ros', '--tmpfs', '/home',
        '--dir', '/home/robot/workspaces',
        '--ro-bind', str(ROOT / 'runtime/r4'), R4,
        '--ro-bind', str(ROOT / 'runtime/v5'), V5,
        '--ro-bind', str(ROOT / 'runtime/v1'), V1,
        '--dir', '/home/robot/pudu_robot_ws',
        '--ro-bind', str(fixed), FIXED,
        '--ro-bind', str(venv), '/venv',
        '--ro-bind', str(ROOT), '/demo',
        '--ro-bind', str(ROOT / 'runtime/nav2_sdk'), '/sdk',
        '--tmpfs', '/tmp', '--tmpfs', '/run', '--proc', '/proc',
        '--dev-bind', '/dev', '/dev', '--tmpfs', '/dev/shm',
        '--bind', str(output), '/output', '--chdir', '/output',
        '--', 'bash', '-c', command,
    ]


def run(output, queries, venv, cpu, probe=False):
    verify_files()
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise ValueError('This release supports Linux x86_64 only.')
    if not shutil.which('bwrap'):
        raise ValueError('Install bubblewrap; see README.md.')
    if not (Path('/opt/ros/humble/setup.bash').is_file()):
        raise ValueError('ROS 2 Humble is required; see README.md.')
    if cpu not in os.sched_getaffinity(0):
        raise ValueError('Requested CPU is outside the allowed affinity set.')
    if not (venv / 'lib/python3.10/site-packages/numpy').is_dir():
        raise ValueError('Pass --venv pointing to the pinned Python 3.10 environment.')
    output.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='3ydl3-garage-b-') as temporary:
        fixed = prepare_map(Path(temporary) / 'fixed')
        args = ['python3', R4 + '/tools/run_crossmap_v10.py',
                '--workload', 'fixed', '--l3-backend', 'independent',
                '--v9-mode', 'A', '--review-mode', 'S', '--r2-mode', 'D',
                '--r3-mode', 'control', '--r5-mode', 'C', '--r6-mode', 'H',
                '--r7-mode', 'NT', '--r8-mode', 'OGR', '--r9-mode', 'PLMX',
                '--map-id', MAP_ID, '--domain', '211', '--arm', 'candidate',
                '--output', '/output/run', '--queries', *queries]
        setup = (ROOT / 'runtime/r4/run.sh').read_text().splitlines()
        flags = '\n'.join(line for line in setup if line.startswith('export THREEYD_V10_'))
        body = ('set -eo pipefail\n'
                f'source {R4}/env.bash\n'
                f'export THREEYD_BENCH_SOURCE={R4}\n'
                f'export THREEYD_BENCH_CPU={cpu}\n'
                'export PYTHONNOUSERSITE=1\n'
                'mkdir -p /tmp/home /output/ros_logs\n' + flags + '\n')
        if probe:
            body += 'exec python3 /demo/probe.py\n'
        else:
            body += 'exec ' + shlex.join(args) + '\n'
        with (output / 'console.log').open('w') as log:
            result = subprocess.run(namespace_command(output, fixed, venv, body),
                                    stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f'Run failed ({result.returncode}); inspect {output / "console.log"}')
    if not probe:
        from validate_result import validate
        result = validate(output / 'run', queries)
        (output / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    else:
        print((output / 'console.log').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true', help='Check shipped bytes; requires only Python standard library.')
    parser.add_argument('--probe', action='store_true', help='Check isolated imports and map/query binding, without planning.')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--queries', nargs='+', default=['Q01'])
    parser.add_argument('--all-queries', action='store_true')
    parser.add_argument('--venv', type=Path, default=ROOT / '.venv')
    parser.add_argument('--cpu', type=int, default=min(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else 0)
    a = parser.parse_args()
    if a.verify_only:
        m = verify_files()
        print(f'VERIFIED {len(m["files"])} files; synthetic garage B raster SHA-256 matches')
        return
    queries = QUERY_IDS if a.all_queries else a.queries
    if not queries or len(queries) != len(set(queries)) or not set(queries) <= set(QUERY_IDS):
        parser.error('Select unique Q01–Q10 query IDs.')
    if a.output is None:
        parser.error('--output must be a new directory.')
    run(a.output.resolve(), queries, a.venv.resolve(), a.cpu, a.probe)


if __name__ == '__main__':
    main()
