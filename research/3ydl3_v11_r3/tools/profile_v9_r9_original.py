"""Serial CPU12 profiling of the original A2B-08 four-step fixture only."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for name, modes in [('AS', ['control', 'control', 'control', 'control']),
                        ('OGR', ['C', 'H', 'NT', 'OGR'])]:
        out = args.output_root / name / 'arena_large_250x300_2d_005'
        command = ['taskset', '-c', '12', sys.executable, str(ROOT / 'tools/run_task_progress.py'),
                   '--map-id', 'arena_large_250x300_2d_005', '--output', str(out),
                   '--workload', 'moving', '--queries', 'A2B-08', '--profile',
                   '--optimization-mode', 'AB', '--v9-mode', 'A', '--review-mode', 'S',
                   '--r2-mode', 'control', '--r3-mode', 'control']
        for flag, value in zip(('r5', 'r6', 'r7', 'r8'), modes):
            command.extend([f'--{flag}-mode', value])
        print('PROFILE ORIGINAL FIXTURE', name, out, flush=True)
        subprocess.run(command, cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
