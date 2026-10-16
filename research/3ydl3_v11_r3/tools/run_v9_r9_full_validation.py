"""Original full10 contemporary AS/control/candidate requests, CPU12 serial."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MAPS = ('mengjia', 'arena_large_250x300_2d_005')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--phase', choices=('progress', 'fixed'), required=True)
    parser.add_argument('--modes', nargs='+', choices=('AS', 'control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU'), default=('AS', 'control', 'P', 'PL'))
    args = parser.parse_args()
    if len(set(args.modes)) != len(args.modes):
        parser.error('modes must be unique')
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    kinds = ('moving', 'perturbed') if args.phase == 'progress' else ('fixed',)
    for kind in kinds:
        for mid in MAPS:
            for mode in args.modes:
                original = mode == 'AS'
                command = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
                           '--map-id', mid, '--output', str(args.output_root/kind/mode/mid),
                           '--optimization-mode', 'AB', '--v9-mode', 'A', '--review-mode', 'S',
                           '--r2-mode', 'control', '--r3-mode', 'control',
                           '--r5-mode', 'control' if original else 'C',
                           '--r6-mode', 'control' if original else 'H',
                           '--r7-mode', 'control' if original else 'NT',
                           '--r8-mode', 'control' if original else 'OGR',
                           '--r9-mode', 'control' if original else mode,
                           '--workload', 'moving' if args.phase == 'progress' else 'fixed',
                           '--repeats', '1' if args.phase == 'progress' else '3']
                if kind == 'perturbed':
                    command += ['--pose-offset', '.03', '--yaw-offset-deg', '2']
                print('BATCH', kind, mode, mid, flush=True)
                subprocess.run(command, cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
