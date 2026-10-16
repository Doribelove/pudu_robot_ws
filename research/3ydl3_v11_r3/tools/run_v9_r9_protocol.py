"""Serial, original fixed fixtures; profile and diagnostic phases stay separate."""
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
    parser.add_argument('--phase', choices=['diagnostic', 'profile-tail', 'fixed', 'shared'], required=True)
    parser.add_argument('--modes', nargs='+', choices=['control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU'], default=['control', 'P'])
    parser.add_argument('--storage-ledger', action='store_true')
    args = parser.parse_args()
    if len(set(args.modes)) != len(args.modes):
        parser.error('modes must be unique')
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    kinds = ('moving', 'perturbed') if args.phase == 'diagnostic' else (args.phase,)
    for kind in kinds:
        for mid in (MAPS[1:] if args.phase == 'profile-tail' else MAPS):
            arms = ('A', 'ABP') if args.phase == 'shared' else ('A',)
            for arm in arms:
                for mode in args.modes:
                    output = args.output_root / kind / mode / (arm if kind == 'shared' else '') / mid
                    extra = []
                    if args.phase in ('diagnostic', 'profile-tail'):
                        extra = ['--workload', 'moving', '--queries', 'Q09' if mid == 'mengjia' else 'A2B-08']
                        if kind == 'perturbed':
                            extra += ['--pose-offset', '.03', '--yaw-offset-deg', '2']
                        if args.phase == 'profile-tail':
                            extra += ['--profile']
                    elif args.phase == 'fixed':
                        extra = ['--workload', 'fixed', '--repeats', '3']
                    else:
                        definition = ROOT / 'datasets/v9_shared_segments_v2' / (mid + '.json')
                        extra = ['--workload', 'fixed', '--query-file', str(definition)]
                    if args.storage_ledger:
                        if args.phase != 'diagnostic':raise ValueError('ledger only in diagnostic, never formal performance')
                        extra += ['--storage-ledger']
                    command = ['taskset', '-c', '12', sys.executable, str(ROOT / 'tools/run_task_progress.py'),
                               '--map-id', mid, '--output', str(output), '--optimization-mode', 'AB',
                               '--v9-mode', 'A' if arm == 'A' else 'AB', '--review-mode', 'S',
                               '--r2-mode', 'control' if arm == 'A' else 'P', '--r3-mode', 'control',
                               '--r5-mode', 'C', '--r6-mode', 'H', '--r7-mode', 'NT', '--r8-mode', 'OGR',
                               '--r9-mode', mode, *extra]
                    print('BATCH', kind, arm, mode, mid, flush=True)
                    subprocess.run(command, cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
