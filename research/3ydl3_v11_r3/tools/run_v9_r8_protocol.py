"""Serial original-input R8 experiments; profiling is a separate phase."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MAPS = ('mengjia', 'arena_large_250x300_2d_005')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output-root', type=Path, required=True)
    p.add_argument('--phase', choices=['shared', 'diagnostic', 'profile-progress', 'fixed', 'cold', 'regression'], required=True)
    p.add_argument('--modes', nargs='+', choices=['control', 'O', 'F', 'OF', 'G', 'OG', 'FG', 'OFG', 'R', 'GR', 'OGR'], default=['control', 'OGR'])
    a = p.parse_args()
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')

    def run(mid, mode, kind, extra, arm='A'):
        out = a.output_root / kind / mode / (arm if kind == 'shared' else '') / mid
        cmd = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
               '--map-id', mid, '--output', str(out), '--optimization-mode', 'AB',
               '--v9-mode', 'A' if arm == 'A' else 'AB', '--review-mode', 'S',
               '--r2-mode', 'control' if arm == 'A' else 'P', '--r3-mode', 'control',
               '--r5-mode', 'C', '--r6-mode', 'H', '--r7-mode', 'NT', '--r8-mode', mode, *extra]
        print('BATCH', kind, mode, arm, mid, flush=True)
        subprocess.run(cmd, cwd=ROOT, env=env, check=True)

    if a.phase == 'shared':
        for mid in MAPS:
            for arm in ('A', 'ABP'):
                for mode in a.modes:
                    definition = ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json')
                    queries = json.loads(definition.read_text())['queries']
                    ids = [q['id'] for q in queries]
                    if len(ids) != 4 or len(set(ids)) != 4:
                        raise ValueError('R8_REQUIRES_ORIGINAL_SHARED_V2_SEED_AND_THREE_TASKS')
                    run(mid, mode, 'shared', ['--workload', 'fixed', '--query-file',
                        str(definition), '--queries', *ids], arm)
    elif a.phase in ('diagnostic', 'profile-progress'):
        kinds = ('moving',) if a.phase == 'profile-progress' else ('moving', 'perturbed')
        for kind in kinds:
            for mid in MAPS:
                for mode in a.modes:
                    extra = ['--workload', 'moving', '--queries', 'Q09' if mid == 'mengjia' else 'A2B-03']
                    if kind == 'perturbed': extra += ['--pose-offset', '.03', '--yaw-offset-deg', '2']
                    if a.phase == 'profile-progress': extra += ['--profile']
                    run(mid, mode, a.phase+'/'+kind, extra)
    elif a.phase == 'fixed':
        for mode in a.modes:
            for mid in MAPS: run(mid, mode, 'fixed', ['--workload', 'fixed', '--repeats', '3'])
    elif a.phase == 'cold':
        for rep in range(1, 4):
            for mode in a.modes if rep % 2 else list(reversed(a.modes)):
                for mid in MAPS:
                    run(mid, mode, f'cold/r{rep}', ['--workload', 'fixed', '--cold-memory',
                        '--queries', 'Q01' if mid == 'mengjia' else 'A2B-01'])
    else:
        for kind in ('moving', 'perturbed'):
            for mode in a.modes:
                for mid in MAPS:
                    extra = ['--workload', 'moving']
                    if kind == 'perturbed': extra += ['--pose-offset', '.03', '--yaw-offset-deg', '2']
                    run(mid, mode, kind, extra)
        for mode in a.modes:
            for mid in MAPS:
                print('EVENT', mode, mid, flush=True)
                subprocess.run(['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_progress_events.py'),
                                '--map-id', mid, '--output', str(a.output_root/'events'/mode/mid)],
                               cwd=ROOT, env={**env, 'THREEYD_V8_R11_MODE': 'AB', 'THREEYD_V9_MODE': 'A',
                                   'THREEYD_V9_R1_MODE': 'S', 'THREEYD_V9_R2_MODE': 'control',
                                   'THREEYD_V9_R3_MODE': 'control', 'THREEYD_V9_R5_MODE': 'C',
                                   'THREEYD_V9_R6_MODE': 'H', 'THREEYD_V9_R7_MODE': 'NT',
                                   'THREEYD_V9_R8_MODE': mode}, check=True)


if __name__ == '__main__':
    main()
