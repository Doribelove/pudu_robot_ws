"""Serial, fresh-process L1 controls; 3 launches x fixed10 per map/cohort."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = Path('/home/robot/文档/ChatGPT/dddd-任务')
CORPUS = PROJECT / 'output/3yd_r13_astar_experiment_record_20260923'
SEM = ['pudu_wanda_3f', 'mengjia', '3yd_synth_multiloop_a_v1',
       '3yd_synth_twinwing_b_v1', '3yd_synth_trizone_c_v2']
FREE = ['arena_medium_200x200_2d_005', 'mentor_map_20260825_005',
        'mentor_map_20260825_005_4x_area', 'arena_large_250x300_2d_005']
FLAGS = {'control': ('0', '0'), 'A': ('1', '0'),
         'B': ('0', '1'), 'AB': ('1', '1')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--cohorts', nargs='+', choices=list(FLAGS), default=list(FLAGS))
    parser.add_argument('--maps', nargs='+', default=SEM + FREE)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(PROJECT / 'output/3yd_fixed10_20260922'))
    from fixed10_binding import resolve
    manifest = json.loads((CORPUS / 'manifest.json').read_text())
    bindings = {}
    for map_id in args.maps:
        if map_id not in SEM + FREE:
            raise ValueError('Unregistered map: ' + map_id)
        if map_id in SEM and map_id != 'mengjia':
            _, binding, _ = resolve(map_id)
            entry = next(m for m in manifest['maps'] if m['map_id'] == map_id)
            assert binding['input_sha256'] == entry['input_hashes']
            bindings[map_id] = binding
    protocol = {
        'version': '3YD-V7-r1', 'maps': args.maps, 'rounds': args.rounds,
        'flags': FLAGS, 'cpu': 12, 'candidate_limit': 3,
        'first_budget': 50000, 'alternate_budget': 10000,
        'fixed_bindings': bindings, 'timing': 'routes+all channel_plan calls; initialization and audit excluded',
        'processes': 'serial fresh process per map/cohort/round; no L2/L3/ROS',
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [*sorted((ROOT/'src').rglob('*.py')),
                                    ROOT/'native/goal_dijkstra.cpp', ROOT/'native/libgoal_dijkstra.so']},
    }
    (args.output / 'PROTOCOL.json').write_text(json.dumps(protocol, ensure_ascii=False, indent=2)+'\n')
    for round_id in range(1, args.rounds+1):
        for map_id in args.maps:
            mode = 'semantic' if map_id in SEM else 'free_space'
            # Reverse alternate rounds to reduce a fixed control-first bias.
            cohorts = args.cohorts if round_id % 2 else args.cohorts[::-1]
            for cohort in cohorts:
                if cohort == 'A' and mode == 'semantic' or cohort == 'B' and mode == 'free_space':
                    continue
                out = args.output / cohort / mode / map_id / f'round_{round_id}'
                flags = FLAGS[cohort]
                env = dict(os.environ, THREEYD_V7_GOAL_STOP=flags[0],
                           THREEYD_V7_COMPACT_LEGACY=flags[1],
                           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
                script = ROOT / 'tools' / ('l1_only_worker.py' if mode == 'semantic' else 'run_v7_l1.py')
                command = [sys.executable, str(script), '--map-id', map_id, '--output', str(out)]
                if mode == 'semantic':
                    command += ['--round', str(round_id), '--first-budget', '50000', '--alternate-budget', '10000']
                print(f'START {cohort} {map_id} round={round_id}', flush=True)
                result = subprocess.run(command, env=env, text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, timeout=180)
                out.mkdir(parents=True, exist_ok=True)
                (out/'worker.log').write_text(result.stdout)
                print(result.stdout.strip().splitlines()[-1] if result.stdout.strip() else 'No worker output', flush=True)
                if result.returncode:
                    raise RuntimeError(f'Worker failed: {out}, code={result.returncode}')
    print('DONE', args.output, flush=True)


if __name__ == '__main__':
    main()
