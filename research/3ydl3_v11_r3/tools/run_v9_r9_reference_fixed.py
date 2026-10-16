"""Predeclared contemporary original AS/current/candidate fixed10x3 block."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load, sha

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', type=Path, default=ROOT/'report/SOURCE4_REFERENCE_FIXED_PROTOCOL.json')
    args = parser.parse_args()
    spec = load(args.protocol)
    assert spec['source_cycle'] == 4 and spec['declared_before_reference_fixed_native']
    assert spec['expected_records'] == 240 and spec['repeats'] == 3
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    for name, expected in spec['analysis_tools'].items():
        assert sha(ROOT/name) == expected, name
    for name, expected in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name) == expected, name
    assert load(ROOT/'report/SOURCE4_COLD_REVIEW.json')['strict_total'] == 24
    assert sha(ROOT/'report/SOURCE4_COLD_REVIEW_LOCK.json') == spec['cold_review_lock_sha256']
    for name, expected in load(ROOT/'report/SOURCE4_COLD_REVIEW_LOCK.json').items():
        assert sha(ROOT/'report'/name) == expected, name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        state = load(journal)
        assert state['state'] == 'complete' and state['returncode'] == 0 and state['cleanup']['complete'], journal
    out = Path(spec['root'])
    assert not out.exists(), 'Preserve all previous and failed blocks'
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    count = 0
    for mid, order in spec['mode_orders_by_map'].items():
        assert sorted(order) == sorted(spec['modes'])
        query = spec['original_queries'][mid]
        assert sha(Path(query['source'])) == query['sha256']
        for mode in order:
            flags = spec['AS_configuration'] if mode == 'AS' else dict(spec['current_configuration'], r9_mode=mode)
            command = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
                       '--map-id', mid, '--output', str(out/'fixed'/mode/mid), '--optimization-mode', 'AB']
            for key, value in flags.items():
                command += ['--'+key.replace('_', '-'), value]
            command += ['--workload', 'fixed', '--repeats', '3']
            print('REFERENCE_FIXED_BATCH', mid, mode, 'ORIGINAL10x3_NO_PROFILER_OR_LEDGER', flush=True)
            subprocess.run(command, cwd=ROOT, env=env, check=True)
            count += 30
    assert count == spec['expected_records']
    print('REFERENCE_FIXED_NATIVE240_COMPLETED_REVIEW_REQUIRED', flush=True)


if __name__ == '__main__':
    main()
