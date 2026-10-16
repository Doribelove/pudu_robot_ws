"""Independent reversed source5 original fixed queries; no planning changes."""
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load, sha

ROOT = Path(__file__).resolve().parents[1]

def main():
    spec = load(ROOT/'report/SOURCE5_CONFIRMATION_PROTOCOL.json')
    assert spec['source_cycle'] == 5 and spec['declared_before_confirmation_native']
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    assert load(ROOT/'report/SOURCE5_FIXED_REVIEW.json')['strict_total'] == 300
    assert sha(ROOT/'report/SOURCE5_FIXED_REVIEW_LOCK.json') == spec['fixed_review_lock_sha256']
    for name, expected in load(ROOT/'report/SOURCE5_FIXED_REVIEW_LOCK.json').items():
        assert sha(ROOT/'report'/name) == expected, name
    for name, expected in spec['analysis_tools'].items():
        assert sha(ROOT/name) == expected, name
    for name, expected in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name) == expected, name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        state = load(journal)
        assert state['state'] == 'complete' and state['returncode'] == 0 and state['cleanup']['complete'], journal
    output = Path(spec['root']); assert not output.exists(), 'Never overwrite a prior or failed block'
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    run = load(ROOT/'report/SOURCE5_RUN_PROTOCOL.json')
    count = 0
    for mid in spec['map_order']:
        query = spec['original_queries'][mid]
        assert sha(Path(query['source'])) == query['sha256']
        for mode in spec['mode_order']:
            flags = dict(run['current_configuration'], r9_mode=mode)
            command = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
                       '--map-id', mid, '--output', str(output/'fixed'/mode/mid), '--optimization-mode', 'AB']
            for key, value in flags.items():
                command += ['--'+key.replace('_', '-'), value]
            command += ['--workload', 'fixed', '--repeats', '3']
            print('SOURCE5_CONFIRM_BATCH', mid, mode, 'ORIGINAL_INPUT_CLOCKS_NO_PROFILE_OR_LEDGER', flush=True)
            subprocess.run(command, cwd=ROOT, env=env, check=True)
            count += 30
    assert count == spec['expected_records'] == 180
    print('SOURCE5_CONFIRM_NATIVE180_COMPLETED_REVIEW_REQUIRED', flush=True)

if __name__ == '__main__':
    main()
