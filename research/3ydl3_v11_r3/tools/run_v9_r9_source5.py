"""Serial predeclared source5 diagnostics or complete original fixed fixtures."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load, sha

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--phase', choices=('diagnostic', 'fixed'), required=True)
    args = p.parse_args()
    spec = load(ROOT/'report/SOURCE5_RUN_PROTOCOL.json')
    main_spec = load(ROOT/'report/SOURCE5_PROTOCOL.json')
    assert spec['declared_before_native'] and spec['source_cycle'] == main_spec['source_cycle'] == 5
    assert sha(ROOT/'report/SOURCE5_PROTOCOL.json') == spec['implementation_protocol_sha256']
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    assert load(ROOT/'report/PROTECTED_SOURCE5_REVIEW.json')['planning_files'] == 124
    for name, expected in spec['analysis_tools'].items():
        assert sha(ROOT/name) == expected, name
    for name, expected in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name) == expected, name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        state = load(journal)
        assert state['state'] == 'complete' and state['returncode'] == 0 and state['cleanup']['complete'], journal
    phase = main_spec['preliminary_diagnostic' if args.phase == 'diagnostic' else 'formal_fixed']
    output = Path(phase['root'])
    assert not output.exists(), 'Never overwrite a previous or failed stage'
    if args.phase == 'fixed':
        assert load(ROOT/'report/SOURCE5_DIAGNOSTIC_REVIEW.json')['strict_total'] == 64
        protected = load(ROOT/'report/SOURCE5_DIAGNOSTIC_PROTECTED_RESOURCE_REVIEW.json')
        assert protected['planning_files'] == 124
        for name, expected in load(ROOT/'report/SOURCE5_DIAGNOSTIC_REVIEW_LOCK.json').items():
            assert sha(ROOT/'report'/name) == expected, name
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    count = 0
    for kind in (('moving', 'perturbed') if args.phase == 'diagnostic' else ('fixed',)):
        for mid in spec['map_order']:
            query = spec['moving_fixtures'][mid] if args.phase == 'diagnostic' else main_spec['original_queries'][mid]
            assert sha(Path(query['fixture'] if args.phase == 'diagnostic' else query['source'])) == query['sha256']
            for mode in phase['modes']:
                flags = spec['AS_configuration'] if mode == 'AS' else dict(spec['current_configuration'], r9_mode=mode)
                command = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
                           '--map-id', mid, '--output', str(output/kind/mode/mid), '--optimization-mode', 'AB']
                for key, value in flags.items():
                    command += ['--'+key.replace('_', '-'), value]
                if args.phase == 'diagnostic':
                    command += ['--workload', 'moving', '--queries', phase['original_queries'][mid]]
                    if kind == 'perturbed':
                        command += ['--pose-offset', '.03', '--yaw-offset-deg', '2']
                else:
                    command += ['--workload', 'fixed', '--repeats', '3']
                print('SOURCE5_BATCH', kind, mid, mode, 'ORIGINAL_INPUT_CLOCKS_NO_PROFILE_OR_LEDGER', flush=True)
                subprocess.run(command, cwd=ROOT, env=env, check=True)
                count += 4 if args.phase == 'diagnostic' else 30
    assert count == phase['expected_records']
    print('SOURCE5_NATIVE', args.phase, count, 'COMPLETED_REVIEW_REQUIRED', flush=True)


if __name__ == '__main__':
    main()
