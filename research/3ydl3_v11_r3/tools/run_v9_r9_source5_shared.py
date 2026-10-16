"""Original unprofiled seed plus three tasks; independent B/P benefit gate."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load, sha

ROOT = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--protocol', type=Path, default=ROOT/'report/SOURCE5_SHARED_PROTOCOL.json')
    args = p.parse_args(); spec = load(args.protocol)
    assert spec['source_cycle'] == 5 and spec['declared_before_shared_native']
    assert not spec['profiler'] and not spec['array_ledger']
    assert tuple(spec['modes']) == ('control', 'PLMX')
    assert tuple(spec['arms']) == ('A', 'ABP') and spec['expected_records'] == 32
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    for name, digest in spec['required_review_locks'].items():
        assert sha(ROOT/'report'/name) == digest, name
    for name, digest in spec['analysis_tools'].items():
        assert sha(ROOT/name) == digest, name
    for name, digest in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name) == digest, name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        record = load(journal)
        assert record['state'] == 'complete' and record['returncode'] == 0 and record['cleanup']['complete']
    out = Path(spec['root']); assert not out.exists(), 'never overwrite any previous block'
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    count = 0
    for mid, fixture_spec in spec['original_queries'].items():
        fixture = Path(fixture_spec['fixture']); definition = load(fixture)
        assert sha(fixture) == fixture_spec['fixture_sha256']
        assert [q['id'] for q in definition['queries']] == fixture_spec['ids']
        assert len(definition['queries']) == fixture_spec['n_all'] == 4 and not definition['fixed10']
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        for arm, flags in spec['arms'].items():
            for mode in spec['modes']:
                command = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
                    '--map-id', mid, '--output', str(out/'shared'/arm/mode/mid), '--optimization-mode', 'AB',
                    '--v9-mode', flags['v9_mode'], '--review-mode', 'S', '--r2-mode', flags['r2_mode'],
                    '--r3-mode', 'control', '--r5-mode', 'C', '--r6-mode', 'H', '--r7-mode', 'NT',
                    '--r8-mode', 'OGR', '--r9-mode', mode, '--workload', 'fixed', '--query-file', str(fixture),
                    '--repeats', '1']
                print('SOURCE5_SHARED_BATCH', mid, arm, mode, 'ORIGINAL_UNPROFILED_SEED_PLUS3', flush=True)
                subprocess.run(command, cwd=ROOT, env=env, check=True)
                count += 4
    assert count == 32
    print('SOURCE5_SHARED32_NATIVE_COMPLETED_REVIEW_REQUIRED', flush=True)

if __name__ == '__main__': main()
