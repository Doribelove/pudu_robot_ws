"""Predeclared original shared tasks; profiling never supplies adoption timing."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from compare_v9_review import load, sha

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', type=Path, default=ROOT/'report/SOURCE4_SHARED_PROFILE_PROTOCOL.json')
    args = parser.parse_args()
    declared = load(args.protocol)
    assert declared['source_cycle'] == 4 and declared['diagnostic_only'] and declared['profiler']
    assert declared['declared_before_profile_Native'] and declared['timing_not_used_for_adoption']
    assert declared['planner_lock_sha256'] == sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')
    original_full = load(ROOT/'report/SOURCE4_FULL_PROGRESS_REVIEW.json')
    assert original_full['strict_progress_total'] == 640 and original_full['source_cycle'] == 4
    assert not original_full['profiler'] and not original_full['diagnostic_subset']
    completed = list((ROOT/'results/full_progress_source4').rglob('*.supervisor.json'))
    assert len(completed) == 16
    for journal in completed:
        record = load(journal)
        assert record['state'] == 'complete' and record['returncode'] == 0 and record['cleanup']['complete']
    for name, expected in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name) == expected, name
    out = Path(declared['root'])
    assert not out.exists(), 'preserve an existing run; never overwrite it'
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    count = 0
    for mid, fixture_spec in declared['original_queries'].items():
        fixture = Path(fixture_spec['fixture'])
        definition = load(fixture)
        assert sha(fixture) == fixture_spec['fixture_sha256']
        assert [query['id'] for query in definition['queries']] == fixture_spec['ids']
        assert len(definition['queries']) == fixture_spec['n_all'] == 4 and not definition['fixed10']
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        for mode in declared['modes']:
            for arm, configuration in declared['arms'].items():
                directory = out/arm/mode/mid
                command = ['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
                    '--map-id',mid,'--output',str(directory),'--optimization-mode','AB',
                    '--v9-mode',configuration['v9_mode'],'--review-mode','S',
                    '--r2-mode',configuration['r2_mode'],'--r3-mode','control',
                    '--r5-mode','C','--r6-mode','H','--r7-mode','NT','--r8-mode','OGR',
                    '--r9-mode',mode,'--workload','fixed','--query-file',str(fixture),
                    '--repeats','1','--profile']
                print('PROFILE_BATCH',mid,mode,arm,'DIAGNOSTIC_NO_ADOPTION_TIMING',flush=True)
                subprocess.run(command,cwd=ROOT,env=env,check=True)
                count += len(definition['queries'])
    assert count == declared['expected_records'] == 48
    print('PROFILE_NATIVE_COMPLETED48_REVIEW_REQUIRED',flush=True)


if __name__ == '__main__':
    main()
