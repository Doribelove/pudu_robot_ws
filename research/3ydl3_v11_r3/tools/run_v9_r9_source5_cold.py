"""Predeclared independent cold processes; original runner clocks/sampler."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load,sha

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--protocol',type=Path,default=ROOT/'report/SOURCE5_COLD_PROTOCOL.json')
    args=parser.parse_args();spec=load(args.protocol)
    assert spec['source_cycle']==5 and spec['declared_before_cold_native']
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')==spec['planner_lock_sha256']
    previous=load(ROOT/'report/SOURCE5_FULL_PROGRESS_REVIEW.json');assert previous['strict_progress_total']==640
    assert sha(ROOT/'report/SOURCE5_FULL_PROGRESS_REVIEW_LOCK.json')==spec['full_progress_review_lock_sha256']
    for name,expected in spec['analysis_tools'].items():
        assert sha(ROOT/name)==expected,name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        d=load(journal);assert d['state']=='complete' and d['returncode']==0 and d['cleanup']['complete']
    for name,expected in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name)==expected,name
    out=Path(spec['root']);assert not out.exists(),'Never overwrite a previous cold block'
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    count=0
    for rep,order in enumerate(spec['mode_orders_by_round'],1):
        assert sorted(order)==sorted(spec['modes'])
        for mid,query in spec['original_queries'].items():
            assert sha(Path(query['source']))==query['sha256']
            for mode in order:
                flags=spec['AS_configuration'] if mode=='AS' else dict(spec['current_configuration'],r9_mode=mode)
                command=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
                    '--map-id',mid,'--output',str(out/'cold'/f'r{rep}'/mode/mid),'--optimization-mode','AB']
                for key,value in flags.items():command += ['--'+key.replace('_','-'),value]
                command += ['--workload','fixed','--repeats','1','--cold-memory','--queries',query['id']]
                print('COLD_BATCH',rep,mid,mode,'ORIGINAL_STARTUP_AND_FIRST_RESPONSE',flush=True)
                subprocess.run(command,cwd=ROOT,env=env,check=True);count+=1
    assert count==spec['expected_records']==24
    print('COLD_NATIVE24_COMPLETED_REVIEW_REQUIRED',flush=True)


if __name__=='__main__':main()
