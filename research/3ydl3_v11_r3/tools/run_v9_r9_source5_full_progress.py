"""Source5 complete original10 progress fixtures; exact and3cm/2degree."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load,sha

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--protocol',type=Path,default=ROOT/'report/SOURCE5_FULL_PROGRESS_PROTOCOL.json')
    args=parser.parse_args();spec=load(args.protocol)
    assert spec['source_cycle']==5 and spec['declared_before_full_progress_native']
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')==spec['planner_lock_sha256']
    assert load(ROOT/'report/SOURCE5_CONFIRMATION_FIXED_REVIEW.json')['strict_total']==180
    assert sha(ROOT/'report/SOURCE5_CONFIRMED_REVIEW_LOCK.json')==spec['confirmation_review_lock_sha256']
    for name,expected in load(ROOT/'report/SOURCE5_CONFIRMED_REVIEW_LOCK.json').items():
        assert sha(ROOT/'report'/name)==expected,name
    for name,expected in spec['analysis_tools'].items():
        assert sha(ROOT/name)==expected,name
    for name,expected in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name)==expected,name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        state=load(journal)
        assert state['state']=='complete' and state['returncode']==0 and state['cleanup']['complete'],journal
    out=Path(spec['root']);assert not out.exists(),'Never overwrite any prior or failed block'
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    run=load(ROOT/'report/SOURCE5_RUN_PROTOCOL.json');count=0
    for kind in ('moving','perturbed'):
        for mid in spec['map_order']:
            assert sha(Path(spec['original_queries'][mid]['fixture']))==spec['original_queries'][mid]['sha256']
            for mode in spec['modes_in_serial_order_per_map_and_kind']:
                flags=run['AS_configuration'] if mode=='AS' else dict(run['current_configuration'],r9_mode=mode)
                command=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
                    '--map-id',mid,'--output',str(out/kind/mode/mid),'--optimization-mode','AB']
                for key,value in flags.items():command+=['--'+key.replace('_','-'),value]
                command+=['--workload','moving']
                if kind=='perturbed':command+=['--pose-offset','.03','--yaw-offset-deg','2']
                print('SOURCE5_FULL_BATCH',kind,mid,mode,'ORIGINAL40_INPUTS_CLOCKS_NO_PROFILE_LEDGER',flush=True)
                subprocess.run(command,cwd=ROOT,env=env,check=True);count+=40
    assert count==spec['expected_records']==640
    print('SOURCE5_FULL_NATIVE640_COMPLETED_REVIEW_REQUIRED',flush=True)

if __name__=='__main__':main()
