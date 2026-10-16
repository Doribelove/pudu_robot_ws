"""Predeclared original44 event assertions; wrapper preserves original event clocks."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from compare_v9_review import load, sha

ROOT = Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--protocol',type=Path,default=ROOT/'report/SOURCE5_EVENTS_PROTOCOL.json')
    args=p.parse_args();spec=load(args.protocol)
    assert spec['source_cycle']==5 and spec['declared_before_events_native']
    assert spec['expected_assertion_events']==44 and tuple(spec['modes'])==('control','PLMX')
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')==spec['planner_lock_sha256']
    for name,digest in spec['required_review_locks'].items():assert sha(ROOT/'report'/name)==digest,name
    for name,digest in spec['analysis_tools'].items():assert sha(ROOT/name)==digest,name
    for name,digest in load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():assert sha(ROOT/name)==digest,name
    for name,digest in spec['original_input_files'].items():assert sha(ROOT/name)==digest,name
    for journal in (ROOT/'results').rglob('*.supervisor.json'):
        d=load(journal);assert d['state']=='complete' and d['returncode']==0 and d['cleanup']['complete']
    root=Path(spec['root']);assert not root.exists(),'never overwrite a previous event block'
    count=0
    for mid in spec['map_order']:
        for mode in spec['modes']:
            env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',
                THREEYD_V8_R11_MODE='AB',THREEYD_V9_MODE='A',THREEYD_V9_R1_MODE='S',
                THREEYD_V9_R2_MODE='control',THREEYD_V9_R3_MODE='control',THREEYD_V9_R5_MODE='C',
                THREEYD_V9_R6_MODE='H',THREEYD_V9_R7_MODE='NT',THREEYD_V9_R8_MODE='OGR',THREEYD_V9_R9_MODE=mode)
            command=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_v9_r9_source5_events.py'),
                '--map-id',mid,'--output',str(root/'events'/mode/mid)]
            print('SOURCE5_EVENTS_BATCH',mid,mode,'ORIGINAL11_ASSERTIONS_WITH_INTENDED_FAILURES',flush=True)
            subprocess.run(command,cwd=ROOT,env=env,check=True);count+=11
    assert count==44
    print('SOURCE5_EVENTS44_NATIVE_COMPLETED_REVIEW_REQUIRED_NOT44_STRICT_PLANNING_SUCCESSES',flush=True)

if __name__=='__main__':main()
