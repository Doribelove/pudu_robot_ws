"""Run final request implementation checks, then the complete paired suite."""
import json
import os
from pathlib import Path
import subprocess
import sys
from run_v10r3_suite import ROOT, run


def main():
    folder=ROOT/'results/diagnostic_final'
    checks=[]
    for mid,qs in [('arena_large_250x300_2d_005',['A2B-01','A2B-06']),
                   ('3yd_synth_trizone_c_v2',['Q01','Q06']),('mengjia',['Q01','Q06'])]:
        for arm,mode in [('r3','ABRTHI'),('r2','control')]:
            run(folder/mid/arm,mid,arm,mode,qs)
        for q in qs:
            a=folder/mid/'r2/cases'/f'{q}_1';b=folder/mid/'r3/cases'/f'{q}_1'
            for name in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy'):
                assert (a/name).read_bytes()==(b/name).read_bytes(),(mid,q,name)
            checks.append((mid,q))
    (folder/'EQUIVALENCE.json').write_text(json.dumps({'all_outputs_identical':checks,'mode':'ABRTHI'},indent=2))
    env=dict(os.environ,THREEYD_V10_R3_MODE='ABRTHI',THREEYD_BENCH_CPU='14')
    for mid in ('mengjia','arena_large_250x300_2d_005'):
        out=ROOT/'results/events_final'/mid
        print('EVENTS '+mid,flush=True)
        with (ROOT/'results'/f'events_final_{mid}.log').open('w') as stream:
            subprocess.run([sys.executable,str(ROOT/'tools/run_v10r3_events.py'),
                            '--map-id',mid,'--output',str(out)],env=env,
                           stdout=stream,stderr=subprocess.STDOUT,check=True)
        j=json.loads(out.with_suffix('.supervisor.json').read_text())
        assert j['returncode']==0 and j['cleanup']['complete']
        assert json.loads((out/'summary.json').read_text())['assertions_passed']
    print('FORMAL START',flush=True)
    subprocess.run([sys.executable,str(ROOT/'tools/run_v10r3_suite.py'),'formal','--mode','ABRTHI',
                    '--output',str(ROOT/'results/paired_final')],check=True)


if __name__=='__main__':main()
