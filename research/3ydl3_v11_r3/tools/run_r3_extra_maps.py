"""Serial additional semantic and map-only validation; keep failures as results."""
import argparse,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--r3-mode',choices=['A','AB'],required=True);args=p.parse_args()
    stages=[]
    for mode in ('control',args.r3_mode):
      for branch,mid in [('semantic','3yd_synth_multiloop_a_v1'),('free-space','arena_medium_200x200_2d_005')]:
        out=f'results/additional/{mode}/{mid}'
        print('START additional',mode,mid,flush=True);started=time.time()
        env=dict(os.environ,THREEYD_V8_R3_MODE=mode,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        run=subprocess.run(['taskset','-c','12','./run.sh','--mode',branch,'--map-id',mid,'--repeats','1','--output',out],cwd=ROOT,env=env)
        stages.append({'r3_ablation':mode,'mode':branch,'map_id':mid,'output':out,'start':started,'end':time.time(),'exit_code':run.returncode})
        (ROOT/'report/additional_stages.json').write_text(json.dumps(stages,indent=2))
        if run.returncode:raise SystemExit(run.returncode)
        rows=[json.loads(s) for s in (ROOT/out/'runs.jsonl').read_text().splitlines()]
        if len(rows)!=10:raise SystemExit('INCOMPLETE_FIXED10:'+mid)
        print('COMPLETE additional',mode,mid,sum(r['success'] for r in rows),'/10 strict',flush=True)
if __name__=='__main__':main()
