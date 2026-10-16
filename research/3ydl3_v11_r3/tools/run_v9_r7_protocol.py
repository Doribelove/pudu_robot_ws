"""Serial R7 local-connector/storage experiments on original fixed inputs."""
import argparse, os, sys, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAPS=('mengjia','arena_large_250x300_2d_005')
ARMS={'control':('control','control','control'),'N':('control','control','N'),'CH':('C','H','control'),'CHN':('C','H','N'),'CHNT':('C','H','NT')}

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output-root',type=Path,required=True)
    p.add_argument('--phase',choices=['diagnostic','fixed','cold','regression'],required=True)
    p.add_argument('--arms',nargs='+',choices=list(ARMS))
    a=p.parse_args(); arms=a.arms or ['control','CHNT']
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    def run(mid,arm,kind,extra):
        mode,h,n=ARMS[arm]; out=a.output_root/kind/arm/mid
        cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
             '--map-id',mid,'--output',str(out),'--optimization-mode','AB',
             '--v9-mode','A','--review-mode','S','--r2-mode','control','--r3-mode','control',
             '--r5-mode',mode,'--r6-mode',h,'--r7-mode',n,*extra]
        print('BATCH',kind,arm,mid,flush=True)
        subprocess.run(cmd,cwd=ROOT,env=env,check=True)
    if a.phase=='diagnostic':
        for kind in ('moving','perturbed'):
            for arm in arms:
                for mid in MAPS:
                    extra=['--workload','moving','--queries','Q09' if mid=='mengjia' else 'A2B-03']
                    if kind=='perturbed': extra+=['--pose-offset','.03','--yaw-offset-deg','2']
                    run(mid,arm,'diagnostic/'+kind,extra)
    elif a.phase=='fixed':
        for arm in arms:
            for mid in MAPS: run(mid,arm,'fixed',['--workload','fixed','--repeats','3'])
    elif a.phase=='cold':
        for rep in range(1,4):
            for arm in (arms if rep%2 else list(reversed(arms))):
                for mid in MAPS:
                    run(mid,arm,f'cold/r{rep}',['--workload','fixed','--cold-memory','--queries',
                                             'Q01' if mid=='mengjia' else 'A2B-01'])
    else:
        for kind in ('moving','perturbed'):
            for arm in arms:
                for mid in MAPS:
                    extra=['--workload','moving']
                    if kind=='perturbed': extra+=['--pose-offset','.03','--yaw-offset-deg','2']
                    run(mid,arm,kind,extra)
        for arm in arms:
            for mid in MAPS:
                mode,h,n=ARMS[arm]; print('EVENT',arm,mid,flush=True)
                subprocess.run(['taskset','-c','12',sys.executable,str(ROOT/'tools/run_progress_events.py'),
                                '--map-id',mid,'--output',str(a.output_root/'events'/arm/mid)],
                               cwd=ROOT,env={**env,'THREEYD_V8_R11_MODE':'AB','THREEYD_V9_MODE':'A',
                               'THREEYD_V9_R1_MODE':'S','THREEYD_V9_R2_MODE':'control',
                               'THREEYD_V9_R3_MODE':'control','THREEYD_V9_R5_MODE':mode,
                               'THREEYD_V9_R6_MODE':h,'THREEYD_V9_R7_MODE':n},check=True)
if __name__=='__main__': main()
