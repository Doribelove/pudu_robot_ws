"""Serial complete-task storage experiment; never a moving-window replay."""
import argparse,os,sys,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];MAPS=('mengjia','arena_large_250x300_2d_005')
def main():
 p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
 p.add_argument('--phase',choices=['diagnostic','fixed','cold','regression'],required=True);a=p.parse_args()
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
 def run(mid,mode,kind,extra):
  out=a.output_root/kind/mode/mid
  cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),'--map-id',mid,'--output',str(out),'--optimization-mode','AB','--v9-mode','A','--review-mode','S','--r2-mode','control','--r3-mode','control','--r5-mode',mode,*extra]
  print('BATCH',kind,mode,mid,flush=True);subprocess.run(cmd,cwd=ROOT,env=env,check=True)
 if a.phase=='diagnostic':
  for mode in ('control','C'):
   for mid in MAPS:run(mid,mode,'diagnostic',['--workload','fixed','--cold-memory','--queries',*(['Q01','Q07','Q09'] if mid=='mengjia' else ['A2B-01','A2B-07','A2B-09'])])
 elif a.phase=='fixed':
  for mode in ('control','C'):
   for mid in MAPS:run(mid,mode,'fixed',['--workload','fixed','--repeats','3'])
 elif a.phase=='cold':
  for rep in range(1,4):
   for mode in (('control','C') if rep%2 else ('C','control')):
    for mid in MAPS:run(mid,mode,f'cold/r{rep}',['--workload','fixed','--cold-memory','--queries','Q01' if mid=='mengjia' else 'A2B-01'])
 else:
  for kind in ('moving','perturbed'):
   for mode in ('control','C'):
    for mid in MAPS:
     extra=['--workload','moving']
     if kind=='perturbed':extra+=['--pose-offset','.03','--yaw-offset-deg','2']
     run(mid,mode,kind,extra)
  for mode in ('control','C'):
   for mid in MAPS:
    print('EVENT',mode,mid,flush=True)
    subprocess.run(['taskset','-c','12',sys.executable,str(ROOT/'tools/run_progress_events.py'),'--map-id',mid,'--output',str(a.output_root/'events'/mode/mid)],cwd=ROOT,env={**env,'THREEYD_V8_R11_MODE':'AB','THREEYD_V9_MODE':'A','THREEYD_V9_R1_MODE':'S','THREEYD_V9_R2_MODE':'control','THREEYD_V9_R3_MODE':'control','THREEYD_V9_R5_MODE':mode},check=True)
if __name__=='__main__':main()
