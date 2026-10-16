"""R2-only serial CPU12 controls against the reviewed parent AS."""
import argparse,os,sys,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAPS=('mengjia','arena_large_250x300_2d_005')
def main():
 p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
 p.add_argument('--phase',choices=['fixed','regression','shared','cold'],required=True)
 p.add_argument('--candidate',choices=['D','DP','DPM'],default='D');a=p.parse_args()
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
 def run(mid,label,v9,r2,work,extra):
  out=a.output_root/work/label/mid
  cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),'--map-id',mid,'--output',str(out),
       '--optimization-mode','AB','--v9-mode',v9,'--review-mode','S','--r2-mode',r2,*extra]
  print('BATCH',work,label,mid,flush=True);subprocess.run(cmd,cwd=ROOT,env=env,check=True)
 if a.phase=='fixed':
  for mode in ('control','D','DP'):
   for mid in MAPS:run(mid,mode,'A',mode,'fixed',['--workload','fixed','--repeats','3'])
 elif a.phase=='cold':
  # Three independent processes per map/arm. Alternate order in each round.
  # Each raw point is real preparation+ROS startup+one first response.
  for rep in range(1,4):
   for mode in (('control',a.candidate) if rep%2 else (a.candidate,'control')):
    for mid in MAPS:run(mid,mode,'A',mode,f'cold/r{rep}',['--workload','fixed','--cold-memory','--queries','Q01' if mid=='mengjia' else 'A2B-01'])
 elif a.phase=='shared':
  for label,v9,r2 in [('A','A','D'),('AB','AB','D'),('ABM','AB','DM'),('AP','A','DP'),('ABPM','AB','DPM')]:
   for mid in MAPS:run(mid,label,v9,r2,'shared',['--workload','fixed','--query-file',str(ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json'))])
 else:
  for work in ('moving','perturbed'):
   for mode in ('control',a.candidate):
    for mid in MAPS:
     extra=['--workload','moving']
     if work=='perturbed':extra+=['--pose-offset','.03','--yaw-offset-deg','2']
     run(mid,mode,'A',mode,work,extra)
  for mode in ('control',a.candidate):
   for mid in MAPS:
    out=a.output_root/'events'/mode/mid
    cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_progress_events.py'),'--map-id',mid,'--output',str(out)]
    print('EVENT',mode,mid,flush=True)
    subprocess.run(cmd,cwd=ROOT,env={**env,'THREEYD_V8_R11_MODE':'AB','THREEYD_V9_MODE':'A','THREEYD_V9_R1_MODE':'S','THREEYD_V9_R2_MODE':mode},check=True)
if __name__=='__main__':main()
