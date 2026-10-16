"""Independent V9-r1 serial CPU12 protocol, no production pointers."""
import argparse,os,sys,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAPS=('mengjia','arena_large_250x300_2d_005')


def main():
 p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
 p.add_argument('--phase',choices=['fixed','regression','shared'],required=True)
 p.add_argument('--candidate',choices=['AS','ASK'],default='ASK');a=p.parse_args()
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
 def run(mid,label,v9,review,extra):
  out=a.output_root/extra[0]/label/mid
  cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
       '--map-id',mid,'--output',str(out),'--optimization-mode','AB','--v9-mode',v9,
       '--review-mode',review,'--arm','control' if label=='control' else 'candidate',*extra[1:]]
  print('BATCH',a.phase,label,mid,flush=True);subprocess.run(cmd,cwd=ROOT,env=env,check=True)
 selected=('A','S' if a.candidate=='AS' else 'SK')
 if a.phase=='fixed':
  for label,v9,review in [('control','control','control'),('S','control','S'),('AS','A','S'),('ASK','A','SK')]:
   for mid in MAPS:run(mid,label,v9,review,['fixed','--workload','fixed','--repeats','3'])
 elif a.phase=='shared':
  # B attribution holds all map/storage/joint settings fixed in each pair.
  for label,v9,review in [('A','A','SK'),('AB','AB','SK'),('ACJ','AC','SJK'),('ABCJ','ABC','SJK')]:
   for mid in MAPS:run(mid,label,v9,review,['shared','--workload','fixed','--query-file',str(ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json'))])
 else:
  for work in ('moving','perturbed'):
   for label,v9,review in [('control','control','control'),(a.candidate,*selected)]:
    for mid in MAPS:
     extra=[work,'--workload','moving']
     if work=='perturbed':extra+=['--pose-offset','.03','--yaw-offset-deg','2']
     run(mid,label,v9,review,extra)
  for label,v9,review in [('control','control','control'),(a.candidate,*selected)]:
   for mid in MAPS:
    out=a.output_root/'events'/label/mid
    cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_progress_events.py'),'--map-id',mid,'--output',str(out)]
    print('EVENT',label,mid,flush=True)
    subprocess.run(cmd,cwd=ROOT,env={**env,'THREEYD_V8_R11_MODE':'AB','THREEYD_V9_MODE':v9,'THREEYD_V9_R1_MODE':review},check=True)
if __name__=='__main__':main()
