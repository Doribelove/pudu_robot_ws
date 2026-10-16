"""Serial V9 comparisons and regressions, frozen r11 AB in every arm."""
import argparse,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
 p.add_argument('--phase',choices=['fixed','shared','regression'],required=True)
 p.add_argument('--final-mode',default='A')
 p.add_argument('--shared-query-dir',type=Path,default=ROOT/'datasets/v9_shared_segments_v2');args=p.parse_args()
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
 def run(mid,mode,out,extra):
  cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),'--map-id',mid,'--output',str(out),
       '--optimization-mode','AB','--v9-mode',mode,'--arm','control' if mode=='control' else 'candidate',*extra]
  print('BATCH',args.phase,mode,mid,out,flush=True);subprocess.run(cmd,cwd=ROOT,env=env,check=True)
 if args.phase=='fixed':
  for mode in dict.fromkeys(['control',args.final_mode,'A','B','C']):
   for mid in ('mengjia','arena_large_250x300_2d_005'):
    run(mid,mode,args.output_root/'fixed'/mode/mid,['--workload','fixed','--repeats','3' if mode in ('control',args.final_mode) else '1'])
 elif args.phase=='shared':
  for mode in ('control','B','C','ABC'):
   for mid in ('mengjia','arena_large_250x300_2d_005'):
    run(mid,mode,args.output_root/'shared'/mode/mid,['--workload','fixed','--query-file',str(args.shared_query_dir/(mid+'.json'))])
 else:
  for work in ('moving','perturbed'):
   for mode in ('control',args.final_mode):
    for mid in ('mengjia','arena_large_250x300_2d_005'):
     extra=['--workload','moving']
     if work=='perturbed':extra+=['--pose-offset','.03','--yaw-offset-deg','2']
     run(mid,mode,args.output_root/work/mode/mid,extra)
  for mid,label in [('mengjia','semantic'),('arena_large_250x300_2d_005','free_space')]:
   for mode in dict.fromkeys([args.final_mode,'ABC']):
    out=args.output_root/'events'/mode/label
    cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_progress_events.py'),'--map-id',mid,'--output',str(out)]
    print('EVENT',mode,mid,flush=True);subprocess.run(cmd,cwd=ROOT,env={**env,'THREEYD_V8_R11_MODE':'AB','THREEYD_V9_MODE':mode},check=True)
if __name__=='__main__':main()
