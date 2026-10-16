"""V9-only variable, serial CPU12, frozen r11 AB in every arm."""
import argparse,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
 p.add_argument('--modes',nargs='+',default=['control','A','B','C','ABC'])
 p.add_argument('--diagnostic',action='store_true');p.add_argument('--repeats',type=int,default=3)
 args=p.parse_args();env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
 for mode in args.modes:
  for mid in ('mengjia','arena_large_250x300_2d_005'):
   output=args.output_root/mode/mid
   cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),'--map-id',mid,
        '--output',str(output),'--workload','fixed','--optimization-mode','AB','--v9-mode',mode,
        '--arm','control' if mode=='control' else 'candidate','--repeats',str(1 if args.diagnostic else args.repeats)]
   if args.diagnostic:cmd+=['--queries',*(['Q06','Q09'] if mid=='mengjia' else ['A2B-01','A2B-03'])]
   print('BATCH',mode,mid,flush=True);subprocess.run(cmd,env=env,cwd=ROOT,check=True)
if __name__=='__main__':main()
