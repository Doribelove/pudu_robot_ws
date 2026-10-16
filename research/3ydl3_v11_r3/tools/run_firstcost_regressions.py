"""r10 continuation retained in BOTH arms; new data prep is sole variable."""
import argparse,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,default=ROOT/'results')
RESULTS=p.parse_args().output_root.resolve()
env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
for workload in ('moving','perturbed'):
 for mid in ('mengjia','arena_large_250x300_2d_005'):
  for arm in ('control','candidate'):
   out=RESULTS/workload/arm/mid
   cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
        '--map-id',mid,'--output',str(out),'--workload','moving','--arm',arm,
        '--optimization-mode','control' if arm=='control' else 'AB']
   if workload=='perturbed':cmd+=['--pose-offset','.03','--yaw-offset-deg','2']
   print('BATCH',workload,arm,mid,flush=True);subprocess.run(cmd,env=env,cwd=ROOT,check=True)
for mid,mode in [('mengjia','semantic'),('arena_large_250x300_2d_005','free_space')]:
 out=RESULTS/'events'/mode
 cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_progress_events.py'),'--map-id',mid,'--output',str(out)]
 print('EVENT',mode,flush=True);subprocess.run(cmd,env={**env,'THREEYD_V8_R11_MODE':'AB'},cwd=ROOT,check=True)
