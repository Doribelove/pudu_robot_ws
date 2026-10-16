import os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for workload in ('fixed','moving','perturbed'):
 for mid in ('mengjia','arena_large_250x300_2d_005'):
  for arm in ('control','candidate'):
   out=ROOT/'results'/workload/arm/mid
   cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
    '--map-id',mid,'--output',str(out),'--workload','moving' if workload=='perturbed' else workload,
    '--arm',arm,'--repeats','3' if workload=='fixed' else '1']
   if workload=='perturbed':cmd+=['--pose-offset','.03','--yaw-offset-deg','2']
   print('BATCH',workload,arm,mid,flush=True)
   subprocess.run(cmd,env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1'),cwd=ROOT,check=True)
