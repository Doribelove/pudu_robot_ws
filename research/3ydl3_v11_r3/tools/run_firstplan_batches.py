"""Serial fixed-query and moving-start paired workload (CPU12, no profiling)."""
import os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for workload in ('fixed','moving'):
 for mode,mid in [('semantic','mengjia'),('free-space','arena_large_250x300_2d_005')]:
  for arm,flags in [('control','control'),('candidate','AB')]:
   out=ROOT/'results'/workload/arm/mid
   env=dict(os.environ,THREEYD_V8_FIRST_MODE=flags,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
   cmd=['taskset','-c','12',str(ROOT/'run.sh'),'--mode',mode,'--map-id',mid,'--output',str(out),'--domain','220','--repeats','3' if workload=='fixed' else '1']
   if workload=='moving':cmd+=['--query-file',str(ROOT/'datasets/moving_start_v1'/(mid+'.json'))]
   print('BATCH',workload,arm,mid,flush=True)
   subprocess.run(cmd,env=env,cwd=ROOT,check=True)
