"""Serial CPU12 comparisons, task continuation is enabled in BOTH arms."""
import argparse,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,default=ROOT/'results')
RESULTS=p.parse_args().output_root.resolve()
env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
for mode in ('control','A','B','AB'):
 for mid in ('mengjia','arena_large_250x300_2d_005'):
  out=(RESULTS/'accepted_fixed'/('control' if mode=='control' else 'candidate')/mid
       if mode in ('control','AB') else RESULTS/'accepted_ablation'/mode/mid)
  cmd=['taskset','-c','12',sys.executable,str(ROOT/'tools/run_task_progress.py'),
       '--map-id',mid,'--output',str(out),'--workload','fixed','--optimization-mode',mode,
       '--arm','control' if mode=='control' else 'candidate','--repeats','3' if mode in ('control','AB') else '1']
  print('BATCH',mode,mid,flush=True);subprocess.run(cmd,env=env,cwd=ROOT,check=True)
