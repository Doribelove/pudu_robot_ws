"""Reviewed V9 local entry; explicit CLI modes override the saved choice."""
import json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'BASELINE.json'
args=sys.argv[1:]
if path.exists():
 chosen=json.loads(path.read_text())
 if not any(v=='--v9-mode' or v.startswith('--v9-mode=') for v in args):
  args=['--v9-mode',chosen['v9_mode'],*args]
 if not any(v=='--review-mode' or v.startswith('--review-mode=') for v in args):
  args=['--review-mode',chosen['review_mode'],*args]
 if 'r2_mode' in chosen and not any(v=='--r2-mode' or v.startswith('--r2-mode=') for v in args):
  args=['--r2-mode',chosen['r2_mode'],*args]
 for key in ('r3_mode','r5_mode','r6_mode','r7_mode','r8_mode','r9_mode'):
  flag='--'+key.replace('_','-')
  if key in chosen and not any(v==flag or v.startswith(flag+'=') for v in args):args=[flag,chosen[key],*args]
if '--fixed-default' in args:
 args.remove('--fixed-default')
 if '--workload' not in args:args=['--workload','fixed',*args]
os.execv(sys.executable,[sys.executable,str(ROOT/'tools/run_task_progress.py'),*args])
