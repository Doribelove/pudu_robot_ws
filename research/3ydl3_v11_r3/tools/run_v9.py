"""V9 independent entry, complete task planning with frozen r11 AB."""
import argparse,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--mode',choices=['semantic','free-space']);p.add_argument('--map-id',required=True)
 p.add_argument('--output',required=True);p.add_argument('--repeats',type=int,default=3)
 a,rest=p.parse_known_args()
 cmd=[sys.executable,str(ROOT/'tools/run_task_progress.py'),'--map-id',a.map_id,'--output',a.output,
      '--workload','fixed','--repeats',str(a.repeats),'--optimization-mode','AB',*rest]
 os.execv(sys.executable,cmd)
if __name__=='__main__':main()
