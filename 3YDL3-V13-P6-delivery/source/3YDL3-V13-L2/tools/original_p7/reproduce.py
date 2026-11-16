"""Fresh single-batch reproduction; old completed results are never resumed."""
from pathlib import Path
import argparse,json,os,subprocess,sys,shlex,hashlib
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--arm',choices=['H-A','H-I'],default='H-I');p.add_argument('--output',type=Path,required=True);p.add_argument('--diagnostic',action='store_true');p.add_argument('--moving-offset',type=float);args=p.parse_args()
assert not args.output.exists() and not Path(str(args.output)+'.supervisor.json').exists(),'OUTPUT_MUST_BE_FRESH'
for name in ['CANDIDATE_SOURCE_LOCK','PARENT_SOURCE_LOCK']:
 for file,digest in json.loads((R/'docs'/(name+'.json')).read_text()).items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
j=next(j for j in json.loads((R/'docs/FINAL_QUEUE.json').read_text()) if j['case']==args.case)
cmd=['/usr/bin/python3',str(R/'tools/pilot.py'),'--map-id',j['map_id'],'--query',j['query'],'--arm',args.arm,'--output',str(args.output.resolve()),'--events',str(R/'registered_events'/(args.case+'.json')),'--domain','225','--thunder']
cmd+=['--fixed-start'] if args.moving_offset is None else ['--offset',str(args.moving_offset)]
env=dict(os.environ,V13_FORMAL='0' if args.diagnostic else '1',V13_CAPTURE_TRACE='1' if args.diagnostic else '0')
subprocess.run(['bash','-c','source '+shlex.quote(str(R/'env.bash'))+'\n'+shlex.join(cmd)],cwd=R,env=env,check=True)
assert json.loads(Path(str(args.output)+'.supervisor.json').read_text())['cleanup']['complete']
