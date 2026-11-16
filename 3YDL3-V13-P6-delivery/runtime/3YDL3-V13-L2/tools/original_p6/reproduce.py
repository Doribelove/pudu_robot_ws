"""Replay one registered case into a new directory; never overwrite evidence."""
from pathlib import Path
import argparse,hashlib,json,os,shlex,subprocess,sys
R=Path(__file__).resolve().parents[1];P5=Path('/home/robot/workspaces/3ydl3_v13_p5_fivemap_20261028')
a=argparse.ArgumentParser();a.add_argument('--output',required=True,type=Path);a.add_argument('--case',default='arena_short');a.add_argument('--arm',choices=['H-A','H-I'],default='H-I');a.add_argument('--version',choices=['baseline','tube','local'],default='local');a.add_argument('--xlarge',action='store_true');args=a.parse_args()
assert not args.output.exists(),'Use a new output directory'
cases={x['case']:x for x in json.loads((R/'docs/PROTOCOL.json').read_text())['cases']};c=cases[args.case]
for name in ['FROZEN_SOURCE_LOCK','CANDIDATE_SOURCE_LOCK','BASELINE_HARNESS_LOCK']:
    for p,h in json.loads((R/'docs'/(name+'.json')).read_text()).items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
baseline=args.version=='baseline';home=P5 if baseline else R;pilot=R/'tools/baseline_pilot.py' if baseline else R/'tools/pilot.py'
cmd=[sys.executable,str(pilot),'--map-id',c['map_id'],'--query',c['query'],'--arm',args.arm,'--events',str(R/'registered_events'/(args.case+'.json')),'--fixed-start','--output',str(args.output.resolve()),'--domain','224','--xlarge' if args.xlarge else '--four-sizes']
raise SystemExit(subprocess.call(['bash','-c','source '+shlex.quote(str(home/'env.bash'))+'\n'+shlex.join(cmd)],env=dict(os.environ,V13_P6_HINT_MODE='off' if baseline else args.version,V13_FORMAL='1',V13_CAPTURE_TRACE='0',PYTHONDONTWRITEBYTECODE='1')))
