"""Replay registered batches into a new destination without altering delivery."""
import argparse,json,hashlib,os,subprocess,sys,shlex
from pathlib import Path
R=Path(__file__).resolve().parents[1];P4=Path('/home/robot/workspaces/3ydl3_v13_p4_corridor_recovery_20261027')
a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);a.add_argument('--batch',action='append');args=a.parse_args()
jobs=json.loads((R/'docs/FORMAL_QUEUE.json').read_text())
if args.batch:
    assert set(args.batch)<=set(j['name'] for j in jobs),'Unknown registered batch'
    jobs=[j for j in jobs if j['name'] in args.batch]
for lock in ['CANDIDATE_SOURCE_LOCK.json','FROZEN_SOURCE_LOCK.json']:
    for file,digest in json.loads((R/'docs'/lock).read_text()).items():assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
(args.output/'REGISTERED_BATCHES.json').write_text(json.dumps(jobs,indent=2))
for job in jobs:
    flags=[sys.executable,str(R/'tools/pilot.py'),'--map-id',job['map'],'--query',job['query'],'--arm',job['arm'],'--events',job['event'],'--offset',str(job['offset']),'--output',str(args.output/job['name']),'--domain','220']
    if job['mode']!='sequence':flags+=['--only-'+job['mode']]
    if job['fixed']:flags+=['--fixed-start']
    env=dict(os.environ,V13_P5_SCHEDULE='0' if job['version']=='baseline' else '1',V13_P5_CACHE='0' if job['version']=='no_cache' else '1',V13_FORMAL='1',V13_CAPTURE_TRACE='0',V13_P5_HEADING_REFERENCE='0')
    envfile=P4/'env_delivery.bash' if job['version']=='baseline' else R/'env.bash'
    with (args.output/(job['name']+'.log')).open('w') as f:
        subprocess.run(['bash','-c','source '+shlex.quote(str(envfile))+'\n'+shlex.join(flags)],env=env,cwd=R,stdout=f,stderr=subprocess.STDOUT,check=True)
    print('DONE',job['name'],flush=True)
