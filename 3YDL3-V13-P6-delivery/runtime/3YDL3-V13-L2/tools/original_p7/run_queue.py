from pathlib import Path
import json,hashlib,os,sys,subprocess,shlex
R=Path(__file__).resolve().parents[1];P=Path('/home/robot/workspaces/3ydl3_v13_p6_thunder_20261028')
name=sys.argv[1];jobs=json.loads((R/'docs'/name).read_text());status=R/'report'/(Path(name).stem+'_STATUS.json');assert not status.exists()
locks=json.loads((R/'docs/CANDIDATE_SOURCE_LOCK.json').read_text())
def verify():
 for f,h in locks.items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h,f
def save(d):status.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
done=[]
for j in jobs:
 verify();out=R/'results'/j['phase']/j['name'];out.parent.mkdir(parents=True,exist_ok=True);assert not out.exists()
 root=P if j.get('baseline') else R
 cmd=[sys.executable,str(root/'tools/pilot.py'),'--map-id',j['map_id'],'--query',j['query'],'--arm',j['arm'],'--output',str(out),'--events',str(R/'registered_events'/(j['case']+'.json')),'--domain','225','--thunder']
 if not j.get('moving'):cmd+=['--fixed-start']
 else:cmd+=['--offset',str(j['offset'])]
 if 'size' in j:cmd+=['--obstacle-size',str(j['size'])]
 save(dict(status='running',current=j,done=done,total=len(jobs)));print('START',len(done)+1,'/',len(jobs),j['name'],flush=True)
 env=dict(os.environ,V13_FORMAL='0' if j['phase']=='diagnostic' else '1',V13_CAPTURE_TRACE='1' if j['phase']=='diagnostic' else '0',V13_VERIFY_EXTRACTION='0')
 with (R/'report'/(j['name']+'.log')).open('w') as f:
  rc=subprocess.run(['bash','-c','source '+shlex.quote(str(root/'env.bash'))+'\n'+shlex.join(cmd)],env=env,cwd=root,stdout=f,stderr=subprocess.STDOUT).returncode
 if rc:save(dict(status='failed_harness',current=j,done=done,rc=rc));raise RuntimeError((j,rc))
 summary=json.loads((out/'summary.json').read_text());assert summary['strict_recheck_passed'] and summary['all_failures_withdraw_trajectory']
 assert json.loads(Path(str(out)+'.supervisor.json').read_text())['cleanup']['complete']
 done.append(j['name']);print('DONE',j['name'],[(x['label'],x['success'],round(x['wall_ms']/1000,3),x.get('failure_code')) for x in summary['rows'] if x['label']=='thunder_add'],flush=True)
save(dict(status='completed',done=done,total=len(jobs)))
