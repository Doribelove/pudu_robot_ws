from pathlib import Path
import argparse,json,subprocess,time,hashlib
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent'])
a=argparse.ArgumentParser();a.add_argument('--root',type=Path,required=True);a.add_argument('--round',type=int,choices=[1,2,3],required=True);args=a.parse_args();root=args.root.resolve();root.mkdir(parents=True,exist_ok=True)
core=[*E.glob('src/**/*.py'),*E.glob('native/*'),E/'config/l3.json',E/'run.sh',E/'env.bash',E/'tools/run_crossmap_v10.py',E/'tools/crossmap_inputs.py'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();locked={str(p):sha(p) for p in core if p.is_file()}
if (root/'source.json').exists():assert locked==json.loads((root/'source.json').read_text()),'SOURCE_CHANGED'
else:(root/'source.json').write_text(json.dumps(locked,indent=2))
maps=json.loads((E/'MAPS.json').read_text())['maps'];status=json.loads((root/'status.json').read_text()) if (root/'status.json').exists() else [];rnd=args.round;order=maps if rnd!=2 else list(reversed(maps))
for index,m in enumerate(order):
 mid=m['map_id'];arms=['r1','r2'] if (rnd+index)%2 else ['r2','r1']
 for arm in arms:
  assert all(sha(Path(p))==h for p,h in locked.items())
  base=P if arm=='r1' else E;dest=root/f'round{rnd}'/mid/arm;dest.parent.mkdir(parents=True,exist_ok=True)
  if dest.exists():
   assert (dest/'summary.json').exists();sup=json.loads(dest.with_suffix('.supervisor.json').read_text());assert sup['returncode']==0 and sup['cleanup']['complete'];continue
  tick=time.time();print('START',rnd,mid,arm,flush=True)
  with dest.with_suffix('.log').open('w') as f:proc=subprocess.run([str(base/'run.sh'),'--map-id',mid,'--repeats','1','--cold-memory','--output',str(dest),'--domain','218'],cwd=base,stdout=f,stderr=subprocess.STDOUT)
  summary=json.loads((dest/'summary.json').read_text()) if (dest/'summary.json').exists() else {};sup=json.loads(dest.with_suffix('.supervisor.json').read_text());record={'round':rnd,'map_id':mid,'arm':arm,'elapsed_s':time.time()-tick,'returncode':proc.returncode,'cleanup_complete':sup.get('cleanup',{}).get('complete',False),'summary':summary,'path':str(dest)};status.append(record);(root/'status.json').write_text(json.dumps(status,ensure_ascii=False,indent=2));print('DONE',json.dumps(record,ensure_ascii=False),flush=True)
  if proc.returncode or summary.get('runs')!=10 or not record['cleanup_complete']:raise RuntimeError('BATCH_ERROR')
  if arm=='r2':
   old=[json.loads(x) for x in (P/'results/paired_final/round1'/mid/'r1/runs.jsonl').read_text().splitlines()];new=[json.loads(x) for x in (dest/'runs.jsonl').read_text().splitlines()];assert [r['query_id'] for r in old]==[r['query_id'] for r in new];assert not [x['query_id'] for x,y in zip(old,new) if x['success'] and not y['success']],'HISTORICAL_SUCCESS_REGRESSION'
print('ROUND_COMPLETE',rnd,flush=True)
