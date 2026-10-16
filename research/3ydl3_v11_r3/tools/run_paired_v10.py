from pathlib import Path
import argparse,json,subprocess,time,hashlib
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent'])
a=argparse.ArgumentParser();a.add_argument('--root',type=Path,default=E/'results/paired');args=a.parse_args()
root=args.root.resolve();root.mkdir(parents=True,exist_ok=True)
core=[*E.glob('src/**/*.py'),*E.glob('native/*'),E/'config/l3.json',E/'run.sh',E/'env.bash',E/'tools/run_crossmap_v10.py',E/'tools/crossmap_inputs.py']
hashfile=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
locked={str(p):hashfile(p) for p in core if p.is_file()}
if (root/'source.json').exists():assert locked==json.loads((root/'source.json').read_text()),'RESUME_SOURCE_CHANGED'
else:(root/'source.json').write_text(json.dumps(locked,indent=2))
maps=json.loads((E/'MAPS.json').read_text())['maps'];status=[]
for rnd in range(1,4):
    order=maps if rnd!=2 else list(reversed(maps))
    for index,m in enumerate(order):
        mid=m['map_id'];arms=['v9','v10'] if (rnd+index)%2 else ['v10','v9']
        for arm in arms:
            assert all(hashfile(Path(p))==h for p,h in locked.items()),'SOURCE_CHANGED_DURING_EXPERIMENT'
            base=P if arm=='v9' else E;dest=root/f'round{rnd}'/mid/arm;dest.parent.mkdir(parents=True,exist_ok=True)
            if (dest/'summary.json').exists():
                assert json.loads(dest.with_suffix('.supervisor.json').read_text())['cleanup']['complete']
                previous=[v for v in json.loads((root/'status.json').read_text()) if v['round']==rnd and v['map_id']==mid and v['arm']==arm]
                status.append(previous[0] if previous else {'round':rnd,'map_id':mid,'arm':arm,'elapsed_s':None,'returncode':0,'cleanup_complete':True,'summary':json.loads((dest/'summary.json').read_text()),'path':str(dest),'orchestrator_record_recovered_from_finished_worker':True})
                continue
            assert not dest.exists(), 'INCOMPLETE_BATCH_REQUIRES_REVIEW'
            tick=time.time();print('START',rnd,mid,arm,flush=True)
            with dest.with_suffix('.log').open('w') as f:
                proc=subprocess.run([str(base/'run.sh'),'--map-id',mid,'--repeats','1','--output',str(dest),'--domain','218'],cwd=base,stdout=f,stderr=subprocess.STDOUT)
            summary=json.loads((dest/'summary.json').read_text()) if (dest/'summary.json').exists() else {}
            journal=dest.with_suffix('.supervisor.json');clean=json.loads(journal.read_text()).get('cleanup',{}).get('complete',False) if journal.exists() else False
            record={'round':rnd,'map_id':mid,'arm':arm,'elapsed_s':time.time()-tick,'returncode':proc.returncode,'cleanup_complete':clean,'summary':summary,'path':str(dest)}
            status.append(record);(root/'status.json').write_text(json.dumps(status,ensure_ascii=False,indent=2));print('DONE',json.dumps(record,ensure_ascii=False),flush=True)
            if proc.returncode or summary.get('runs')!=10 or not clean:raise RuntimeError('BATCH_ERROR')
            if arm=='v10':
                old=[json.loads(x) for x in (P/'results/formal/round1'/mid/'runs.jsonl').read_text().splitlines()]
                new=[json.loads(x) for x in (dest/'runs.jsonl').read_text().splitlines()]
                bad=[x['query_id'] for x,y in zip(old,new) if x['success'] and not y['success']]
                if bad:raise RuntimeError('V9_SUCCESS_REGRESSION:'+repr(bad))
    if (root/'STOP_AFTER_ROUND').exists():
        print('STOPPED_AT_COMPLETED_ROUND_FOR_NEXT_SOURCE_ITERATION',rnd,flush=True);break
else:print('COMPLETE_600_REQUESTS',flush=True)
