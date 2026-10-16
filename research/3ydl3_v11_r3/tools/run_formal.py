from pathlib import Path
import json,subprocess,time,sys,hashlib,argparse
E=Path(__file__).resolve().parents[1]
a=argparse.ArgumentParser();a.add_argument('--output-root',type=Path,default=E/'results/formal');args=a.parse_args()
output_root=args.output_root.resolve()
status_file=E/'STATUS.json' if output_root==E/'results/formal' else output_root.with_suffix('.status.json')
maps=json.loads((E/'MAPS.json').read_text())['maps']
status=[]
for rnd in range(1,4):
    order=maps if rnd!=2 else list(reversed(maps))
    for m in order:
        dest=output_root/f'round{rnd}'/m['map_id'];dest.parent.mkdir(parents=True,exist_ok=True)
        assert not dest.exists(),str(dest)
        log=dest.with_suffix('.log');started=time.time()
        print('START',rnd,m['map_id'],flush=True)
        with log.open('w') as f:
            p=subprocess.run([str(E/'run.sh'),'--map-id',m['map_id'],'--repeats','1','--output',str(dest),'--domain','218'],stdout=f,stderr=subprocess.STDOUT,cwd=E)
        journal=dest.parent/(dest.name+'.supervisor.json')
        record=json.loads(journal.read_text()) if journal.exists() else {}
        summary=json.loads((dest/'summary.json').read_text()) if (dest/'summary.json').exists() else {}
        item={'round':rnd,'map_id':m['map_id'],'returncode':p.returncode,'elapsed_s':time.time()-started,'summary':summary,'cleanup_complete':record.get('cleanup',{}).get('complete',False),'path':str(dest)}
        status.append(item);status_file.write_text(json.dumps(status,ensure_ascii=False,indent=2))
        print('DONE',json.dumps(item,ensure_ascii=False),flush=True)
        if p.returncode!=0 or summary.get('runs')!=10 or not item['cleanup_complete']:
            print('BATCH_ERROR: inspect log, keep original output',flush=True);sys.exit(1)
print('COMPLETE_300_REQUESTS',flush=True)
