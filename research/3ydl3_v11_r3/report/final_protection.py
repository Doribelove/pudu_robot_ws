"""Run only after measurements: verify immutable source, inputs and resources."""
from pathlib import Path
import hashlib,json,psutil
R=Path(__file__).resolve().parents[1]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def verify(path):
 data=read(path);bad=[p for p,h in data.items() if not Path(p).is_file() or sha(Path(p))!=h]
 assert not bad,bad
 return {'lock':str(path),'files':len(data),'unchanged':True}
def main():
 assert read(R/'results/paired_final/COMPLETE.json')['complete']
 locks={'measured_candidate':verify(R/'results/paired_final/source.json'),
        'checks_candidate':verify(R/'results/checks_final/source.json')}
 for p in [R/'PARENT_PROTECTION.json',*sorted(R.glob('INTERMEDIATE*PROTECTION.json'))]:locks[p.stem]=verify(p)
 frozen=read(R/'L1_LOCK.json')['source_files'];assert all(sha(R/p)==h for p,h in frozen.items())
 locks['frozen_L1']={'files':len(frozen),'unchanged':True}
 parent=Path('/home/robot/workspaces/3ydl3_v11_r2_tailperf_20261016')
 immutable={}
 for name in ['config','native','install']:
  files=[p for p in (R/name).rglob('*') if p.is_file() and '__pycache__' not in str(p)]
  assert all(sha(p)==sha(parent/p.relative_to(R)) for p in files)
  immutable[name]={'files':len(files),'identical_to_parent':True}
 journals=[];live=[]
 for p in sorted((R/'results').rglob('*.supervisor.json')):
  j=read(p);assert j['cleanup']['complete'] and j['state']=='complete',str(p)
  for proc in j.get('processes',{}).values():
   try:
    current=psutil.Process(proc['pid'])
    if abs(current.create_time()-proc['created'])<.01 and current.is_running():live.append({'journal':str(p),'pid':proc['pid']})
   except psutil.NoSuchProcess:pass
  journals.append({'path':str(p.relative_to(R)),'returncode':j['returncode'],'cleanup_complete':True})
 assert not live,live
 formal=[j for j in journals if j['path'].startswith('results/paired_final/')]
 assert len(formal)==60 and all(j['returncode']==0 for j in formal)
 result={'version':'3YDL3-V11-r3','locks':locks,'immutable_parent_components':immutable,'owned_batches':len(journals),'formal_batches':len(formal),'all_owned_cleanup_complete':True,'owned_worker_processes_remaining':live,'nonzero_exit_diagnostics':[j for j in journals if j['returncode']!=0],'journals':journals}
 (R/'report/FINAL_PROTECTION.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k!='journals'},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
