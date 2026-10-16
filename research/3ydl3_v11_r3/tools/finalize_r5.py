"""Record verified r5 candidate; does not promote any production entry."""
import json,hashlib,psutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r4_20261008')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
c=json.loads((ROOT/'report/COMPARISON.json').read_text())
b=json.loads((PARENT/'BASELINE.json').read_text());b.update(version='3YD-V8-r5',parent='3YD-V8-r4',parent_workspace=str(PARENT),scope='r4 ABC plus steering-only repair trigger normalization',selected_mode='ABC + repair1',entry_default='run.sh ABC + repair1; direct module import conservative',test_count=141,additional_strict_success=[18,20],cross_map_distinct_queries_passed=[38,40])
assert sum(v['r5']['success'] for v in c['maps'].values())==78
write(ROOT/'BASELINE.json',b)
records=[]
for path in sorted((ROOT/'results').rglob('*.supervisor.json')):
 j=json.loads(path.read_text());live=[];resources=[]
 for proc in [*j.get('processes',{}).values(),j['supervisor']]:
  try:
   v=psutil.Process(proc['pid'])
   if abs(v.create_time()-proc['created'])<.05:live.append(proc['pid'])
  except psutil.NoSuchProcess:pass
 for name,identity in j.get('resources',{}).items():
  f=Path('/dev/shm')/name
  try:
   s=f.stat()
   if [s.st_dev,s.st_ino,s.st_uid]==identity:resources.append(name)
  except FileNotFoundError:pass
 assert not live and not resources,(str(path),live,resources)
 assert j['state']=='complete' and j['cleanup']['complete'],path
 records.append({'journal':str(path.relative_to(ROOT)),'live_owned_processes':live,'remaining_owned_resources':resources,'cleanup_complete':True})
write(ROOT/'report/CLEANUP_VERIFIED.json',{'complete':True,'journals':records})
parent=json.loads((PARENT/'RELEASE.json').read_text());assert all(sha(PARENT/f)==h for f,h in parent['files'].items())
write(ROOT/'report/PARENT_VERIFIED.json',{'parent':str(PARENT),'files_verified':len(parent['files']),'unchanged':True})
files={}
for directory in ('src','native','inputs','config','tests','tools','results','report'):
 for p in sorted((ROOT/directory).rglob('*')):
  if not p.is_file() or '__pycache__' in p.parts or '.pytest_cache' in p.parts or p.suffix=='.pyc':continue
  files[str(p.relative_to(ROOT))]=sha(p)
for p in sorted(ROOT.iterdir()):
 if p.is_file() and p.suffix in ('.md','.json','.sh','.bash') and p.name not in ('RELEASE.json','VERIFY.json'):files[p.name]=sha(p)
write(ROOT/'RELEASE.json',{'version':'3YD-V8-r5','selected_mode':'ABC + repair1','status':'static_tested_candidate_not_production','parent':'3YD-V8-r4','primary_strict_success':[60,60],'additional_strict_success':[18,20],'test_count':141,'L1_frozen':True,'files':files,'verification':{'original_successful_paths_equal':[76,76],'parent_files_verified':len(parent['files']),'cleanup_journals_verified':len(records)}})
assert all(sha(ROOT/f)==h for f,h in files.items())
write(ROOT/'VERIFY.json',{'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')})
print('Release verified',len(files),'files;',len(records),'clean supervisors')
