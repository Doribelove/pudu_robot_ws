"""Save V9-only artifacts. Never writes V8 or production pointers."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,d):Path(p).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 d=json.loads((ROOT/'report/COMPARISON.json').read_text())
 assert all(x['arms']['A']['all']['success']==30 for x in d['fixed'].values())
 assert all(x['arms']['A']['moving']['success']==30 for xs in d['regression'].values() for x in xs.values())
 base={'version':'3YD-V9-r0','workspace':str(ROOT),'parent':'3YD-V8-r11 AB frozen',
  'date':'2026-10-09','status':'experimental_not_adopted_memory_goal_not_met','selected_mode':'control','tested_latency_mode':'A','memory_goal_achieved':False,
  'experimental_modes':['A','B','C','AB','AC','BC','ABC'],'L1_frozen':True,'moving_window':False,
  'global_L3_fallback':False,'fixed_strict_success':[60,60],'moving_strict_success':[60,60],
  'perturbed_strict_success':[60,60],'report':str(ROOT/'report/RESULTS.md'),
  'verification':{k:v for k,v in d['verification'].items() if k not in ('cleanup','source_snapshots')},
  'cleanup_batches':len(d['verification']['cleanup']),
  'limits':['only two maps this round','Nav2/Python whole-map storage retained','Q09 future parking obstacle fails safely','no live execution acceptance'],
  'not_adopted':['A initial duplicate-buffer ownership','A as default: RSS median rose despite latency improvement','B/C as defaults due bounded-connection overhead on difficult routes']}
 write(ROOT/'BASELINE.json',base)
 write(ROOT/'report/SELECTED_DATA.json',{'formal':d['data_selection']['formal'],
  'excluded':['results/diagnostic_v1','results/diagnostic_v2','results/diagnostic_v3','results/accepted_v9','results/accepted_v9_final','results/accepted_v9_final2','results/accepted_v9_final3/shared'],
  'reasons':{'diagnostic_v1/v2/v3':'different planning source iterations; diagnostic subset only',
   'accepted_v9':'A retained old buffers alongside immutable copies, increasing memory',
   'accepted_v9_final':'strong target-mask cache; replaced by weak ownership',
   'accepted_v9_final2':'control hash implementation corrected after this family; final control uses original parent digest paths',
   'accepted_v9_final3/shared':'SHARED-02 and SHARED-03 accidentally had identical endpoints; whole-route hits occurred. Replaced with verified distinct v2 inputs and all eight shared arms rerun.'},
  'shared_queries':d['data_selection']['shared_queries'],
  'selection':'All fixed/control and repeated latency records retained from final source. B/C/ABC independent experiments; none adopted as default.'})
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file() and not any(v in p.parts for v in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json')}
 write(ROOT/'RELEASE.json',{**base,'files':files})
 assert all(sha(ROOT/f)==h for f,h in files.items())
 verify={'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')};write(ROOT/'VERIFY.json',verify);print(verify)
if __name__=='__main__':main()
