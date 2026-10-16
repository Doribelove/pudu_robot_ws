"""Save reviewed R2 evidence without modifying any parent or production file."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 d=json.loads((ROOT/'report/COMPARISON.json').read_text());candidate=d['candidate']
 selected=d['selected_r2_mode']
 assert selected==(candidate if d['cold_benefit_verified'] else 'control')
 assert 'passed' in d['verification']['tests'] and 'failed' not in d['verification']['tests']
 assert all(x['arms'][candidate]['all']['success']==30 for x in d['fixed'].values())
 assert all(x['arms'][candidate]['moving']['success']==30 for maps in d['regression'].values() for x in maps.values())
 for x in d['fixed'].values():
  a,b=x['arms']['control']['all']['metrics'],x['arms'][candidate]['all']['metrics']
  assert b['tree_rss_gib']['median']<=a['tree_rss_gib']['median']*1.05
 base={'version':'3YD-V9-r2','workspace':str(ROOT),'parent':'3YD-V9-r1 AS','date':'2026-10-09',
   'status':'reviewed_static_candidate' if selected!='control' else 'reviewed_experiments_parent_configuration_retained','selected_mode':'AS'+('+'+selected if selected!='control' else ''),'tested_r2_mode':candidate,'v9_mode':'A','review_mode':'S','r2_mode':selected,
   'L1_frozen':True,'historical_production_entry_unchanged':True,'moving_window':False,'global_L3_free_search':False,
   'fixed_strict_success':[60,60],'moving_strict_success':[60,60],'perturbed_strict_success':[60,60],
   'event_assertions':44,'tests':d['verification']['tests'],'cleanup_batches':len(d['verification']['cleanup']),
   'report':str(ROOT/'report/RESULTS.md'),'formal_data':d['data_selection']['formal'],
   'cold_benefit_verified':d['cold_benefit_verified'],'limits':['two-map static replay only','Q09 front parking obstacle still fails and withdraws trajectory',
      'complete maps remain resident','cold samples only three independent processes per arm/map','no continuous or real-vehicle acceptance']}
 save(ROOT/'BASELINE.json',base)
 save(ROOT/'report/SELECTED_DATA.json',{'formal':d['data_selection']['formal'],'excluded_from_performance':['all firstplan_diagnostic batches','all offline joint probes','gate_init_diagnostic_v1'],
     'shared':'separate v2 related-task diagnostics, n=3; not fixed10','cold':'three independent processes per map/arm; not fixed query median'})
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file() and not any(v in p.parts for v in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json')}
 save(ROOT/'RELEASE.json',{**base,'files':files});assert all(sha(ROOT/f)==h for f,h in files.items())
 verified={'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')};save(ROOT/'VERIFY.json',verified);print(verified)
if __name__=='__main__':main()
