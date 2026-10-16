"""Snapshot only V9-r1 artifacts after all verification; no external writes."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):Path(p).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 d=json.loads((ROOT/'report/COMPARISON.json').read_text());candidate=d['candidate']
 assert all(x['arms'][candidate]['all']['success']==30 for x in d['fixed'].values())
 assert all(x['arms'][candidate]['moving']['success']==30 for maps in d['regression'].values() for x in maps.values())
 for x in d['fixed'].values():
  a,b=x['arms']['control']['all']['metrics'],x['arms'][candidate]['all']['metrics']
  assert b['tree_rss_gib']['median']<a['tree_rss_gib']['median']
  a,b=x['arms']['control']['repeat']['metrics'],x['arms'][candidate]['repeat']['metrics']
  assert b['e2e_s']['median']<a['e2e_s']['median']
 base={'version':'3YD-V9-r1','workspace':str(ROOT),'parent':'3YD-V9-r0','date':'2026-10-09',
       'status':'reviewed_static_candidate','selected_mode':candidate,'v9_mode':'A','review_mode':'S' if candidate=='AS' else 'SK',
       'L1_frozen':True,'historical_production_entry_unchanged':True,'moving_window':False,'global_L3_free_search':False,
       'fixed_strict_success':[60,60],'moving_strict_success':[60,60],'perturbed_strict_success':[60,60],
       'event_assertions':44,'tests':d['verification']['tests'],'cleanup_batches':len(d['verification']['cleanup']),
       'report':str(ROOT/'report/RESULTS.md'),'formal_data':d['data_selection']['formal'],
       'limits':['two-map static replay only','Q09 front parking obstacle still fails and withdraws trajectory',
                 'complete maps remain resident','original audit distance construction peak not removed','no continuous or real-vehicle acceptance']}
 save(ROOT/'BASELINE.json',base)
 save(ROOT/'report/SELECTED_DATA.json',{'formal':d['data_selection']['formal'],
      'diagnostic':['results/audit_profile_v1','results/storage_diagnostic_v1','results/review_diagnostic_v2'],
      'selection':'formal source-locked control/S/AS/ASK fixed data; selected candidate regressions; shared diagnostics are separate',
      'excluded_from_performance':['all profile/diagnostic batches'],
      'shared_queries':'datasets/v9_shared_segments_v2'})
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file()
       and not any(v in p.parts for v in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc'
       and p.name not in ('RELEASE.json','VERIFY.json')}
 save(ROOT/'RELEASE.json',{**base,'files':files})
 assert all(sha(ROOT/f)==h for f,h in files.items())
 verify={'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')}
 save(ROOT/'VERIFY.json',verify);print(verify)
if __name__=='__main__':main()
