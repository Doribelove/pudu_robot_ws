"""Seal reviewed R3 evidence; parent releases and production remain untouched."""
import json
from pathlib import Path
from compare_v9_review import load,sha,save
ROOT=Path(__file__).resolve().parents[1]
def main():
 d=load(ROOT/'report/COMPARISON.json');s=d['selected_r3_mode'];v=d['verification']
 assert s==('I' if (d['first_benefit_verified'] or d['cold_benefit_verified']) and d['repeat_and_memory_gate'] else 'control')
 assert 'passed' in v['tests'] and 'failed' not in v['tests']
 assert all(x['arms']['I']['all']['success']==30 for x in d['fixed'].values())
 assert all(x['arms']['I']['moving']['success']==30 for maps in d['regression'].values() for x in maps.values())
 # The existing selector does not inject R3. An adopted I needs a separately
 # reviewed selector/source cycle before any default change can be recorded.
 assert s=='control','I adoption requires an audited entry-routing cycle'
 b={'version':'3YD-V9-r3','workspace':str(ROOT),'parent':'3YD-V9-r2 AS','date':'2026-10-09',
    'status':'reviewed_experiments_parent_configuration_retained','selected_mode':'AS','v9_mode':'A','review_mode':'S','r2_mode':'control','r3_mode':s,
    'tested_r3_mode':'I','L1_frozen':True,'historical_production_entry_unchanged':True,'moving_window':False,'global_L3_free_search':False,
    'fixed_strict_success':[60,60],'moving_strict_success':[60,60],'perturbed_strict_success':[60,60],
    'event_assertions':44,'tests':v['tests'],'cleanup_batches':len(v['cleanup']),
    'report':str(ROOT/'report/RESULTS.md'),'formal_data':d['data_selection']['formal'],
    'first_benefit_verified':d['first_benefit_verified'],'cold_benefit_verified':d['cold_benefit_verified'],
    'limits':['two-map static replay only','Q09 front parking obstacle withdraws trajectory until clear','complete maps remain resident','n=3 independent cold processes per map/arm','no continuous or real-vehicle acceptance']}
 save(ROOT/'BASELINE.json',b)
 save(ROOT/'report/SELECTED_DATA.json',{'formal':d['data_selection']['formal'],'excluded_from_performance':['startup_delta_diagnostic_v1: wrong MRO, I inactive','startup_delta_diagnostic_v2: three-query diagnostic','default_entry_check'],'cold':'n=3 independent processes per map/arm, shown as points'})
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json')}
 save(ROOT/'RELEASE.json',{**b,'files':files});assert all(sha(ROOT/f)==h for f,h in files.items())
 result={'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')};save(ROOT/'VERIFY.json',result);print(result)
if __name__=='__main__':main()
