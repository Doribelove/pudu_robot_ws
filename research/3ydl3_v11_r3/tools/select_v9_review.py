"""Choose the additional K complexity only for measured two-map benefits."""
import argparse,json
from pathlib import Path
from analyze_v8 import MAPS,measure,stats
from compare_v9 import read
from compare_v9_review import exact
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);a=p.parse_args()
 data={};criteria={}
 for mid,label in MAPS:
  dirs={m:a.results_root/'fixed'/m/mid for m in ('control','S','AS','ASK')}
  for m in ('S','AS','ASK'):exact(dirs['control'],dirs[m])
  data[mid]={}
  for m,d in dirs.items():
   rows=read(d);assert len(rows)==30 and all(r['success'] for r in rows)
   data[mid][m]={part:{metric:stats([measure(r,metric) for r in rows if part=='all' or (r['repeat']==1 if part=='first' else r['repeat']>1)])
       for metric in ('e2e_s','tree_rss_gib','python_pss_gib')} for part in ('all','first','repeat')}
  old,new=data[mid]['AS'],data[mid]['ASK']
  criteria[mid]={'K_repeated_median_at_least_3pct_faster':new['repeat']['e2e_s']['median']<=.97*old['repeat']['e2e_s']['median'],
     'K_first_median_within_2pct':new['first']['e2e_s']['median']<=1.02*old['first']['e2e_s']['median'],
     'K_RSS_median_within_0point5pct':new['all']['tree_rss_gib']['median']<=1.005*old['all']['tree_rss_gib']['median']}
 selected='ASK' if all(all(v.values()) for v in criteria.values()) else 'AS'
 for mid in data:
  control,candidate=data[mid]['control'],data[mid][selected]
  assert candidate['repeat']['e2e_s']['median']<control['repeat']['e2e_s']['median']
  assert candidate['all']['tree_rss_gib']['median']<control['all']['tree_rss_gib']['median']
 result={'selected':selected,'criteria':criteria,'raw_summary':data,
         'policy':'Additional K only when both maps show >=3% repeat median benefit, <=2% first median overhead, <=0.5% RSS median overhead versus AS; static output parity required. These thresholds are engineering selection criteria, not significance tests.'}
 (ROOT/'report/SELECTION.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(selected)
if __name__=='__main__':main()
