"""Report every predeclared final batch plus retained development failures."""
import json,hashlib,os,math,xml.etree.ElementTree as ET
from pathlib import Path
from collections import Counter
from types import SimpleNamespace
import numpy as np,yaml
from summarize_turnaround import summarize,quant,read
from benchmark import arc_metrics
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text())
def summarize_extra(rows,roots):
 d=summarize(rows)
 d['stage_ms']={k:quant([r.get('l3',{}).get(k) for r in rows]) for k in ['local_map_update_ms','local_map_publication_ms','costmap_ack_wait_ms','planning_time_ms']}
 d['reference_adapter_ms']=quant([r.get('reference',{}).get('adapter_ms') for r in rows])
 d['gap_repairs']=sum(r.get('l3',{}).get('semantic_gap_repair_count',0) for r in rows)
 d['exact_ack_all_accepted']=all(r['l3'].get('costmap_update_acknowledged') and r['l3'].get('costmap_ack_mismatch_cells')==0 for r in rows if r['success'])
 journals=[load(p.with_name(p.name+'.supervisor.json')) for p in roots]
 d['supervisor_peak_rss_mib']=max(j.get('supervisor_peak_rss_bytes',0) for j in journals)/2**20
 d['cleanup_complete']=all(j['state']=='complete' and j['cleanup']['complete'] for j in journals)
 return d

def main():
 os.sched_setaffinity(0,{12})
 roots=[WORK/'results'/n for n in ['final_1_baseline','final_2_combined','final_3_combined','final_4_baseline']]
 data={p.name:read(p) for p in roots};arms={'baseline':[roots[0],roots[3]],'candidate':[roots[1],roots[2]]}
 out={'protocol':'A/B/B/A; 20 fixed queries x 3 repeats per batch; two independent process/cache batches per arm; every row included','main':{},'batches':{},'controls':{},'stress':{},'faults':{}}
 for p in roots:out['batches'][p.name]=summarize_extra(data[p.name],[p])
 grouped={a:[dict(r,batch=p.name) for p in ps for r in data[p.name]] for a,ps in arms.items()}
 for a,rows in grouped.items():out['main'][a]=summarize_extra(rows,arms[a])
 out['old_previous_task_final']=load('/home/robot/workspaces/3yd_v1_turnaround_ack_20260914/report/performance.json')['main']['candidate']
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache/report_world')
 fields={};quality_cache={};quality=[];paired=[]
 base=Path('/home/robot/workspaces/3yd_multires_static_20260914/results/formal_01_baseline/cases')
 for a,rows in grouped.items():
  for r in rows:
   if not r['success']:continue
   q=r['query_id'];key=(q,r['pose_hash'])
   if key not in quality_cache:
    if q not in fields:
     with np.load(base/f'{q}__1__right_0/evaluation_field.npz') as f:fields[q]=SimpleNamespace(normalized_right=f['q'],eligible=f['eligible'])
    quality_cache[key]=arc_metrics(w,np.load(Path(r['case'])/'path.npy'),fields[q])
   r['quality']=quality_cache[key];quality.append({'arm':a,'batch':r['batch'],'query':q,'repetition':r['repetition'],**r['quality']})
  out['main'][a]['right_band_ratio']=quant([r.get('quality',{}).get('right_band_ratio') for r in rows])
 for q in dict.fromkeys(r['query_id'] for r in grouped['baseline']):
  rr={a:[r for r in rows if r['query_id']==q] for a,rows in grouped.items()};item={'query':q}
  for a,rows in rr.items():
   item[a]={'success':sum(r['success'] for r in rows),'n':len(rows),'length_m':quant([r.get('length_m') for r in rows])['p50'],'right_band_ratio':quant([r.get('quality',{}).get('right_band_ratio') for r in rows])['p50'],'completion_modes':dict(Counter(r.get('multires',{}).get('completion_mode','coarse_only') for r in rows))}
  first=next((r for r in rr['baseline'] if r['success']),None)
  item['all_paths_identical']=bool(first and all(r['success'] and r['pose_hash']==first['pose_hash'] and np.array_equal(np.load(Path(r['case'])/'path.npy'),np.load(Path(first['case'])/'path.npy')) for rows in rr.values() for r in rows))
  paired.append(item)
 out['paired_queries']=paired;out['identical_query_paths']=sum(p['all_paths_identical'] for p in paired)
 for p in (WORK/'results').glob('final_control_*'):
  if not p.is_dir():continue
  rows=read(p);d=summarize_extra(rows,[p]);scene=p.name.removeprefix('final_control_')
  if scene in ['calibration','rotated']:
   d['right_left_right_pose_restored']=len(rows)==9 and all(rows[i].get('pose_hash')==rows[i+2].get('pose_hash') for i in range(0,len(rows),3))
   bands=[]
   for r in rows:
    if not r['success']:continue
    pts=np.load(Path(r['case'])/'path.npy');ds=np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1);s=np.r_[0,np.cumsum(ds)];bounds=np.r_[np.arange(0,s[-1],.025),s[-1]];mid=(bounds[:-1]+bounds[1:])/2;weights=np.diff(bounds);xy=np.column_stack([np.interp(mid,s,pts[:,i]) for i in (0,1)])
    right=(xy[:,1]-.5)/5 if scene=='calibration' else (7.5-xy[:,0])/7;sel=(mid>=5)&(mid<=s[-1]-5);lo,hi=(.125,.375) if r['side']=='right' else (.625,.875);bands.append(float(weights[sel&(right>=lo)&(right<=hi)].sum()/weights[sel].sum()))
   d['minimum_target_band_ratio']=min(bands) if bands else None
  out['controls'][scene]=d
 for name in ['final_stress_calibration','final_stress_rotated','stress_locked_real']:
  p=WORK/'results'/name;rows=read(p,'updates.jsonl');d=load(p/'summary.json');d.update(elapsed_s=quant([r['elapsed_s'] for r in rows]),full_fallbacks=sum(bool(r.get('ack',{}).get('local_map_update_fallback')) for r in rows),gap_repairs=sum(r.get('ack',{}).get('semantic_gap_repair_count',0) for r in rows));out['stress'][name]=d
 for p in (WORK/'results').glob('final_fault_*'):
  if not p.is_dir():continue
  out['faults'][p.name]={'summary':load(p/'summary.json'),'updates':[{'index':r['index'],'success':r['success'],'elapsed_s':r['elapsed_s'],'injected_tiles':r['injected_tiles'],'gap_repairs':r.get('ack',{}).get('semantic_gap_repair_count'),'full_fallback':r.get('ack',{}).get('local_map_update_fallback')} for r in read(p,'updates.jsonl')]}
 out['lifecycle']=load(WORK/'results/final_resource_cycles/summary.json')
 out['development']={p.name:summarize(read(p)) for p in (WORK/'results').iterdir() if (p/'runs.jsonl').exists() and p.name not in data and not p.name.startswith('final_control_')}
 for summary in out['development'].values():
  summary['ack_statuses']={('not_recorded' if k is None else k):v for k,v in summary['ack_statuses'].items()}
 out['development_semantic']={p.name:load(p/'summary.json') for p in (WORK/'results').iterdir() if (p/'updates.jsonl').exists() and not p.name.startswith('final_') and p.name!='stress_locked_real'}
 out['fixture_diagnosis']=load(WORK/'report/stress_fixture_diagnosis.json')
 freeze=load(WORK/'report/FINAL_IMPLEMENTATION_FREEZE.json');base=load(WORK/'BASELINE_MANIFEST.json');br=Path(base['root']);native=load(WORK/'report/native_snapshot/local_provenance.json');nr=Path(native['source'])
 libs={p.name:load(p/'loaded_nav2_libraries.json') for p in roots};smac={h for d in libs.values() for proc in d['processes'] for p,h in proc['libraries'].items() if p.endswith('/libnav2_smac_planner.so')}
 out['integrity']={'frozen_files':len(freeze),'changed_frozen_files':[p for p,h in freeze.items() if sha(WORK/p)!=h],'baseline_files':len(base['files']),'baseline_changed':[p for p,h in base['files'].items() if sha(br/p)!=h],'native_source_changed_in_original':[p for p,h in native['original_file_hashes'].items() if sha(nr/p)!=h],'native_copy_changes':[p for p,h in native['original_file_hashes'].items() if sha(WORK/'external/nav2_snapshot_local/nav2_costmap_2d'/p)!=h],'parameters_identical':len({sha(p/'smac/logs/smac_strict_real.yaml') for p in roots})==1,'smac_plugin_hashes':sorted(smac),'same_smac_plugin':len(smac)==1,'loaded_libraries':libs,'same_vehicle_sweep':sha(WORK/'build/libsemantic_sweep.so')==sha(br/'build/libsemantic_sweep.so')}
 tree=ET.parse(WORK/'report/final_regression.xml').getroot();out['regression']=[s.attrib for s in tree.iter('testsuite')]
 out['dispatch']=read(WORK/'report','final_dispatch.jsonl')
 json_write(WORK/'report/performance.json',out);json_write(WORK/'report/path_quality.json',quality)
 print(json.dumps({'main':out['main'],'batches':{k:v['timing_s'] for k,v in out['batches'].items()},'identical_paths':out['identical_query_paths'],'controls':out['controls'],'integrity':{k:v for k,v in out['integrity'].items() if k!='loaded_libraries'}},indent=2))
if __name__=='__main__':main()
