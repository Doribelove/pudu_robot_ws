"""Review full-task native storage with unchanged global input and audits.

Local server readback hashes deliberately cannot be compared to global hashes.
Geometry/path changes are reported and tested against predeclared quality gates.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np,yaml
from compare_v9 import read,tag,pair,parts,summary
from quality_v9_r6 import quality as relative_quality
from compare_v9_review import load,save,sha
from compare_v8_r6 import verify_cleanup
from analyze_v8 import MAPS,measure
ROOT=Path(__file__).resolve().parents[1]

def mode_evidence(directory,mode,protocol=True,expected_v9_mode='A',expected_r2_mode='control'):
 expected_h='H' if mode in ('CH','CHN','CHNT') else 'control'
 expected_n='NT' if mode=='CHNT' else 'N' if mode in ('N','CHN') else 'control'
 mode='C' if mode in ('CH','CHN','CHNT') else 'control' if mode=='N' else mode
 if protocol:
  p=load(directory/'protocol.json')
  assert p['version']=='3YD-V9-r7-L3' and p['r6_mode']==expected_h and p['r7_mode']==expected_n
  assert p['r5_mode']==mode and p['r2_mode']==expected_r2_mode and p['r3_mode']=='control'
  assert p['v9_mode']==expected_v9_mode and p['review_mode']=='S' and p['CPU']==[12]
  assert not p['moving_window'] and not p['global_L3_fallback']
 else:
  p=load(directory/'summary.json')
  assert p['version']=='3YD-V9-r7' and p['r6_mode']==expected_h and p['r7_mode']==expected_n and p['r5_mode']==mode and p['r3_mode']=='control' and p['CPU']==[12]
  assert p['optimization_mode']=='AB' and p['v9_mode']==expected_v9_mode and p['review_mode']=='S' and p['r2_mode']==expected_r2_mode
  assert p['static_event_replay'] and not p['live_execution']
 preparation=load(directory/'v9_preparation.json');neutral=preparation['neutral_labels']
 assert preparation['r7_mode']==expected_n
 applicable=expected_n in ('N','NT') and directory.name=='arena_large_250x300_2d_005'
 assert neutral['enabled']==applicable
 if applicable:
  assert neutral['storage_values_unchanged'] and neutral['broadcast_readonly']
  assert neutral['new_backing_buffer_bytes']==2 and neutral['logical_bytes']==60000000
 events=load(directory/'smac/lifecycle_evidence/r5_complete_geometry.json')
 if mode=='control':assert not events;return {'geometry_events':0}
 boot=directory/'smac/bootstrap/map.yaml';b=yaml.safe_load(boot.read_text())
 assert b['resolution']==.05 and Path(b['image']).name=='map.pgm'
 native_log=next((directory/'smac/logs').glob('*.log')).read_text()
 assert 'Loading yaml file: '+str(boot.resolve()) in native_log
 assert 'Read map '+b['image']+': 16 X 16 map @ 0.05 m/cell' in native_log
 pgm=Path(b['image']).read_bytes().split(b'\n',3)
 assert pgm[:3]==[b'P5',b'16 16',b'255'] and pgm[3]==bytes(256)
 # Every changed map is explicitly a complete task. Within a logical task it
 # may only grow, never shrink or recenter during start progress.
 previous={}
 for e in events:
  assert e['complete_task_scope'];key=json.dumps(e['task'],sort_keys=True);box=e['bounds']
  assert f"StaticLayer: Resizing costmap to {e['shape'][1]} X {e['shape'][0]} at 0.050000 m/pix" in native_log
  if key in previous:
   old=previous[key];assert box[0]<=old[0] and box[1]>=old[1] and box[2]<=old[2] and box[3]>=old[3]
  previous[key]=box
 reads=load(directory/'smac/lifecycle_evidence/readbacks.json')
 return {'bootstrap_cells':256,'original_map_yaml_not_loaded':True,
         'geometry_events':len(events),'complete_task_events':events,
         'actual_readback_requests':len(reads),'readback_bytes':sum(r.get('bytes',0) for r in reads),
         'readback_errors':[r for r in reads if not r.get('success')]}

def compare(a,b,allow_diagnostic_subset=False,report_upstream_mismatch=False):
 # Reporting mismatches preserves a failed adoption gate; it never makes
 # an upstream difference acceptable. All Native and strict checks still run.
 records=pair(a,b,False,require_upstream=not report_upstream_mismatch);left={tag(r):r for r in read(a)};right={tag(r):r for r in read(b)}
 assert right.keys()<=left.keys() if allow_diagnostic_subset else left.keys()==right.keys()
 quality=True;upstream_gate=True
 for item in records:
  t=item['tag'];x,y=left[t],right[t];d=y['l3']
  assert x['success']==y['success'],(t,'strict success regression')
  if not y['success']:quality=False;continue
  assert y['audit']['valid'] and y['constraints']['steering_jump_valid'] and y['constraints']['traffic']['valid']
  upstream=(all(item['same_outputs'][f] for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','l3_scope.npz'))
            and all(item['L2_parity'].values())
            and d['r5_source_full_grid_hash']==x['l3'].get('r5_source_full_grid_hash',x['l3']['semantic_expected_grid_hash'])
            and d['r5_source_full_master_hash']==x['l3'].get('r5_source_full_master_hash',x['l3']['semantic_expected_master_hash']))
  if not report_upstream_mismatch:assert upstream,(t,'upstream/global source changed')
  upstream_gate &= upstream
  mask=np.load(b/'cases'/t/'l3_scope.npz')['mask']
  mask_hash=hashlib.sha256(np.ascontiguousarray(mask).tobytes()).hexdigest()
  assert d['r5_source_full_mask_hash']==mask_hash,(t,'global mask source')
  if 'canonical_mask_hash' in d:
   assert d['canonical_mask_hash']==mask_hash and d['canonical_path_within_mask']
   if upstream:assert d['canonical_mask_hash']==x['l3']['canonical_mask_hash']
  else:assert y['fullroute']['full_original_audits'] and y['fullroute']['reused'],(t,'original audit evidence for reused route')
  assert d['costmap_update_acknowledged'] and d['costmap_ack_mismatch_cells']==0
  assert d['server_costmap_content_hash']==d['semantic_expected_server_content_hash']
  assert d['semantic_exact_stable_observations']>=2
  h,w=d['r5_native_shape'];assert (h,w)==(d['local_map_height_cells'],d['local_map_width_cells'])
  assert 0<d['costmap_ack_checked_cells']<=h*w
  box=d['r5_complete_task_bounds'];assert (h,w)==(box[1]-box[0],box[3]-box[2])
  assert d['r5_complete_scope_not_rolling'] and not d['r5_original_map_loaded_in_nav2']
  r0,r1,c0,c1=box
  assert not (np.any(mask[:r0]) or np.any(mask[r1:]) or np.any(mask[r0:r1,:c0]) or np.any(mask[r0:r1,c1:])),(t,'complete route scope escaped map')
  path=np.load(b/'cases'/t/'path.npy');origin=d['r5_native_origin'];res=.05
  assert np.all(path[:,0]>=origin[0]) and np.all(path[:,0]<origin[0]+w*res)
  assert np.all(path[:,1]>=origin[1]) and np.all(path[:,1]<origin[1]+h*res)
  before,after=[load(p/'cases'/t/'outcome.json')['metrics'] for p in (a,b)]
  length=y['length_m']/x['length_m'];clearance=after['minimum_clearance_m']-before['minimum_clearance_m']
  numeric=relative_quality(x['length_m'],y['length_m'],before['minimum_clearance_m'],after['minimum_clearance_m'])
  gate=numeric['quality_gate'];quality &= gate
  item.update(numeric)
  item.update(global_source_and_mask_identical=bool(upstream),predeclared_upstream_gate=bool(upstream),current_native_cells_in_verified_content_hash=h*w,
              affected_cells_exactly_compared=d['costmap_ack_checked_cells'],
              ack_evidence_reused=bool(d.get('semantic_ack_reused')),
              native_bounds=box,native_origin=origin,full_route_inside_task_map=True,
              control_global_source_hashes_are_not_native_observation=bool(x['l3'].get('r5_native_shape')),
              length_ratio=length,minimum_clearance_delta_m=clearance,quality_gate=gate,
              local_server_hash_is_not_global=True)
 return {'n':len(records),'quality_gate':bool(quality),'upstream_gate':bool(upstream_gate),'records':records,
         'upstream_mismatch_reporting_only':report_upstream_mismatch,
         'upstream_failed_tags':[r['tag'] for r in records if not r.get('predeclared_upstream_gate',False)],
         'max_length_increase_pct':max(r.get('length_change_pct',0) for r in records),
         'minimum_clearance_delta_m':min(r.get('minimum_clearance_delta_m',0) for r in records)}

def plot(data,kind):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
 splits=['diagnostic'] if kind=='diagnostic' else ['first','repeat']
 for split in splits:
  fig,axes=plt.subplots(2,3,figsize=(15,8))
  for rr,(mid,label) in enumerate(MAPS):
   for cc,(metric,title) in enumerate([('e2e_s','实际查询端到端 / s'),('tree_rss_gib','查询进程树峰值RSS / GiB'),('python_pss_gib','独立Python进程PSS快照 / GiB')]):
    ax=axes[rr,cc];values=[]
    for mode in ('control','CHNT'):
     rows=data[mid][mode];selected=rows if split=='diagnostic' else parts(rows,'fixed')[split]
     values.append([measure(r,metric) for r in selected])
    if split=='diagnostic':
     for i,v in enumerate(values):ax.scatter(i+np.linspace(-.07,.07,len(v)),v,s=45)
     ax.set_xticks([0,1],[f'{m} n={len(v)}' for m,v in zip(('control','CHNT'),values)])
    else:ax.boxplot(values,labels=[f'{m} n={len(v)}' for m,v in zip(('control','CHNT'),values)],whis=1.5,showfliers=True)
    ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V9-r7：'+{'diagnostic':'完整任务存图诊断','first':'首次完整规划','repeat':'重复完整规划'}[split],fontsize=17)
  note=('每图每组仅三条相关诊断，各一次；显示原始点，不能代表固定10组分布。' if split=='diagnostic' else
        '逐次原始记录；箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值；圆点离群值。')
  fig.text(.5,.02,note+'\n线性坐标；RSS含Python与ROS子进程，PSS为独立Python快照；初始化单独记录；严格成功另列表。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report'/(split+'_r7.png'),dpi=160);plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);p.add_argument('--phase',choices=['diagnostic','fixed'],required=True);a=p.parse_args();root=a.results_root.resolve()
 raw={};out={'version':'3YD-V9-r7','phase':a.phase,'diagnostic_only':a.phase=='diagnostic','maps':{},'results_root':str(root)}
 for mid,label in MAPS:
  ds={m:root/a.phase/m/mid for m in ('control','CHNT')};rows={m:read(d) for m,d in ds.items()};raw[mid]=rows
  assert all(len(r)==(3 if a.phase=='diagnostic' else 30) for r in rows.values())
  item={'label':label,'arms':{m:{k:summary(v) for k,v in parts(r,'fixed').items()} for m,r in rows.items()},
        'parity':compare(ds['control'],ds['CHNT']),'mode_evidence':{m:mode_evidence(d,m) for m,d in ds.items()},
        'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in ds.items()},
        'cleanup':{m:verify_cleanup(d) for m,d in ds.items()}}
  if a.phase=='diagnostic':item['cold_single_observation']={m:{k:rows[m][0][k] for k in ('cold_first_response_ms','cold_tree_rss_peak_bytes')} for m in ds}
  out['maps'][mid]=item
 if a.phase=='fixed':
  def value(item,mode,split,metric,stat='median'):return item['arms'][mode][split]['metrics'][metric][stat]
  out['predeclared_fixed_gates']={
   'quality':all(v['parity']['quality_gate'] for v in out['maps'].values()),
   'strict':all(arm['all']['success']==30 for v in out['maps'].values() for arm in v['arms'].values()),
   'both_first_medians_improve_at_least_3_percent':all(value(v,'CHNT','first','e2e_s')<=.97*value(v,'control','first','e2e_s') for v in out['maps'].values()),
   'both_query_tree_RSS_medians_improve_at_least_5_percent':all(value(v,'CHNT','all','tree_rss_gib')<=.95*value(v,'control','all','tree_rss_gib') for v in out['maps'].values()),
   'repeat_medians_regress_at_most_5_percent':all(value(v,'CHNT','repeat','e2e_s')<=1.05*value(v,'control','repeat','e2e_s') for v in out['maps'].values()),
   'first_and_repeat_P95_regress_at_most_5_percent':all(value(v,'CHNT',split,'e2e_s','p95')<=1.05*value(v,'control',split,'e2e_s','p95') for v in out['maps'].values() for split in ('first','repeat'))}
  out['adoption_eligible']=False
  out['remaining_required_checks']='independent cold n=3, moving/perturbed/event regressions, all original/frozen/parent/production/source locks, cleanup and reviewed default entry; fixed gates alone cannot adopt'
 save(ROOT/'report'/('R7_'+a.phase.upper()+'_COMPARISON.json'),out);plot(raw,a.phase)
 print({mid:{m:{'first_s':v['first']['metrics']['e2e_s']['median'],'RSS_GiB':v['all']['metrics']['tree_rss_gib']['median'],'strict':v['all']['success']} for m,v in item['arms'].items()} for mid,item in out['maps'].items()})
 print('QUALITY_GATE',all(v['parity']['quality_gate'] for v in out['maps'].values()))
 if a.phase=='fixed':print('FIXED_GATES',out['predeclared_fixed_gates'])
if __name__=='__main__':main()
