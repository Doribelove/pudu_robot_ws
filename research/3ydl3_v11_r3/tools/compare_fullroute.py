"""Compare actual calls, including reuse overhead and unchanged-path evidence."""
import json,hashlib,ast
from pathlib import Path
from compare_v8_r3 import arm_summary,parity,common_protocol,changes,libraries,read,audit_ast
from compare_v8_r6 import verify_cleanup
from analyze_v8 import measure
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r6_20261008')
MAPS=[('mengjia','万达3w地库'),('arena_large_250x300_2d_005','arena_large（无语义）')]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
 result={'version':'3YD-V8-r7','branch':'fullroute','parent':'3YD-V8-r6 ABC','maps':{},'additional':{},
         'method':'CPU12 serial two independent batches; no profiling; not randomized; identical fixed endpoints',
         'moving_window':False,'unrestricted_smac':False,
         'timing_note':'Formal reuse local_map_update_ms includes sync_only adaptation; excluded from substage comparison. Whole L3 and adapter actual clocks remain comparable.'}
 data={}
 for mid,label in MAPS:
  a=ROOT/'results/control'/mid;b=ROOT/'results/final'/mid
  left,right=arm_summary(a),arm_summary(b);p=parity(a,b)
  # Early formal reuse telemetry measures sync_only including adaptation;
  # native ACK timing in r6 has a narrower scope. Do not compare that subclock.
  for summary in (left,right):summary['metrics'].pop('map_sync_s',None)
  assert left['success']==right['success']==30
  assert all(v==30 for v in p['counts'].values())
  assert {Path(x).name:y for x,y in libraries(a).items()}=={Path(x).name:y for x,y in libraries(b).items()}
  rs=read(b);reuse=[r for r in rs if r.get('fullroute',{}).get('reused')]
  assert len(reuse)==20 and all(r['l3'].get('backend_call_count')==0 for r in reuse)
  result['maps'][mid]={'name':label,'r6_control':left,'r7':right,'path_parity':p,
    'protocol_equal':common_protocol(a,b),'change_percent':changes(left['metrics'],right['metrics']),
    'whole_trajectory_reuse':len(reuse),'max_cached_bytes':max(r['fullroute_cache']['bytes'] for r in rs),
    'max_cache_entries':max(r['fullroute_cache']['entries'] for r in rs)}
  data[mid]={'r6':read(a),'r7':rs}
  for d in (a,b):verify_cleanup(d)
 for mid,label in [('3yd_synth_multiloop_a_v1','合成地库A'),('arena_medium_200x200_2d_005','arena_medium')]:
  a=PARENT/'results/additional'/mid;b=ROOT/'results/additional'/mid
  if not b.exists():continue
  p=parity(a,b);assert all(x['same_success'] for x in p['records'])
  assert all(x['path.npy'] for x in p['records'] if x['path.npy'] is not None)
  result['additional'][mid]={'name':label,'r6_historical':arm_summary(a),'r7':arm_summary(b),'parity':p}
  verify_cleanup(b)
 result['dynamic']={}
 for mid,label in MAPS:
  d=Path(json.loads((ROOT/'report/SELECTED_DATA.json').read_text())['dynamic'][mid])
  if (d/'summary.json').exists():
   result['dynamic'][mid]=json.loads((d/'summary.json').read_text());verify_cleanup(d)
 lock=json.loads((ROOT/'L1_LOCK.json').read_text())
 assert all(sha(ROOT/f)==h for f,h in lock['source_files'].items())
 result['L1_source_files_unchanged']=len(lock['source_files'])
 for field in ('input_files','pixel_input_files'):
  assert all(sha(ROOT/f)==h for f,h in lock[field].items())
  result[field+'_unchanged']=len(lock[field])
 for f,h in lock['l1_outputs'].items():assert sha(ROOT/f.replace('results/formal/','results/final/'))==h
 result['frozen_L1_outputs_unchanged']=len(lock['l1_outputs'])
 parent=json.loads((PARENT/'RELEASE.json').read_text())
 assert all(sha(PARENT/f)==h for f,h in parent['files'].items())
 result['parent_registered_files_unchanged']=len(parent['files'])
 assert audit_ast(PARENT/'src/three_yd_v5/controller.py')==audit_ast(ROOT/'src/three_yd_v5/controller.py')
 result['original_audit_unchanged']=True
 result['cleanup_journals']=[verify_cleanup(p.with_suffix('')) for p in []]
 # Every owned batch (including diagnosis failures) must be complete/clean.
 journals=list((ROOT/'results').rglob('*.supervisor.json'))
 result['cleanup_journals']=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in journals]
 (ROOT/'report/COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 plot(data)
 print(json.dumps({mid:{'success':v['r7']['success'],'reuse':v['whole_trajectory_reuse'],'delta':v['change_percent']['e2e_s']} for mid,v in result['maps'].items()},ensure_ascii=False))

def plot(data):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 f='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(f);name=fm.FontProperties(fname=f).get_name()
 assert name=='SimSun',name
 plt.rcParams.update({'font.family':name,'axes.unicode_minus':False,'font.size':12})
 fig,axes=plt.subplots(2,3,figsize=(15,8))
 for row,(mid,label) in enumerate(MAPS):
  for col,(metric,subset,title) in enumerate([('e2e_s','first','首次完整规划（秒）'),('e2e_s','repeat','重复完整查询（秒）'),('tree_rss_gib','all','查询进程树峰值 RSS（GiB）')]):
   groups=[]
   for arm in ('r6','r7'):
    rows=[r for r in data[mid][arm] if subset=='all' or (r['repeat']==1 if subset=='first' else r['repeat']>1)]
    groups.append([measure(r,metric) for r in rows])
   ax=axes[row,col];bp=ax.boxplot(groups,labels=[f'r6 对照 n={len(groups[0])}',f'r7 完整路线 n={len(groups[1])}'],patch_artist=True,whis=1.5,showfliers=True)
   for box,color in zip(bp['boxes'],('#bfd2ed','#b6d9c0')):box.set_facecolor(color)
   ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
 fig.suptitle('新 V8-r7 与 r6：两张固定地图，各 10 组 × 3 次',fontsize=18,fontweight='bold')
 fig.text(.5,.02,'箱体 Q1–Q3；中线中位数；须为 1.5×IQR 范围内最远数据；圆点为离群点。初始化和结果导出不计入查询时间。',ha='center')
 fig.tight_layout(rect=(0,.065,1,.93));fig.savefig(ROOT/'report/performance_boxplots.png',dpi=180);plt.close(fig)
if __name__=='__main__':main()
