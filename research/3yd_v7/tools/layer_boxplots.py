"""Per-map per-layer r6/r7 boxplots from the formal raw records.

This host has no verified SimSun font. Figures explicitly use Noto Serif CJK SC
as a disclosed fallback; never describe their lettering as SimSun.
"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
import numpy as np
from fontTools.ttLib import TTCollection

ROOT=Path(__file__).resolve().parents[1]
BASE=Path('/home/robot/workspaces/3yd_v6_r6_20260924')
MAPS=[('pudu_wanda_3f','万达2w地库'),('mengjia','万达3w地库'),
      ('3yd_synth_multiloop_a_v1','合成地库A：多环路错位连接'),
      ('3yd_synth_twinwing_b_v1','合成地库B：双翼分区与瓶颈'),
      ('3yd_synth_trizone_c_v2','合成地库C')]
VERSIONS=[('r6',BASE/'results/formal_r6'),('r7',ROOT/'results/formal_r7')]
COLORS={'r6':'#3669a8','r7':'#e48428'}
OUT=ROOT/'report/boxplots'
FALLBACK_FONT=Path('/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc')
assert FALLBACK_FONT.is_file(), 'The disclosed CJK fallback font is unavailable'
SC_FONT=Path('/tmp/3yd_noto_serif_cjk_sc.otf')
if not SC_FONT.exists():
    collection=TTCollection(str(FALLBACK_FONT))
    collection.fonts[2].save(str(SC_FONT))
    collection.close()
font_manager.fontManager.addfont(str(SC_FONT))
FONT_NAME=font_manager.FontProperties(fname=str(SC_FONT)).get_name()
assert FONT_NAME == 'Noto Serif CJK SC', FONT_NAME
plt.rcParams.update({'font.family':FONT_NAME,'axes.unicode_minus':False,
                     'font.size':10,'axes.titlesize':11,'figure.titlesize':15})
SHORT_MAPS=['万达2w\n地库','万达3w\n地库','合成A\n多环路','合成B\n双翼瓶颈','合成C']
FIELDS={
    'l1_ms':('L1', 'ms'),
    'l2_controller_s':('L2 控制器', 's'),
    'adapter_s':('L2→L3 适配', 's'),
    'l3_pipeline_s':('L3 流水线', 's'),
    'wall_s':('端到端', 's'),
    'l2_state_mib':('L2 选中候选状态', 'MiB'),
    'python_pss_gib':('Python 工作进程后置快照', 'GiB'),
    'planner_pss_gib':('L3 planner_server 计划调用峰值', 'GiB'),
    'stack_pss_gib':('地图/规划器/生命周期服务 PSS', 'GiB'),
    'tree_peak_rss_gib':('整套进程树查询峰值 RSS', 'GiB'),
}

def hash_file(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def record(row):
    attempts=row['recovery']['attempts']
    planner=[a.get('l3_diagnostics',{}).get('planner_pss_peak_bytes') for a in attempts]
    stack=[a.get('l3_diagnostics',{}).get('stack_pss_peak_bytes') for a in attempts]
    planner=[x for x in planner if x is not None]
    stack=[x for x in stack if x is not None]
    worker=[p['pss'] for p in row['memory_processes'] if p['name']=='python3']
    assert len(worker)==1 and planner and stack
    return dict(l1_ms=row['recovery']['l1_actual_ms'],
                l2_controller_s=row['recovery']['all_candidates_controller_ms']/1000,
                adapter_s=row['recovery']['all_candidates_adapter_ms']/1000,
                l3_pipeline_s=row['recovery']['all_candidates_l3_pipeline_ms']/1000,
                wall_s=row['wall_ms']/1000,
                l2_state_mib=row['l2']['state_memory_bytes']/2**20,
                python_pss_gib=worker[0]/2**30,
                planner_pss_gib=max(planner)/2**30,
                stack_pss_gib=max(stack)/2**30,
                tree_peak_rss_gib=row['rss_peak_bytes']/2**30)

def stats(values):
    a=np.asarray(values,float)
    q1,med,q3=np.quantile(a,[.25,.5,.75])
    lo=q1-1.5*(q3-q1);hi=q3+1.5*(q3-q1)
    inside=a[(a>=lo)&(a<=hi)]
    return dict(n=len(a),minimum=float(a.min()),q1=float(q1),median=float(med),
                q3=float(q3),maximum=float(a.max()),p95=float(np.quantile(a,.95)),
                whisker_low=float(inside.min()),whisker_high=float(inside.max()),
                outliers=[float(v) for v in a if v<lo or v>hi])

def draw_group(ax,field,samples):
    for i,(map_id,_) in enumerate(MAPS):
        if i%2==0:ax.axvspan(i*3+.35,i*3+2.1,color='#f3f5f7',zorder=0)
        for version,offset in [('r6',.85),('r7',1.4)]:
            series=samples[map_id][version][field]
            bp=ax.boxplot(series,positions=[i*3+offset],widths=.42,patch_artist=True,
                          whis=1.5,showfliers=True,manage_ticks=False,
                          medianprops={'color':'#202020','linewidth':1.5},
                          whiskerprops={'linewidth':1.1,'color':COLORS[version]},
                          capprops={'linewidth':1.1,'color':COLORS[version]},
                          flierprops={'marker':'o','markersize':3.3,'markerfacecolor':COLORS[version],
                                      'markeredgecolor':COLORS[version],'alpha':.75})
            bp['boxes'][0].set(facecolor=COLORS[version],edgecolor=COLORS[version],alpha=.66,linewidth=1.2)
    ax.set_xlim(.35,(len(MAPS)-1)*3+2.1)
    ax.set_xticks([i*3+1.125 for i in range(len(MAPS))],SHORT_MAPS)
    ax.grid(axis='y',alpha=.23)
    ax.tick_params(axis='x',length=0,pad=5)
    ax.spines[['top','right']].set_visible(False)
    ax.spines['left'].set_alpha(.35)
    ax.spines['bottom'].set_alpha(.35)

def draw_panels(fields,name,samples,title,log_fields=()):
    fig,axes=plt.subplots(len(fields),1,figsize=(14,3.25*len(fields)+1.0),dpi=180,
                          gridspec_kw={'hspace':.66})
    if len(fields)==1:axes=[axes]
    for ax,field in zip(axes,fields):
        draw_group(ax,field,samples)
        label,unit=FIELDS[field]
        ax.set_title(f'{label}（{unit}）',loc='left',pad=7)
        ax.set_ylabel(unit)
        if field in log_fields:
            ax.set_yscale('log')
            ax.text(.995,.96,'纵轴为对数尺度',transform=ax.transAxes,
                    ha='right',va='top',fontsize=8,color='#505050')
    fig.suptitle(title,y=.995)
    fig.legend(handles=[Patch(facecolor=COLORS[v],alpha=.66,label=v) for v in ('r6','r7')],
               loc='upper right',bbox_to_anchor=(.98,.998),frameon=False,ncol=2)
    fig.subplots_adjust(left=.075,right=.985,top=.93,bottom=.06)
    path=OUT/name
    fig.savefig(path,dpi=180,facecolor='white')
    plt.close(fig)
    return str(path)

def isolated_l1():
    result={}
    for map_id,_ in MAPS:
        result[map_id]={}
        for version,root in [('r6',BASE/'results/l1_only_r6'),
                             ('r7',ROOT/'results/l1_only_r7_formal')]:
            peak=[];query=[];hashes=[]
            for round_index in (1,2,3):
                batch=json.loads((root/map_id/f'round_{round_index}'/'batch.json').read_text())
                peak.append(batch['final_resources']['peak_rss_bytes']/2**20)
                query.extend(x['wall_ms'] for x in batch['queries'])
                hashes.extend(x['output_sha256'] for x in batch['queries'])
            result[map_id][version]={'peak_rss_mib':peak,'query_ms':query,'output_hashes':hashes,
                                     'peak_stats':stats(peak),'query_stats':stats(query)}
        assert result[map_id]['r6']['output_hashes']==result[map_id]['r7']['output_hashes']
    fig,ax=plt.subplots(figsize=(14,4.1),dpi=180)
    for i,(map_id,_) in enumerate(MAPS):
        if i%2==0:ax.axvspan(i*3+.35,i*3+2.1,color='#f3f5f7',zorder=0)
        for version,offset in [('r6',.85),('r7',1.4)]:
            y=result[map_id][version]['peak_rss_mib']
            x=np.full(3,i*3+offset)+np.array([-.07,0,.07])
            ax.scatter(x,y,s=35,color=COLORS[version],alpha=.85)
    ax.set_xlim(.35,(len(MAPS)-1)*3+2.1)
    ax.set_xticks([i*3+1.125 for i in range(len(MAPS))],SHORT_MAPS)
    ax.grid(axis='y',alpha=.23);ax.set_ylabel('MiB')
    ax.set_title('独立 L1 进程峰值 RSS（每地图、每版本仅 3 次启动）',loc='left')
    fig.legend(handles=[Patch(facecolor=COLORS[v],label=v) for v in ('r6','r7')],
               loc='upper right',bbox_to_anchor=(.98,.98),frameon=False,ncol=2)
    ax.spines[['top','right']].set_visible(False)
    fig.subplots_adjust(left=.08,right=.985,top=.84,bottom=.18)
    fig.savefig(OUT/'l1_isolated_memory_points.png',dpi=180,facecolor='white')
    plt.close(fig)
    for versions in result.values():
        for data in versions.values():data.pop('output_hashes')
    return result

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    samples={};sources={};summaries={};csv_rows=[]
    for map_id,name in MAPS:
        samples[map_id]={};sources[map_id]={};summaries[map_id]={}
        for version,root in VERSIONS:
            path=root/map_id/'runs.jsonl'
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            assert len(rows)==10 and len({r['query_id'] for r in rows})==10
            values={field:[] for field in FIELDS}
            for row in rows:
                d=record(row)
                for field,value in d.items():values[field].append(value)
                csv_rows.append({'map_id':map_id,'version':version,'query_id':row['query_id'],
                                 'strict_success':row['success'],**d})
            samples[map_id][version]=values
            sources[map_id][version]=hash_file(path)
            summaries[map_id][version]={field:stats(series) for field,series in values.items()}
    l1=isolated_l1()
    timing=draw_panels(('l1_ms','l2_controller_s','adapter_s','l3_pipeline_s','wall_s'),
                       'timing_boxplots.png',samples,
                       '五张地图的分层与端到端耗时分布（每组 10 条查询）',
                       log_fields=('l1_ms','l2_controller_s','adapter_s'))
    timing_layers=draw_panels(('l1_ms','l2_controller_s','adapter_s','l3_pipeline_s'),
                              'timing_layers_boxplots.png',samples,
                              '五张地图的 L1、L2、适配与 L3 耗时（每组 10 条查询）',
                              log_fields=('l1_ms','l2_controller_s','adapter_s'))
    timing_total=draw_panels(('wall_s',),'timing_e2e_boxplots.png',samples,
                             '五张地图的端到端耗时（每组 10 条查询）')
    memory=draw_panels(('l2_state_mib','python_pss_gib','planner_pss_gib',
                        'stack_pss_gib','tree_peak_rss_gib'),
                       'memory_boxplots.png',samples,
                       '五张地图的状态、服务与整套进程资源分布（每组 10 条查询）',
                       log_fields=('l2_state_mib',))
    memory_components=draw_panels(('l2_state_mib','python_pss_gib','planner_pss_gib'),
                                  'memory_components_boxplots.png',samples,
                                  '五张地图的状态与工作进程资源（每组 10 条查询）',
                                  log_fields=('l2_state_mib',))
    memory_total=draw_panels(('stack_pss_gib','tree_peak_rss_gib'),
                             'memory_total_boxplots.png',samples,
                             '五张地图的服务栈与全树资源（每组 10 条查询）')
    with (OUT/'layer_samples.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,csv_rows[0].keys());writer.writeheader();writer.writerows(csv_rows)
    manifest={'schema':'3YD-V6-r6-r7-layer-boxplots-v1','maps':[{'id':i,'name':name} for i,name in MAPS],
              'colors':COLORS,'panel_order':{'timing':['l1_ms','l2_controller_s','adapter_s',
                                                       'l3_pipeline_s','wall_s'],
                                       'memory':['l2_state_mib','python_pss_gib',
                                                 'planner_pss_gib','stack_pss_gib','tree_peak_rss_gib']},
              'fields':{k:{'name':v[0],'unit':v[1]} for k,v in FIELDS.items()},
              'box_definition':'NumPy linear Q1/Q2/Q3; Tukey whiskers reach furthest observed point within 1.5 IQR; circles are outliers',
              'n_per_map_version':10,
              'font_status':f'SimSun unavailable; figures disclose fallback {FONT_NAME} (face 2 of {FALLBACK_FONT})',
              'log_y_fields':['l1_ms','l2_controller_s','adapter_s','l2_state_mib'],
              'timing_png':timing,'memory_png':memory,
              'timing_layers_png':timing_layers,'timing_e2e_png':timing_total,
              'memory_components_png':memory_components,'memory_total_png':memory_total,
              'l1_memory_png':str(OUT/'l1_isolated_memory_points.png'),
              'raw_sha256':sources,'stats':summaries,'isolated_l1':l1}
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'timing_layers_png':timing_layers,'timing_e2e_png':timing_total,
                      'memory_components_png':memory_components,'memory_total_png':memory_total,
                      'l1_memory_png':manifest['l1_memory_png'],'rows':len(csv_rows)},ensure_ascii=False))

if __name__=='__main__':main()
