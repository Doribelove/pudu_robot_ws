"""Reproducible r1 coverage, real-lane semantics and paired pose ablation.

Three repeats measure deterministic same-process repeatability, not cold-start
latency, multi-map generalization or random-initialization robustness.
"""
import argparse
import json
import shutil
import time
from pathlib import Path
from . import ARCHITECTURE_ID,IMPLEMENTATION_ID
from .map_input import load_dataset,make_scene
from .semantic import make_lane_fixture,audit_local_rule
from .cli import run_scene,dump
from .output import sha256


def run_study(out,workspace='/home/robot/pudu_robot_ws',repeats=3,budget_s=30.,candidates=6,maxiter=240):
    if repeats<1:raise ValueError('INVALID_REPEATS')
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    package=Path(__file__).parent.parent
    source={str(p):sha256(p) for p in package.rglob('*') if p.is_file() and not any(x in p.parts for x in ['__pycache__','.pytest_cache'])}
    shutil.copytree(package,out/'source_snapshot',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    dump(out/'source_hashes.json',source)
    started=time.monotonic();root=Path(workspace)/'private_data/pudu_wanda_3f'
    free,safe,ns,cfg,sem,qs,evidence=load_dataset(root/'extracted/optemap.yaml',root/'results/conversion_v1/semantic_map_v1.json',root/'query_sets/semantic_compare_selected8_r2_v2/selected_queries.yaml')
    prep_s=time.monotonic()-started;dump(out/'input_binding.json',evidence)
    summary={'architecture_id':ARCHITECTURE_ID,'implementation_id':IMPLEMENTATION_ID,'dataset_prepare_s':prep_s,
             'repeats':repeats,'solver_budget_s':budget_s,'candidate_limit':candidates,'maxiter':maxiter,
             'repeat_scope':'same process deterministic paired repeats; not cold start or seed perturbation',
             'production_accepted':False,'full_R2_acceptance':None,'physical':[],'semantic':[],'controlled':[]}
    bases={}
    for query in qs['queries']:
        cover_start=time.monotonic();base=make_scene(safe,ns,cfg,query,evidence);cover_s=time.monotonic()-cover_start
        bases[query['query_id']]=base
        for repeat in range(repeats):
            # Alternate order to avoid assigning all first runs to one arm.
            modes=['joint','fixed_pose'] if repeat%2==0 else ['fixed_pose','joint']
            for mode in modes:
                dest=out/'physical'/query['query_id']/f'{mode}_{repeat}'
                request_start=time.monotonic()
                result=run_scene(base,dest,budget_s,candidates,maxiter,physical_map=(free,cfg),transition_mode=mode,render_plot=(repeat==0))
                row={'query_id':query['query_id'],'mode':mode,'repeat':repeat,'status':result['status'],'valid':result['research_valid'],
                     'cover_s':cover_s,'planning_wall_s':result['planning_wall_s'],
                     'generation_and_audit_wall_s':result['generation_and_audit_wall_s'],
                     'request_including_io_and_optional_plot_s':time.monotonic()-request_start,
                     'cpu_s':result['cpu_s'],'process_peak_rss_kib':result['process_peak_rss_kib'],
                     'path_length_m':result['path_length_m'],'selected_objective':result['selected_objective'],
                     'cover_repair':{k:v for k,v in base['metadata']['cover_repair'].items() if not k.endswith('boxes_cells')},
                     'relative_result':str((dest/'result.json').relative_to(out))}
                summary['physical'].append(row);dump(out/'summary.json',summary)
                print(json.dumps({'phase':'physical',**{k:row[k] for k in ['query_id','mode','repeat','status','planning_wall_s']} }),flush=True)
    for enabled in [False,True]:
        scene=make_lane_fixture(bases['cmp2-01-lane-north'],sem,enabled)
        for repeat in range(repeats):
            for mode in (['joint','fixed_pose'] if enabled else ['joint']):
                dest=out/'semantic'/f'{"rule" if enabled else "no_rule"}_{mode}_{repeat}'
                result=run_scene(scene,dest,budget_s,candidates,maxiter,physical_map=(free,cfg),transition_mode=mode,render_plot=(repeat==0))
                rows=json.loads((dest/'path.json').read_text());audit=audit_local_rule(rows,result['trajectory'],scene['metadata']['local_rule'])
                dump(dest/'local_rule_audit.json',audit)
                row={'rule_enabled':enabled,'mode':mode,'repeat':repeat,'physical_valid':result['research_valid'],
                     'local_rule_valid':audit['valid'],'arm_valid':bool(result['research_valid'] and (not enabled or audit['valid'])),
                     'status':result['status'],'planning_wall_s':result['planning_wall_s'],
                     'generation_and_audit_wall_s':result['generation_and_audit_wall_s'],
                     'path_length_m':result['path_length_m'],'selected_objective':result['selected_objective'],
                     'audit':audit,'relative_result':str((dest/'result.json').relative_to(out))}
                summary['semantic'].append(row);dump(out/'summary.json',summary);print(json.dumps({'phase':'semantic',**row}),flush=True)
    for name in ['straight','turn','right_band','rule_route','deadline_negative']:
        data=json.loads((package/'scenes'/f'{name}.json').read_text())
        result=run_scene(data,out/'controlled'/name,budget_s,candidates,maxiter)
        summary['controlled'].append({'scene':name,'valid':result['research_valid'],'status':result['status']})
    if any(sha256(p)!=value for p,value in source.items()):raise ValueError('SOURCE_CHANGED_DURING_STUDY')
    if any(sha256(p)!=value for p,value in evidence['source_file_hashes'].items()):raise ValueError('DATA_CHANGED_DURING_STUDY')
    summary['source_and_input_unchanged']=True;summary['complete']=True
    dump(out/'summary.json',summary);return summary


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--workspace',default='/home/robot/pudu_robot_ws')
    p.add_argument('--repeats',type=int,default=3);p.add_argument('--budget-s',type=float,default=30.)
    p.add_argument('--candidates',type=int,default=6);p.add_argument('--maxiter',type=int,default=240)
    a=p.parse_args();run_study(a.out,a.workspace,a.repeats,a.budget_s,a.candidates,a.maxiter)

if __name__=='__main__':main()
