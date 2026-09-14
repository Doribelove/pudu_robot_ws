"""Standalone CLI; inputs/results do not depend on any prior architecture."""
import argparse
import json
import resource
import time
import sys
import math
from pathlib import Path
import yaml
import numpy as np
from . import ARCHITECTURE_ID,IMPLEMENTATION_ID,PROTOCOL_ID
from .model import load_scene
from .solver import plan
from .output import sample_trajectory,plot_result,sha256


def dump(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False))


def run_scene(data,out,budget,candidates,maxiter,physical_map=None,transition_mode="joint",render_plot=True,refine_on_failure=True):
    scene=load_scene(data)
    if not math.isfinite(budget) or budget<=0 or candidates<1 or maxiter<1:
        raise ValueError('INVALID_BUDGET')
    sources={str(p):sha256(p) for p in Path(__file__).parent.glob('*.py')}
    sources.update(scene.metadata.get('source_file_hashes',{}))
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    dump(out/'scene.json',data)
    started=time.process_time();request_start=time.monotonic()
    result=plan(scene,budget,candidates,maxiter,transition_mode,refine_on_failure)
    rows=sample_trajectory(result['trajectory'])
    if physical_map is not None:
        from .audit import audit_footprint
        free,cfg=physical_map
        audit_start=time.monotonic()
        audit=audit_footprint(rows,free,cfg,scene.half_length_m,scene.half_width_m)
        audit['wall_s']=time.monotonic()-audit_start
        result['independent_footprint_audit']=audit
        if rows and not audit['valid']:
            result['research_valid']=False;result['status']='INDEPENDENT_FOOTPRINT_REJECTED'
            dump(out/'rejected_candidate_path.json',rows)
            result['trajectory']=None;rows=[]
    if any(sha256(p)!=value for p,value in sources.items()):
        raise ValueError('INPUT_OR_SOURCE_CHANGED_DURING_REQUEST')
    if rows:
        xy=np.array([[p['x'],p['y']] for p in rows])
        result['path_length_m']=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
        result['sampled_max_curvature_per_m']=max(abs(p['curvature_per_m']) for p in rows)
    else:
        result['path_length_m']=None;result['sampled_max_curvature_per_m']=None
    result.update(architecture_id=ARCHITECTURE_ID,implementation_id=IMPLEMENTATION_ID,
                  protocol_id=PROTOCOL_ID,cpu_s=time.process_time()-started,
                  generation_and_audit_wall_s=time.monotonic()-request_start,
                  legacy_architecture_modules_loaded=[n for n in sys.modules if n.startswith(("arena_","rclpy","nav2"))],
                  process_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                  input_scene_sha256=sha256(out/'scene.json'))
    dump(out/'result.json',result);dump(out/'path.json',rows)
    if render_plot:plot_result(scene,result,out/'trajectory.png')
    dump(out/'protocol.json',{'architecture_id':ARCHITECTURE_ID,'implementation_id':IMPLEMENTATION_ID,
                             'protocol_id':PROTOCOL_ID,'dynamic_obstacles':False,
                             'allow_reverse':False,'allow_in_place_rotation':False,
                             'minimum_turning_radius_m':scene.radius_min_m,
                             'padded_footprint_half_extents_m':[scene.half_length_m,scene.half_width_m],
                             'refine_on_failure':refine_on_failure,'plot_generated':render_plot,'transition_mode':transition_mode,'solver_budget_s':budget,'candidate_limit':candidates,
                             'solver_max_iterations':maxiter,'full_R2_acceptance':False,
                             'source_hashes':sources,
                             'dataset':scene.metadata})
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description='STL-V0 independent static trajectory research')
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('plan');p.add_argument('--scene',required=True);p.add_argument('--out',required=True)
    r=sub.add_parser('real-preflight');r.add_argument('--workspace',default='/home/robot/pudu_robot_ws')
    r.add_argument('--out',required=True);r.add_argument('--query-ids',nargs='*')
    r.add_argument('--prepare-only',action='store_true')
    r.add_argument('--cache-dir',help='Content-addressed map and exact query/ROI cover cache; never caches paths')
    r.add_argument('--roi-margin-m',type=float,default=20.)
    r.add_argument('--cover-mode',choices=['legacy','repaired'],default='repaired')
    for p0 in [p,r]:
        p0.add_argument('--budget-s',type=float,default=30.)
        p0.add_argument('--candidates',type=int,default=6)
        p0.add_argument('--maxiter',type=int,default=180)
        p0.add_argument('--no-refine',action='store_true',help='Disable failure-triggered curve subdivision for pure speed comparison')
        p0.add_argument('--transition-mode',choices=['joint','fixed_position','fixed_pose'],default='joint')
    study=sub.add_parser('study-r1');study.add_argument('--out',required=True);study.add_argument('--workspace',default='/home/robot/pudu_robot_ws')
    study.add_argument('--repeats',type=int,default=3);study.add_argument('--budget-s',type=float,default=30.)
    study.add_argument('--candidates',type=int,default=6);study.add_argument('--maxiter',type=int,default=240)
    perf=sub.add_parser('benchmark-r2');perf.add_argument('--out',required=True);perf.add_argument('--workspace',default='/home/robot/pudu_robot_ws')
    perf.add_argument('--repeats',type=int,default=3);perf.add_argument('--budget-s',type=float,default=30.)
    perf.add_argument('--candidates',type=int,default=6);perf.add_argument('--maxiter',type=int,default=240)
    args=parser.parse_args(argv)
    try:
        if args.command=='benchmark-r2':
            from .performance import benchmark
            benchmark(args.out,args.workspace,args.repeats,args.budget_s,args.candidates,args.maxiter)
            return 0
        if args.command=='study-r1':
            from .study import run_study
            run_study(args.out,args.workspace,args.repeats,args.budget_s,args.candidates,args.maxiter)
            return 0  # Completed experiment, not all queries passed.
        if args.command=='plan':
            source=Path(args.scene)
            data=json.loads(source.read_text()) if source.suffix=='.json' else yaml.safe_load(source.read_text())
            result=run_scene(data,args.out,args.budget_s,args.candidates,args.maxiter,transition_mode=args.transition_mode,refine_on_failure=not args.no_refine)
            print(json.dumps({'architecture_id':ARCHITECTURE_ID,'status':result['status'],'out':str(Path(args.out).resolve()),'planning_wall_s':result['planning_wall_s']},ensure_ascii=False))
            return 0 if result['research_valid'] else 2
        from .map_input import load_dataset,make_scene
        root=Path(args.workspace)/'private_data/pudu_wanda_3f'
        started=time.monotonic()
        paths=(root/'extracted/optemap.yaml',root/'results/conversion_v1/semantic_map_v1.json',root/'query_sets/semantic_compare_selected8_r2_v2/selected_queries.yaml')
        cache_timing=None
        if args.cache_dir:
            from .prepared_cache import prepared_dataset
            dataset,cache_timing=prepared_dataset(*paths,args.cache_dir)
        else:dataset=load_dataset(*paths)
        free,safe,nostop,cfg,semantic,qs,evidence=dataset
        prep=time.monotonic()-started
        ids=set(args.query_ids or [q['query_id'] for q in qs['queries']])
        if ids-set(q['query_id'] for q in qs['queries']):raise ValueError('UNKNOWN_QUERY_ID')
        out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
        dump(out/'input_binding.json',evidence);dump(out/'semantic_source.json',semantic)
        summaries=[]
        for q in qs['queries']:
            if q['query_id'] not in ids:continue
            t=time.monotonic();cover_cache=None
            try:
                if args.cache_dir:
                    from .prepared_cache import prepared_scene
                    data,cover_cache=prepared_scene(safe,nostop,cfg,q,evidence,args.cache_dir,margin_m=args.roi_margin_m,cover_mode=args.cover_mode)
                else:
                    data=make_scene(safe,nostop,cfg,q,evidence,margin_m=args.roi_margin_m,cover_mode=args.cover_mode);cover_cache=None
                cover_s=time.monotonic()-t
                if args.prepare_only:
                    dump(out/(q['query_id']+'.scene.json'),data)
                    summaries.append({'query_id':q['query_id'],'status':'PREPARED_ONLY','cover_s':cover_s,'regions':len(data['regions'])})
                else:
                    result=run_scene(data,out/q['query_id'],args.budget_s,args.candidates,args.maxiter,physical_map=(free,cfg),transition_mode=args.transition_mode,refine_on_failure=not args.no_refine)
                    summaries.append({'query_id':q['query_id'],'status':result['status'],'research_valid':result['research_valid'],'regions':len(data['regions']),'cover_s':cover_s,'planning_wall_s':result['planning_wall_s'],'r2_semantic_acceptance':None})
            except ValueError as exc:
                summaries.append({'query_id':q['query_id'],'status':'INPUT_OR_COVER_REJECTED','reason':str(exc),'research_valid':False})
            print(json.dumps(summaries[-1]),flush=True)
            dump(out/'summary.json',{'architecture_id':ARCHITECTURE_ID,'phase':'physical-preflight-only','dataset_prepare_s':prep,'map_cache':cache_timing,'last_cover_cache':cover_cache,'results':summaries,'production_accepted':False,'full_semantic_acceptance':None})
        return 0 if args.prepare_only or all(s.get('research_valid',False) for s in summaries) else 2
    except (ValueError,FileNotFoundError,FileExistsError,KeyError) as exc:
        print(json.dumps({'architecture_id':ARCHITECTURE_ID,'status':'REJECTED','reason':str(exc)}),file=sys.stderr)
        return 3
