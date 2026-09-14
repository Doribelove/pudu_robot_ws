"""Paired r1/r2 timing with a frozen STL-V0-r1 source reference.

Same process/runtime, unchanged scenes, three repeats; geometry-cold and hot
reported separately. These are not process-startup or OS cold-cache benchmarks.
The reference is used only in this benchmark, never as a production fallback.
"""
import importlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import time
from . import IMPLEMENTATION_ID
from .cli import run_scene,dump
from .geometry_graph import clear_geometry_cache
from .prepared_cache import prepared_dataset,prepared_scene,algorithm_hash
from .semantic import make_lane_fixture,audit_local_rule
from .output import sha256


def reference_package(path):
    package=Path(path)/'stl_v0';name='_stlv0_r1_benchmark_reference'
    spec=importlib.util.spec_from_file_location(name,package/'__init__.py',submodule_search_locations=[str(package)])
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    if module.IMPLEMENTATION_ID!='stl-v0-r1':raise ValueError('REFERENCE_MUST_BE_FROZEN_R1')
    return importlib.import_module(name+'.cli'),importlib.import_module(name+'.map_input')


def benchmark(out,workspace='/home/robot/pudu_robot_ws',repeats=3,budget_s=30.,candidates=6,maxiter=240):
    if repeats<1:raise ValueError('INVALID_REPEATS')
    root=Path(workspace);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    reference=root/'experiments/stl_v0/stl_v0_r1_final_20260910/source_snapshot'
    ref_cli,ref_map=reference_package(reference)
    package=Path(__file__).parent.parent;shutil.copytree(package,out/'source_snapshot',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    source_hashes={str(p):sha256(p) for p in package.rglob('*') if p.is_file() and not any(x in p.parts for x in ['__pycache__','.pytest_cache'])}
    reference_hashes={str(p):sha256(p) for p in (reference/'stl_v0').glob('*.py')}
    dump(out/'source_hashes.json',source_hashes);dump(out/'reference_hashes.json',reference_hashes)
    path=root/'private_data/pudu_wanda_3f';inputs=(path/'extracted/optemap.yaml',path/'results/conversion_v1/semantic_map_v1.json',path/'query_sets/semantic_compare_selected8_r2_v2/selected_queries.yaml')
    dataset,map_cold=prepared_dataset(*inputs,out/'cache');_,map_warm=prepared_dataset(*inputs,out/'cache')
    free,safe,ns,cfg,semantic,queries,evidence=dataset;dump(out/'input_binding.json',evidence)
    summary={'implementation_id':IMPLEMENTATION_ID,'reference':'stl-v0-r1','reference_source':str(reference),
             'repeats':repeats,'budget_s':budget_s,'candidate_limit':candidates,'maxiter':maxiter,
             'timing_scope':'same process; geometry-cold/hot separately; excludes process startup and plotting',
             'map_preparation_cold':map_cold,'map_preparation_hot':map_warm,'physical':[],'semantic':[],
             'preparation':[],'controlled':[],'full_R2_acceptance':None,'production_accepted':False}
    bases={}
    for query in queries['queries']:
        begin=time.monotonic();baseline=ref_map.make_scene(safe,ns,cfg,query,evidence);reference_cover_s=time.monotonic()-begin
        scene,cover_cold=prepared_scene(safe,ns,cfg,query,evidence,out/'cache')
        scene_hot,cover_hot=prepared_scene(safe,ns,cfg,query,evidence,out/'cache')
        assert scene['regions']==baseline['regions'] and scene['start']==baseline['start'] and scene['goal']==baseline['goal']
        assert scene_hot==scene
        bases[query['query_id']]=scene
        summary['preparation'].append({'query_id':query['query_id'],'r1_cover_s':reference_cover_s,
              'r2_cold':cover_cold,'r2_hot':cover_hot,'cover_stages':scene['metadata']['cover_stage_timing'],
              'region_count':len(scene['regions']),'regions_identical':True})
        for repeat in range(repeats):
            results={}
            modes=['r1','r2_cold','r2_hot','r2_refined'] if repeat%2==0 else ['r2_cold','r2_hot','r2_refined','r1']
            for mode in modes:
                if mode=='r2_cold':clear_geometry_cache()
                dest=out/'physical'/query['query_id']/f'{mode}_{repeat}'
                kwargs={'physical_map':(free,cfg),'render_plot':repeat==0 and mode=='r2_refined'}
                cli=ref_cli.run_scene if mode=='r1' else run_scene
                if mode!='r1':kwargs['refine_on_failure']=mode=='r2_refined'
                result=cli(scene,dest,budget_s,candidates,maxiter,**kwargs);results[mode]=result
                row={'query_id':query['query_id'],'mode':mode,'repeat':repeat,'valid':result['research_valid'],'status':result['status'],
                     'planning_s':result['planning_wall_s'],'generation_and_audit_s':result['generation_and_audit_wall_s'],
                     'cpu_s':result['cpu_s'],'process_peak_rss_kib':result['process_peak_rss_kib'],
                     'path_length_m':result['path_length_m'],'objective':result['selected_objective'],
                     'timing':result.get('timing'),'refinement_attempts':result.get('refinement_attempts',0),
                     'relative_result':str((dest/'result.json').relative_to(out))}
                summary['physical'].append(row);dump(out/'summary.json',summary)
                print(json.dumps({k:row[k] for k in ['query_id','mode','repeat','valid','planning_s'] }),flush=True)
            expected=[a['route'] for a in results['r1']['candidate_results']]
            for mode in ['r2_cold','r2_hot']:
                assert [a['route'] for a in results[mode]['candidate_results']]==expected,('CANDIDATE_REGRESSION',query['query_id'],mode)
                assert results[mode]['research_valid']==results['r1']['research_valid'],('VALIDITY_REGRESSION',query['query_id'])
                if results['r1']['research_valid']:
                    assert abs(results[mode]['selected_objective']-results['r1']['selected_objective'])<1e-5
            assert not results['r1']['research_valid'] or results['r2_refined']['research_valid']
    for enabled in [False,True]:
        scene=make_lane_fixture(bases['cmp2-01-lane-north'],semantic,enabled)
        for repeat in range(repeats):
            clear_geometry_cache();dest=out/'semantic'/f'{"rule" if enabled else "no_rule"}_{repeat}'
            result=run_scene(scene,dest,budget_s,candidates,maxiter,physical_map=(free,cfg),render_plot=repeat==0)
            audit=audit_local_rule(json.loads((dest/'path.json').read_text()),result['trajectory'],scene['metadata']['local_rule']);dump(dest/'local_rule_audit.json',audit)
            summary['semantic'].append({'rule_enabled':enabled,'repeat':repeat,'physical_valid':result['research_valid'],
                'local_rule_valid':audit['valid'],'planning_s':result['planning_wall_s'],'audit':audit,
                'relative_result':str((dest/'result.json').relative_to(out))})
            dump(out/'summary.json',summary);print(json.dumps(summary['semantic'][-1]),flush=True)
    for name in ['straight','turn','right_band','rule_route','deadline_negative']:
        scene=json.loads((package/'scenes'/f'{name}.json').read_text());result=run_scene(scene,out/'controlled'/name,budget_s,candidates,maxiter)
        summary['controlled'].append({'scene':name,'valid':result['research_valid']})
    for path,expected in {**source_hashes,**reference_hashes,**evidence['source_file_hashes']}.items():
        if sha256(path)!=expected:raise ValueError('SOURCE_OR_INPUT_DRIFT')
    summary.update(complete=True,source_and_input_unchanged=True,all_fast_candidate_sequences_identical=True,
        all_fast_validities_and_objectives_preserved=True)
    dump(out/'summary.json',summary);return summary
