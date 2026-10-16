import hashlib,json
from pathlib import Path
import yaml
from arena_3d_v1.semantic_world import WORK,json_write
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
source=json.loads((WORK/'BASELINE_SOURCE_MANIFEST.json').read_text());base=Path(source['baseline_root'])
freeze=json.loads((WORK/'report/IMPLEMENTATION_FREEZE.json').read_text())
final_freeze=json.loads((WORK/'report/FINAL_IMPLEMENTATION_FREEZE.json').read_text())
r={'baseline_files_checked':len(source['files']),'baseline_changed':[k for k,h in source['files'].items() if not (base/k).exists() or sha(base/k)!=h],
'implementation_files_checked':len(freeze),'implementation_changed':[k for k,h in freeze.items() if not (WORK/k).exists() or sha(WORK/k)!=h],
'original_runtime_modules_changed_in_experiment':[k for k,h in source['files'].items() if '/arena_3d_v1/' in k and k.endswith('.py') and (WORK/k).exists() and sha(WORK/k)!=h]}
r['final_implementation_changed']=[k for k,h in final_freeze.items() if not (WORK/k).exists() or sha(WORK/k)!=h]
cache_check=json.loads((WORK/'report/cache_restore_validation.json').read_text())
r['documented_post_query_cache_correction']=r['implementation_changed']==['external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/multires_topology.py'] and cache_check['passed']==40 and cache_check['changed_methods']==['__init__']
params=[WORK/'results'/f'formal_01_{a}'/'smac/logs/smac_strict_real.yaml' for a in ['baseline','topology_005','topology_015']]
r['smac_parameter_hashes']={str(p):sha(p) for p in params};r['smac_parameters_identical']=len(set(r['smac_parameter_hashes'].values()))==1
r['sweep_binary_identical']=sha(WORK/'build/libsemantic_sweep.so')==sha(base/'build/libsemantic_sweep.so')
cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());mp=Path(cfg['inputs']['map_yaml']);md=yaml.safe_load(mp.read_text());image=mp.parent/md['image']
r['inputs']={k:{'actual':sha(p),'expected':cfg['inputs'][k]} for p,k in [(mp,'map_yaml_sha256'),(image,'map_image_sha256'),(cfg['inputs']['semantic_path'],'semantic_file_sha256'),(WORK/'config/queries.yaml','queries_file_sha256')]}
r['inputs_unchanged']=all(v['actual']==v['expected'] for v in r['inputs'].values());r['pass']=not any([r['baseline_changed'],r['final_implementation_changed'],r['original_runtime_modules_changed_in_experiment']]) and r['documented_post_query_cache_correction'] and r['smac_parameters_identical'] and r['inputs_unchanged'] and r['sweep_binary_identical']
json_write(WORK/'report/integrity.json',r);print(json.dumps(r,indent=2));assert r['pass']
