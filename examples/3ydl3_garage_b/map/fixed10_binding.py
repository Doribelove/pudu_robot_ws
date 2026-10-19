"""Fail-closed registry for immutable map/query pairs; no ROS dependency."""
from pathlib import Path
import hashlib
import json
import math
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parent
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def resolve(map_name, registry_path=None):
    registry_path = Path(registry_path or ROOT/'registry.json')
    registry = json.loads(registry_path.read_text())
    matches = [m for m in registry['maps'] if map_name in [m['map_id'], m['map_name'], *m['aliases']]]
    if len(matches) != 1:
        raise ValueError('MAP_NOT_REGISTERED: '+map_name)
    entry = matches[0]
    binding_path = registry_path.parent/entry['binding']
    if sha(binding_path) != entry['binding_sha256']:
        raise ValueError('MAP_BINDING_CHANGED: '+str(binding_path))
    binding = json.loads(binding_path.read_text())
    inputs = binding_path.parent/'inputs'
    required = {'queries.json','map.yaml','optemap.pgm','semantic.json','topology.json','parking_access.json'}
    if set(binding['input_sha256']) != required:
        raise ValueError('INCOMPLETE_MAP_BINDING')
    for name, expected in binding['input_sha256'].items():
        if sha(inputs/name) != expected:
            raise ValueError('FROZEN_INPUT_CHANGED: '+str(inputs/name))
    query = json.loads((inputs/'queries.json').read_text())
    semantic = json.loads((inputs/'semantic.json').read_text())
    topology = json.loads((inputs/'topology.json').read_text())
    access = json.loads((inputs/'parking_access.json').read_text())
    config = yaml.safe_load((inputs/'map.yaml').read_text())
    mid = entry['map_id']
    if any(obj['map_id'] != mid for obj in [binding,query,access]):
        raise ValueError('MAP_QUERY_ID_MISMATCH')
    if query['query_set_id'] != binding['query_set_id'] or query['frame_id'] != semantic['frame_id']:
        raise ValueError('QUERY_FRAME_OR_VERSION_MISMATCH')
    for name,key in [('semantic.json','semantic_sha256'),('topology.json','topology_sha256'),('optemap.pgm','raster_sha256')]:
        if query[key] != sha(inputs/name): raise ValueError('QUERY_MAP_FINGERPRINT_MISMATCH')
    if config['image'] != 'optemap.pgm' or config['origin'] != semantic['origin'] or config['resolution'] != semantic['resolution']:
        raise ValueError('MAP_COORDINATE_CONTRACT_MISMATCH')
    with Image.open(inputs/'optemap.pgm') as im:
        if im.size != (semantic['width'],semantic['height']): raise ValueError('MAP_DIMENSIONS_MISMATCH')
    cases=query['scenarios']; expected=[f'Q{i:02}' for i in range(1,11)]
    if [q['id'] for q in cases] != expected or binding['default_query_ids'] != expected:
        raise ValueError('FIXED10_REQUIRES_EXACTLY_Q01_TO_Q10')
    regions={r['id']:r for r in topology['regions']}
    poses=set(); expected_parking=set()
    for q in cases:
        for role in ['start','goal']:
            p=q[role]; pose=tuple(p[k] for k in ['x','y','yaw_rad'])
            if not all(math.isfinite(v) for v in pose): raise ValueError('NONFINITE_POSE')
            if pose in poses: raise ValueError('DUPLICATE_POSE')
            poses.add(pose)
            if regions[p['region_id']]['kind']=='parking_area':expected_parking.add((q['id'],role,p['region_id']))
    bindings=access['bindings']
    if {(b['query_id'],b['endpoint'],b['parking_region_id']) for b in bindings} != expected_parking or len(bindings)!=len(expected_parking) or access['unresolved']:
        raise ValueError('PARKING_ACCESS_NOT_COMPLETE')
    # Human-readable names follow the current registry after frozen inputs pass
    # verification. Do not rewrite the archived query bytes or their hashes.
    binding = {**binding, 'map_name': entry['map_name']}
    query = {**query, 'map_name': entry['map_name']}
    return inputs, binding, query

def describe(map_name):
    inputs,binding,query=resolve(map_name)
    return {'map_id':binding['map_id'],'map_name':binding['map_name'],'query_set_id':binding['query_set_id'],'inputs':str(inputs),'query_ids':binding['default_query_ids'],'query_sha256':sha(inputs/'queries.json'),'map_binding_sha256':sha(inputs.parent/'binding.json'),'pairs':len(query['scenarios']),'status':'verified'}
