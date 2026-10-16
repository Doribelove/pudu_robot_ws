"""Map/query binding adapter only. Own-map fixed Q poses verified against the immutable V8 ten-map manifest."""
from pathlib import Path
import json,hashlib,sys
E=Path(__file__).resolve().parents[1]
F=Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_fixed10_20260922')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load_world(mid,out):
    from arena_3d_v1.semantic_world import SemanticWorld,json_write
    from three_yd_v5.topology_default import DefaultTopology
    from three_yd_v5.free_space_topology import FreeSpaceTopology,load_map_only_world
    entry=next(m for m in json.loads((E/'MAPS.json').read_text())['maps'] if m['map_id']==mid)
    for p,h in entry['input_sha256'].items():
        if sha(p)!=h:raise ValueError('INPUT_CHANGED:'+p)
    inp=Path(entry['inputs']);mp=Path(entry['map_yaml']);qf=Path(entry['query_file']);semantic=entry['kind']!='map_only'
    if entry['kind']=='fixed10_semantic':
        sys.path.insert(0,str(F));from fixed10_binding import resolve
        verified,binding,query_set=resolve(mid)
        if verified!=inp:raise ValueError('REGISTRY_LOCATION_CHANGED')
        qs=query_set['scenarios'];json_write(out/'map_binding.json',binding)
    else:
        qs=json.loads(qf.read_text())['scenarios' if semantic else 'queries']
    if qs!=entry['poses']:raise ValueError('FIXED_POSES_CHANGED')
    if semantic:
        from three_yd_v5.v10_firstready import firstready_construction
        with firstready_construction() as storage:
            w=SemanticWorld(mp,inp/'semantic.json',cache_root=out/'world')
            from three_yd_v5.v10_storage import prepare
            prepare(w)
            g=DefaultTopology(w,inp/'topology.json',out/'graph')
        json_write(out/'raster_storage.json',storage)
        if mid=='pudu_wanda_3f':
            audit=json.loads((mp.parent/'audit.json').read_text())
            assert audit['map_id']==mid and audit['original_pgm_sha256']==sha(inp/'optemap.pgm')
            assert audit['effective_yaml_sha256']==sha(mp) and audit['effective_image_sha256']==sha(mp.parent/'optemap_effective.png')
            assert all(sha(p)==h for p,h in zip(audit['companion_paths'],audit['companion_sha256']))
            from arena_evaluation.planner_benchmark.map_utils import HospitalMap
            original=HospitalMap.load(inp/'map.yaml');assert original.occupancy.shape==w.hard.shape
            w.r12_original_hard=w.hard | (original.occupancy!=0)
            json_write(out/'effective_map_audit.json',audit)
    else:
        w=load_map_only_world(mp,out/'world')
        from three_yd_v5.v10_storage import prepare
        prepare(w)
        g=FreeSpaceTopology(w,out/'graph')
    json_write(out/'map_input_verified.json',entry)
    return semantic,mp,w,g,qs,qf
