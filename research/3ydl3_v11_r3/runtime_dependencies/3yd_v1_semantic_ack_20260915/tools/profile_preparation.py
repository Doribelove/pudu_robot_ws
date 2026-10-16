"""Measure cold semantic import, graph construction/serialization and cache restore."""
import argparse,gc,os,time,json
from pathlib import Path
from benchmark import Memory
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write
from arena_3d_v1.semantic_graph import SemanticPoseGraph
import yaml

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False);os.sched_setaffinity(0,{15})
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());rows=[]
 def measure(name,fn):
  start=time.monotonic();cpu=time.process_time()
  with Memory() as mem:value=fn()
  row={'phase':name,'wall_ms':(time.monotonic()-start)*1000,'cpu_ms':(time.process_time()-cpu)*1000,'peak_rss_bytes':mem.peak};rows.append(row);json_write(out/'timings.json',rows);print(name,row,flush=True);return value
 world=measure('cold_semantic_import_and_grid_build',lambda:SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=out/'cache'))
 graph=measure('cold_semantic_region_and_pose_generation',lambda:SemanticPoseGraph(world,out/'cache/graph'))
 measure('cold_connection_validation_and_serialization',graph.prepare)
 meta={'world_key':world.key,'graph_key':graph.key,'nodes':len(graph.nodes),'edges':len(graph.edges),'rejected_or_unverified':len(graph.records),'connection_validation_ms':graph.build_connections_ms,'cache_bytes':sum(p.stat().st_size for p in (out/'cache').rglob('*') if p.is_file()),'annotation_human_time':'not measured; existing manual annotations reused','note':'Cold cache empty at start; cache restore in same process is reported separately.'}
 del graph;del world;gc.collect()
 world=measure('semantic_base_cache_restore',lambda:SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=out/'cache'))
 graph=measure('graph_cache_restore',lambda:SemanticPoseGraph(world,out/'cache/graph'))
 json_write(out/'summary.json',meta)
if __name__=='__main__':main()
