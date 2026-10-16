"""Empty-cache build and fresh-process restore, one arm/process at a time."""
import argparse,os,time
from pathlib import Path
import yaml
from benchmark import Memory
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.multires_grid import GridView

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--arm',choices=['baseline','topology_005','topology_015'],required=True);p.add_argument('--phase',choices=['build','restore'],required=True);a=p.parse_args()
 out=Path(a.output)
 if a.phase=='build':out.mkdir(parents=True,exist_ok=False)
 elif not (out/'build.json').exists():raise ValueError('RESTORE_REQUIRES_COMPLETED_BUILD')
 os.sched_setaffinity(0,{15});cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());rows=[]
 def measure(label,fn):
  t=time.monotonic();cpu=time.process_time()
  with Memory() as m:v=fn()
  rows.append({'phase':label,'wall_ms':(time.monotonic()-t)*1000,'cpu_ms':(time.process_time()-cpu)*1000,'rss_peak_bytes':m.peak});print(a.arm,a.phase,rows[-1],flush=True);json_write(out/(a.phase+'_progress.json'),rows);return v
 w=measure('semantic_world',lambda:SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=out/'world'))
 if a.arm=='baseline':
  g=measure('graph_constructor',lambda:SemanticPoseGraph(w,out/'graph'))
  measure('graph_connection_validation_serialization',g.prepare)
 else:
  g=measure('graph_constructor',lambda:SemanticTopology(w,out/'graph'))
  if a.arm=='topology_015':measure('coarse_view_build',lambda:GridView(w,3))
 json_write(out/(a.phase+'.json'),{'arm':a.arm,'phase':a.phase,'rows':rows,'graph_cache_hit':g.cache_hit,'world_cache_hit':w.cache_hit,'graph_nodes':len(g.nodes),'graph_edges':len(g.edges) if a.arm=='baseline' else g.edges_count,'graph_cache_bytes':sum(x.stat().st_size for x in (out/'graph').rglob('*') if x.is_file()),'world_cache_bytes':sum(x.stat().st_size for x in (out/'world').rglob('*') if x.is_file()),'graph_validation_ms':getattr(g,'build_connections_ms',0.),'map_hash':w.map.sha256,'world_key':w.key,'graph_key':g.key})
if __name__=='__main__':main()
