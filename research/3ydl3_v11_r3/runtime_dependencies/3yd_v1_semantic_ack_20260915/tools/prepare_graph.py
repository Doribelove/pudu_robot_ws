from arena_3d_v1.semantic_world import *
from arena_3d_v1.semantic_graph import SemanticPoseGraph
import time
import os
os.sched_setaffinity(0,{15})
world=SemanticWorld('/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/extracted/optemap.yaml','/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json',cache_root=WORK/'cache')
graph=SemanticPoseGraph(world,WORK/'cache/graph')
print('SOURCE',world.key,'GRAPH',graph.key,'candidate_nodes',len(graph.nodes),flush=True)
graph.prepare()
json_write(WORK/'results/graph_preparation_v4.json',{'key':graph.key,'world_key':world.key,'node_count':len(graph.nodes),'edge_count':len(graph.edges),'rejected_or_unverified_count':len(graph.records),'world_prepare_ms':world.preparation_ms,'region_and_node_prepare_ms':graph.preparation_ms,'edge_validation_ms':graph.build_connections_ms})
print('PREPARED',len(graph.edges),'edges',graph.build_connections_ms,'ms',flush=True)
