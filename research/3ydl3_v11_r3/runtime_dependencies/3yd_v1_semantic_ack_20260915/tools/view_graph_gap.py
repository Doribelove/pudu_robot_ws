import os,yaml,json
os.sched_setaffinity(0,{14})
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_3d_v1.semantic_world import SemanticWorld,WORK
from arena_3d_v1.semantic_graph import SemanticPoseGraph
f=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(f['inputs']['map_yaml'],f['inputs']['semantic_path'],cache_root=WORK/'cache');g=SemanticPoseGraph(w,WORK/'cache/graph')
fig,ax=plt.subplots(figsize=(8,10));m=w.map
ax.imshow(~w.hard,cmap='gray',extent=[m.origin[0],m.origin[0]+m.width*.05,m.origin[1],m.origin[1]+m.height*.05],origin='upper')
for n in g.nodes.values():
 if n.from_region==818 or n.to_region==818:ax.arrow(*n.pose[:2],.25*np.cos(n.pose[2]),.25*np.sin(n.pose[2]),color='red',width=.02)
p=np.load(WORK/'results/real_ack_probe_04/cases/cand2-06b-lane-to-parking__original_r1__1/path.npy');ax.plot(p[:,0],p[:,1],color='lime',linewidth=1.5)
ax.set_xlim(-63,-42);ax.set_ylim(-18,8);ax.set_axis_off();fig.tight_layout(pad=0);fig.savefig(WORK/'results/graph_gap_01.png',dpi=130)
