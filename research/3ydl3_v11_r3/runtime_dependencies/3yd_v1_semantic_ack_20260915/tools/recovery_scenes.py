"""Held-out geometry specified before running the recovery candidate."""
import math
from pathlib import Path
import numpy as np,yaml
from PIL import Image
from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature
from arena_3d_v1.semantic_world import json_write

def return_bulb_inputs(root,rotated=False):
    root=Path(root);root.mkdir(parents=True)
    # A forward-only vehicle starts pointing away from the nearby goal.
    # Its initial semantic lane has no room to turn; the separate bulb does.
    cells=np.zeros((180,400),np.uint8)
    def fill(x0,x1,y0,y1):cells[180-int(y1/.05):180-int(y0/.05),int(x0/.05):int(x1/.05)]=255
    fill(1.,13.,3.,3.95);fill(13.,18.,1.,7.)
    areas=[('lane','lane',[[1,3],[11,3],[11,3.95],[1,3.95]]),('bulb','parking_area',[[11,3],[13,3],[13,1],[18,1],[18,7],[13,7],[13,3.95],[11,3.95]])]
    start=(9.,3.475,0.);goal=(7.,3.475,math.pi)
    if rotated:
        cells=np.rot90(cells)
        areas=[(name,kind,[[9-y,x] for x,y in vertices]) for name,kind,vertices in areas]
        start=(9-start[1],start[0],math.pi/2);goal=(9-goal[1],goal[0],-math.pi/2)
    Image.fromarray(cells).save(root/'map.pgm')
    (root/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[0.,0.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
    features=[SemanticFeature(n,k,'polygon',v+[v[0]],soft=k=='lane') for n,k,v in areas]
    json_write(root/'semantic.json',SemanticMapV1('map',.05,(0.,0.,0.),cells.shape[1],cells.shape[0],'return_bulb',features).to_dict())
    return root/'map.yaml',root/'semantic.json',[{'query_id':'return_bulb_rotated' if rotated else 'return_bulb','start':start,'goal':goal,'seed':20260914}]

def two_routes_inputs(root):
    root=Path(root);root.mkdir(parents=True)
    cells=np.zeros((280,480),np.uint8)
    def fill(x0,x1,y0,y1,value):cells[280-int(y1/.05):280-int(y0/.05),int(x0/.05):int(x1/.05)]=value
    fill(1.,23.,1.,13.,255);fill(9.,15.,4.,12.,0)
    # Upper passage is just wide enough for circular center clearance.
    areas=[('left','parking_area',[[1,1],[9,1],[9,13],[1,13]]),('upper','lane',[[9,12],[15,12],[15,13],[9,13]]),('lower','lane',[[9,1],[15,1],[15,4],[9,4]]),('right','parking_area',[[15,1],[23,1],[23,13],[15,13]])]
    Image.fromarray(cells).save(root/'map.pgm');(root/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[0.,0.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
    features=[SemanticFeature(n,k,'polygon',v+[v[0]],soft=k=='lane') for n,k,v in areas]
    json_write(root/'semantic.json',SemanticMapV1('map',.05,(0.,0.,0.),480,280,'two_routes',features).to_dict())
    return root/'map.yaml',root/'semantic.json',[{'query_id':'two_routes','start':(7.025,10.525,0.),'goal':(17.025,10.525,0.),'seed':20260914}]

def inject_optimistic_edge(world,topology,query):
    """Explicit fault injection, not a naturally generated map benchmark.

An optimistic L1 candidate claims two disconnected regions are adjacent.
Fine validation must reject it, and recovery must use a real alternative.
"""
    from arena_3d_v1.semantic_graph import InterfacePose
    from arena_evaluation.semantic_map import canonical_hash
    start=world.map.world_to_cell(*query['start'][:2]);goal=world.map.world_to_cell(*query['goal'][:2])
    a,b=int(topology.subregions[start]),int(topology.subregions[goal]);portal=topology.portal_count+1;nodes=[]
    for fr,to,yaw in [(a,b,0.),(b,a,math.pi)]:
        n=InterfacePose(max(topology.nodes)+1,(12.,10.525,yaw),fr,to,topology.parent_semantics[fr],topology.parent_semantics[to],portal)
        topology.nodes[n.id]=n;topology.region_nodes[fr]['out'].append(n.id);topology.region_nodes[to]['in'].append(n.id);nodes.append(n.__dict__)
    topology.key=canonical_hash([topology.key,'explicit_optimistic_edge_fault_injection',nodes]);topology.query_cache.clear()
    return {'type':'explicit_candidate_graph_fault_injection','natural_map_generation':False,'nodes':nodes,'topology_key':topology.key,'expected':'reject false first candidate in fine geometry; use upper or lower physical route; do not modify static obstacles'}
