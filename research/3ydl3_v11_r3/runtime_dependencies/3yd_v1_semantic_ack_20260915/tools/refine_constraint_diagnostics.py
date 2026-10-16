"""Additional positive-length region traversal and certified body-margin checks."""
import json,math,os,hashlib
from pathlib import Path
import numpy as np,yaml
from arena_3d_v1.semantic_world import WORK,SemanticWorld,SweptChecker,densify,json_write
R=WORK/'report/constraints_20260915_01';os.sched_setaffinity(0,{12})
cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());world=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=R/'world_cache')
p=next((WORK/'results/final_1_baseline/preparation_cache/graph').rglob('graph.json'));graph=json.loads(p.read_text())
with np.load(p.with_name('regions.npz')) as z:regions=z['subregions']
allrows=json.loads((R/'rows.json').read_text());rows=[r for r in allrows if r['batch']=='final_1_baseline' and r['repetition']==1]
def collapse(v):return [x for i,x in enumerate(v) if i==0 or x!=v[i-1]]
def subsequence(needed,seen):
 index=0
 for value in seen:
  if index<len(needed) and value==needed[index]:index+=1
 return index==len(needed)
def crossed_regions(poses):
 coords=(poses[:,:2]-np.array(world.map.origin[:2]))/world.map.resolution;v=[];sem=[]
 for a,b in zip(coords,coords[1:]):
  d=b-a;ts=[0.,1.]
  for axis in range(2):
   if abs(d[axis])<1e-12:continue
   for line in range(math.ceil(min(a[axis],b[axis])),math.floor(max(a[axis],b[axis]))+1):
    t=(line-a[axis])/d[axis]
    if 1e-12<t<1-1e-12:ts.append(float(t))
  ts=sorted(set(ts));mid=(np.array(ts[:-1])+ts[1:])/2;xy=a+mid[:,None]*d
  c=np.floor(xy[:,0]).astype(int);r=world.map.height-1-np.floor(xy[:,1]).astype(int)
  v.extend(map(int,regions[r,c]));sem.extend(map(int,world.labels[r,c]))
 return collapse(v),collapse(sem)
result=[]
for row in rows:
 poses=np.load(Path(row['case'])/'path.npy');dense=densify(poses);req=row['ordered_regions']['required'];visits,visited_semantic=crossed_regions(poses)
 lo=0.;hi=2.;checks=[]
 def safe(margin):
  ok=SweptChecker(world.hard,world.map.resolution,world.map.origin,world.vehicle[0]+margin,world.vehicle[1]+margin).check(dense)
  checks.append({'margin_m':margin,'certified_free':ok});return ok
 assert safe(0.)
 if safe(hi):lo=hi
 else:
  while hi-lo>.002:
   mid=(lo+hi)/2
   if safe(mid):lo=mid
   else:hi=mid
 parent=lambda seq:collapse([graph['parents'][str(x)] for x in seq])
 required_semantic=parent(req)
 name=lambda sem:world.features[sem].properties.get('source_name',world.features[sem].semantic_id) if sem in world.features else '未标注语义'
 r={'query':row['query'],'grid_crossing_region_visits':visits,'required_regions':req,
  'ordered_regions_verified':subsequence(req,visits),'matches_sampled_subsequence_verdict':subsequence(req,visits)==row['ordered_regions']['as_subsequence'],
  'required_semantic_sequence':[name(s) for s in required_semantic],'visited_semantic_sequence':[name(s) for s in visited_semantic],
  'ordered_semantics_verified':subsequence(required_semantic,visited_semantic),
  'certified_body_clearance_lower_bound_m':lo,'test_limit_m':2.,'binary_resolution_m':.002,'inflation_checks':checks}
 result.append(r);print(row['query'],'region',r['ordered_regions_verified'],'semantic',r['ordered_semantics_verified'],'margin',round(lo,4),flush=True)
json_write(R/'refinements.json',{'rows':result,'method':{'regions':'Enumerate every grid line crossed by each saved XY segment and sample the interior of each resulting interval; checks positive-length cell visits without a fixed sampling step. Same required physical portal chain as logged.',
 'clearance':'Expand the oriented half-length and half-width equally by margin, then run the original conservative continuous swept checker. This rectangle contains the body Minkowski-summed with a disk of that radius. A passed margin certifies a distance lower bound under the saved interpolation, not exact clearance; failed larger envelopes are not exact upper bounds. Search 0..2 m, interval <=2 mm.',
 'scope':'20 unique saved paths; all 12 paths per query were verified identical across arms; no new trajectory or planner changes.'}})
print('grid/sampling verdict differences',sum(not r['matches_sampled_subsequence_verdict'] for r in result),flush=True)
