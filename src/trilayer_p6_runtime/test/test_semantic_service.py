#!/usr/bin/env python3
"""Experimental real semantic-P6 integration; not a supported public scene API."""
import os,sys,tempfile,json,time,hashlib,traceback,math
from pathlib import Path
assert os.environ['ROS_LOCALHOST_ONLY']=='1' and os.environ['ROS_DOMAIN_ID']=='179'
case=sys.argv[2] if len(sys.argv)>2 else 'baseline'
assert case in ('baseline','one_way_forward','one_way_reverse','forbidden_transition','parking_valid','parking_missing','parking_anchor_mismatch','parking_wrong_side')
sys.path.insert(0,sys.argv[1])
import osqp,qdldl,clarabel
solver_sources={n:str(Path(m.__file__).resolve()) for n,m in [('osqp',osqp),('qdldl',qdldl),('clarabel',clarabel)]}
assert all(Path(x).is_relative_to(Path(sys.argv[1]).resolve()) for x in solver_sources.values())
print('SOLVER_SOURCES',json.dumps(solver_sources),flush=True)
from trilayer_p6_runtime.api import SemanticRequest,plan_semantic
out=Path(sys.argv[3]) if len(sys.argv)>3 else Path(tempfile.mkdtemp(prefix='trilayer-p6-semantic-'))
if len(sys.argv)>3:out.mkdir(parents=True,exist_ok=False)
import numpy as np
from PIL import Image
pixels=np.full((400,400),254,np.uint8);pixels[:4,:]=0;pixels[-4:,:]=0;pixels[:,:4]=0;pixels[:,-4:]=0
Image.fromarray(pixels).save(out/'map.pgm');mp=out/'map.yaml';mp.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
regions=[{'id':i,'name':name,'kind':kind} for i,name,kind in [(1,'west','lane'),(2,'junction','junction_area'),(3,'east','lane')]]
boxes=[(1,8,8,12),(8,8,12,12),(12,8,19,12)]
if case.startswith('parking_'):
 regions.append({'id':4,'name':'parking','kind':'parking_area'});boxes.append((1,4,5,8))
features=[]
for r,(x0,y0,x1,y1) in zip(regions,boxes):
 features.append({'semantic_id':r['name'],'semantic_class':r['kind'],'geometry_type':'polygon','properties':{'source_name':r['name']},'coordinates':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]})
semantic={'schema_version':'SemanticMapV1','frame_id':'map','resolution':.05,'origin':[0,0,0],'width':400,'height':400,'source_pdmap_hash':hashlib.sha256((out/'map.pgm').read_bytes()).hexdigest(),'features':features,'traffic_rules':{}}
if case.startswith('one_way_'):semantic['traffic_rules']={'one_way':[{'semantic_id':'west','yaw':0.}]}
if case=='forbidden_transition':semantic['traffic_rules']={'forbidden_transitions':[['west','junction','east']]}
(out/'semantic.json').write_text(json.dumps(semantic))
(out/'topology.json').write_text(json.dumps({'regions':regions,'edges':[{'kind':'shared_outline','regions':[1,2],'geometry':[[8,8],[8,12]]},{'kind':'shared_outline','regions':[2,3],'geometry':[[12,8],[12,12]]}]}))
bindings=[]
if case.startswith('parking_') and case!='parking_missing':
 bindings=[{'query_id':case,'endpoint':'start','parking_region_id':4,'road_id':1,'anchor_xy':[3.,6.],'anchor_tolerance_m':.25,'travel_sides':['top' if case=='parking_wrong_side' else 'bottom'],'allow_local_cross_road':True}]
(out/'parking_access.json').write_text(json.dumps({'bindings':bindings}))
start,goal=(3.,10.,0.),(17.,10.,0.)
if case=='one_way_reverse':start,goal=(17.,10.,math.pi),(3.,10.,math.pi)
if case.startswith('parking_'):start=(4. if case=='parking_anchor_mismatch' else 3.,6.,math.pi/2)

req=SemanticRequest(map_yaml=mp,start=start,goal=goal,output_dir=out/'runtime',query_id=case,seed=20260916,semantic_json=out/'semantic.json',topology_json=out/'topology.json',parking_access_json=out/'parking_access.json')
source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [mp,out/'map.pgm',req.semantic_json,req.topology_json,req.parking_access_json]}
data=plan_semantic(req,solver_root=Path(sys.argv[1]))
result=data['result'];failure=data['exception'];errors=data['cleanup_errors']
if data['path']:result['path']=data['path']
data.update(case=case,input_rules=semantic['traffic_rules'],input_parking_bindings=bindings,input_start=start,input_goal=goal)
(out/'result.json').write_text(json.dumps(data,indent=2))
print('SEMANTIC_RESULT',case,out,'success',result.get('success'),'failure',result.get('failure_code'),'worker_exit',data['worker_exit'],flush=True)
for p in [mp,out/'map.pgm',req.semantic_json,req.topology_json,req.parking_access_json]:assert hashlib.sha256(p.read_bytes()).hexdigest()==source_hashes[p.name]
assert data['mode']=='semantic' and data['input_snapshot']
for name,record in data['input_snapshot'].items():assert hashlib.sha256((out/'runtime/inputs'/name).read_bytes()).hexdigest()==record['snapshot_sha256']
assert not errors and data['worker_exit']==0
negative={'one_way_reverse':'NO_LEGAL_SEMANTIC_ROUTE',
          'forbidden_transition':'NO_LEGAL_SEMANTIC_ROUTE',
          'parking_missing':'PARKING_OWNERSHIP_UNRESOLVED',
          'parking_anchor_mismatch':'PARKING_BINDING_POSITION_MISMATCH',
          'parking_wrong_side':'NO_LEGAL_SEMANTIC_ROUTE'}
if case in negative:
 assert not result.get('success') and not result.get('path')
 assert negative[case] in ((failure or '')+json.dumps(result)), (case,failure,result)
 expected_final={'one_way_reverse':'WRONG_TRAVEL_DIRECTION','forbidden_transition':"V12_FULL_L2_TRAFFIC_REJECT:['FORBIDDEN_TURN']",'parking_missing':'PARKING_OWNERSHIP_UNRESOLVED','parking_anchor_mismatch':'PARKING_BINDING_POSITION_MISMATCH','parking_wrong_side':'PARKING_ENDPOINT_TRAVEL_SIDE'}
 assert failure is None and result.get('failure_code')==expected_final[case], result.get('failure_code')
 assert any(a.get('costmap_ack_status')=='cold_exact_content_verified' for a in data['ack_events'])
 print('PASS expected semantic refusal',negative[case],flush=True)
 raise SystemExit(0)
assert failure is None
assert result.get('success') and result.get('independent_audit',{}).get('valid')
assert result['constraint_audit']['traffic']['valid']
assert result['constraint_audit']['traffic']['visited_roads']==[1,3]
assert result['candidate_recovery']['attempts'][0]['regions']==([4,1,2,3] if case=='parking_valid' else [1,2,3])
assert result['curve_l3']['iterations'] and result['curve_l3']['qp_solver']=='clarabel'
assert any(a.get('costmap_ack_status')=='exact_effective_master_content_verified' and a.get('costmap_ack_mismatch_cells')==0 and a.get('semantic_exact_stable_observations',0)>=2 for a in data['ack_events'])
