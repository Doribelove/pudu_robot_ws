"""No ROS launches: strict input rejection and retained-byte snapshot ownership."""
from pathlib import Path
from dataclasses import replace
import os,tempfile,json,hashlib,copy
from PIL import Image
from trilayer_p6_runtime.api import SemanticRequest,validate_semantic_request
from trilayer_p6_runtime.semantic_inputs import snapshot_inputs
os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']='179'
root=Path(tempfile.mkdtemp(prefix='trilayer-semantic-validation-'))
Image.new('L',(240,240),254).save(root/'map.pgm')
(root/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
base={'schema_version':'SemanticMapV1','frame_id':'map','resolution':.05,'origin':[0,0,0],'width':240,'height':240,'source_pdmap_hash':hashlib.sha256((root/'map.pgm').read_bytes()).hexdigest(),'features':[{'semantic_id':'road','semantic_class':'lane','geometry_type':'polygon','coordinates':[[1,4],[11,4],[11,8],[1,8]],'properties':{'source_name':'road'}},{'semantic_id':'park','semantic_class':'parking_area','geometry_type':'polygon','coordinates':[[1,1],[4,1],[4,4],[1,4]],'properties':{'source_name':'park'}}],'traffic_rules':{}}
top={'regions':[{'id':1,'name':'road','kind':'lane'},{'id':2,'name':'park','kind':'parking_area'}],'edges':[]}
parking={'bindings':[]}
r=SemanticRequest(map_yaml=root/'map.yaml',start=(2.,6.,0.),goal=(9.,6.,0.),output_dir=root/'output',query_id='q',semantic_json=root/'semantic.json',topology_json=root/'topology.json',parking_access_json=root/'parking_access.json')
def save(s=base,t=top,p=parking):
 for name,obj in [('semantic.json',s),('topology.json',t),('parking_access.json',p)]: (root/name).write_text(json.dumps(obj))
save();blobs,cfg=validate_semantic_request(r)
# Files are retained by value: later edits/deletion cannot change the execution snapshot.
(root/'semantic.json').write_text('{}');(root/'map.pgm').unlink()
out=root/'snapshot';out.mkdir();folder,manifest=snapshot_inputs(out,blobs,cfg)
assert (folder/'semantic.json').read_bytes()==blobs['semantic.json']
assert hashlib.sha256((folder/'map_image').read_bytes()).hexdigest()==manifest['map_image']['source_sha256']
assert not r.output_dir.exists()
Image.new('L',(240,240),254).save(root/'map.pgm')
mutations=[lambda s,t,p:s.update(unknown=1),lambda s,t,p:s.update(frame_id='odom'),lambda s,t,p:s.update(width=True),lambda s,t,p:s.update(origin=['0',0,0]),lambda s,t,p:s.update(source_pdmap_hash='wrong'),lambda s,t,p:s.update(semantic_map_hash='wrong'),lambda s,t,p:s.update(traffic_rules={'allowed_u_turn_junctions':[2]}),lambda s,t,p:s.update(traffic_rules={'one_way':{}}),lambda s,t,p:s.update(traffic_rules={'one_way':[{'semantic_id':'missing','yaw':0}]}),lambda s,t,p:s.update(traffic_rules={'one_way':[{'semantic_id':'road','yaw':True}]}),lambda s,t,p:s.update(traffic_rules={'one_way':[{'semantic_id':'road','yaw':1}]}),lambda s,t,p:s.update(traffic_rules={'forbidden_transitions':[['road','park','road']]}),lambda s,t,p:s['features'][0].update(hard=True),lambda s,t,p:s['features'][0]['properties'].update(unknown=True),lambda s,t,p:s['features'][0].update(coordinates=[[1,4],[11,8],[11,4],[1,8]]),lambda s,t,p:s['features'][1].update(semantic_id='road'),lambda s,t,p:s['features'][1].update(coordinates=[[1,5],[4,5],[4,7],[1,7]]),lambda s,t,p:t['regions'][1].update(id=1),lambda s,t,p:t['regions'][1].update(name='missing'),lambda s,t,p:t.update(edges=[{'kind':'shared_outline','regions':[1,99],'geometry':[[1,4],[4,4]]}]),lambda s,t,p:t.update(edges=[{'kind':'shared_outline','regions':[1,2],'geometry':[[1,5],[4,5]]}]),lambda s,t,p:p.update(extra=True),lambda s,t,p:p.update(bindings=[{'unknown':1}])]
for mutate in mutations:
 s,t,p=copy.deepcopy((base,top,parking));mutate(s,t,p);save(s,t,p)
 try:validate_semantic_request(r)
 except ValueError as e:assert str(e)
 else:raise AssertionError('invalid semantic input accepted')
 assert not r.output_dir.exists()
save();(root/'semantic.json').write_text('{"schema_version":"SemanticMapV1","schema_version":"bad"}')
try:validate_semantic_request(r)
except ValueError:pass
else:raise AssertionError('duplicate key accepted')
save();(root/'parking_access.json').write_text('{broken')
try:validate_semantic_request(r)
except ValueError:pass
else:raise AssertionError('bad JSON accepted')
assert not r.output_dir.exists()
print('PASS',len(mutations)+2,'invalid semantic inputs, unchanged originals/output and immutable-byte snapshot')
