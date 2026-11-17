from pathlib import Path
from dataclasses import replace
import tempfile,os,json
from PIL import Image
from trilayer_p6_runtime.api import MapOnlyRequest,validate_request
os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']='179'
root=Path(tempfile.mkdtemp(prefix='trilayer-api-validation-'));mp=root/'map.yaml'
Image.new('L',(240,240),254).save(root/'map.pgm')
mp.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
r=MapOnlyRequest(mp,(1.,2.,0.),(8.,9.,0.),root/'new')
assert validate_request(r)==(mp,root/'new')
for bad in [replace(r,start=(float('nan'),1.,0.)),replace(r,goal=(-1.,2.,0.)),replace(r,vehicle=(1.,1.,1.)),replace(r,output_dir=root),replace(r,master_service='/other'),replace(r,map_yaml=Path('relative.yaml'))]:
 try:validate_request(bad)
 except (ValueError,TypeError):pass
 else:raise AssertionError('Invalid input accepted: '+str(bad))
mp.write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n')
try:validate_request(r)
except ValueError:pass
else:raise AssertionError('Unsupported resolution accepted')
assert not (root/'new').exists()
print('PASS supported input and 7 invalid input rejections before output/process creation')
import yaml
base={'image':'map.pgm','resolution':.05,'origin':[0,0,0],'negate':0,'free_thresh':.196,'occupied_thresh':.65}
invalid=[{'origin':['0','0',0]},{'origin':[False,0,0]},{'origin':[float('inf'),0,0]},
 {'resolution':'0.05'},{'resolution':True},{'resolution':float('nan')},
 {'negate':'0'},{'negate':False},{'negate':True},{'negate':0.0},{'negate':2},
 {'free_thresh':'0.1'},{'free_thresh':False},{'free_thresh':float('nan')},
 {'occupied_thresh':float('inf')},{'occupied_thresh':'0.65'},{'occupied_thresh':True},
 {'free_thresh':-.1},{'occupied_thresh':1.1},{'free_thresh':.7},{'free_thresh':.65},
 {'image':None},{'image':1},{'image':''},{'image':'\x00'},{'image':'missing.pgm'}]
(root/'corrupt.pgm').write_bytes(b'P5\n240 240\n255\nshort')
invalid.append({'image':'corrupt.pgm'})
for override in invalid:
 mp.write_text(yaml.safe_dump({**base,**override}))
 try:validate_request(r)
 except ValueError as e:assert str(e)
 else:raise AssertionError('Invalid map accepted: '+str(override))
 assert not (root/'new').exists()
for override in [{'negate':1},{'free_thresh':0.,'occupied_thresh':1.}]:
 mp.write_text(yaml.safe_dump({**base,**override}));assert validate_request(r)==(mp,root/'new')
print('PASS',len(invalid),'strict map type/threshold/image rejections and 2 valid boundary configurations')
