"""Held-out geometry/pose variations, fixed before their first execution."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import argparse,sys,math
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import static_benchmark,recovery_scenes
from arena_3d_v1.semantic_world import json_write
from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature
from arena_3d_v1.stable_session import StableSmacSession
from arena_3d_v1.curvature_repair import TurnaroundRecovery

CASES={
 'shifted':{'start':(8.,3.475,0.),'goal':(6.,3.475,math.pi),'room':(18.,1.,7.)},
 'yaw':{'start':(9.,3.475,.035),'goal':(7.,3.475,-math.pi+.035),'room':(18.,1.,7.)},
 'smaller_room':{'start':(10.,3.475,0.),'goal':(8.,3.475,math.pi),'room':(17.,1.5,6.5)},
 'larger_room':{'start':(8.5,3.475,0.),'goal':(6.5,3.475,math.pi),'room':(19.,.5,7.5)},
 'sealed_offset':{'start':(9.75,3.475,-.02),'goal':(7.25,3.475,math.pi-.02),'room':(17.5,1.25,6.75)},
 'sealed_near_goal':{'start':(10.5,3.475,0.),'goal':(9.,3.475,math.pi),'room':(18.25,.75,7.25)},
}

def inputs(root,case,rotated):
 root=Path(root);root.mkdir(parents=True);v=CASES[case];right,bottom,top=v['room']
 cells=np.zeros((180,400),np.uint8)
 def fill(x0,x1,y0,y1):cells[180-round(y1/.05):180-round(y0/.05),round(x0/.05):round(x1/.05)]=255
 fill(1.,13.,3.,3.95);fill(13.,right,bottom,top)
 areas=[('lane','lane',[[1,3],[11,3],[11,3.95],[1,3.95]]),('bulb','parking_area',[[11,3],[13,3],[13,bottom],[right,bottom],[right,top],[13,top],[13,3.95],[11,3.95]])]
 start,goal=v['start'],v['goal']
 if rotated:
  cells=np.rot90(cells);areas=[(name,kind,[[9-y,x] for x,y in points]) for name,kind,points in areas]
  transform=lambda p:(9-p[1],p[0],(p[2]+math.pi/2+math.pi)%(2*math.pi)-math.pi)
  start,goal=transform(start),transform(goal)
 Image.fromarray(cells).save(root/'map.pgm')
 (root/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[0.,0.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
 features=[SemanticFeature(name,kind,'polygon',points+[points[0]],soft=kind=='lane') for name,kind,points in areas]
 json_write(root/'semantic.json',SemanticMapV1('map',.05,(0.,0.,0.),cells.shape[1],cells.shape[0],'turnaround_holdout',features).to_dict())
 json_write(root/'holdout_case.json',{'case':case,'rotated':rotated,'definition':v})
 return root/'map.yaml',root/'semantic.json',[{'query_id':case+('_rotated' if rotated else ''),'start':start,'goal':goal,'seed':20260914}]

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--case',choices=list(CASES),required=True);p.add_argument('--rotated',action='store_true');p.add_argument('--baseline',action='store_true')
 args,remaining=p.parse_known_args()
 recovery_scenes.return_bulb_inputs=lambda root,rotated=False:inputs(root,args.case,args.rotated)
 static_benchmark.ReferenceSmacSession=StableSmacSession
 if not args.baseline:static_benchmark.CandidateRecovery=TurnaroundRecovery
 sys.argv=[sys.argv[0],*remaining,'--arm','recovery','--scene','return_bulb']
 static_benchmark.main()
