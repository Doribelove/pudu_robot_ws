import sys,json
from pathlib import Path
import numpy as np
from scipy import ndimage
E=Path(__file__).resolve().parents[1];out=E/'results/bridge_geometry';out.mkdir(exist_ok=False)
from three_yd_v4.r5_controller import BoundedReferenceDStar
original=BoundedReferenceDStar.initialize
saved=[]
def initialize(self,*a,**kw):
 result=original(self,*a,**kw);roi=self.roi;z={'bbox':roi.bbox,'shape':roi.shape,'start':roi.start_local,'goal':roi.goal_local,'success':result.success}
 for n,st in [('4',None),('8',np.ones((3,3)))]:
  label,num=ndimage.label(roi.base_free,st);z['components'+n]=int(num);z['start_component'+n]=int(label[roi.start_local]);z['goal_component'+n]=int(label[roi.goal_local])
 np.savez_compressed(out/f'solve{len(saved)}.npz',free=roi.base_free,labels4=ndimage.label(roi.base_free)[0],bbox=roi.bbox,start=roi.start_local,goal=roi.goal_local,neighbor=self.geometry.neighbor_deltas,linear=self.geometry.state_cells_linear,weights=self.state.edge_weights)
 saved.append(z);print(z,flush=True);return result
BoundedReferenceDStar.initialize=initialize
sys.argv=['tools/inspect_case.py','mentor_map_20260825_005','A2B-03']
s=(E/'tools/inspect_case.py').read_text().replace("ROOT/'results/inspect'","ROOT/'results/bridge_geometry/inspect'")
exec(compile(s,str(E/'tools/inspect_case.py'),'exec'),{'__file__':str(E/'tools/inspect_case.py'),'__name__':'__main__'})
(out/'summary.json').write_text(json.dumps(saved,indent=2))
