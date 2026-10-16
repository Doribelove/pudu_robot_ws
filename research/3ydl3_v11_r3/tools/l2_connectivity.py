from pathlib import Path
import numpy as np,cv2
from scipy import ndimage
from three_yd_v5.controller import inflate_circle
from arena_3d_v1.multires_grid import aggregate
root=Path(__file__).resolve().parents[1];d=np.load(root/'results/inspect/mentor_map_20260825_005_A2B-03/rank1_scope.npz');hard=d['hard'];scope=d['mask'];p=d['start'];q=d['goal'];res=.05
safe=aggregate(~inflate_circle(hard,np.hypot(.265,.225),res),3,all_free=True,pad_value=False)
# Map origin for this registered mentor map is zero.
h,w=hard.shape
cell=lambda xy:(int((h-1-int(xy[1]/res))//3),int(xy[0]/res)//3)
a,b=cell(p),cell(q);print('cells',a,b,'circle_safe',safe[a],safe[b], 'poses',p,q,flush=True)
for extra in [0.,.5,1.,2.,3.,4.,6.]:
 r=int(np.ceil(extra/res));ss=scope if r==0 else cv2.dilate(scope.astype(np.uint8),cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*r+1,2*r+1)))>0
 mask=aggregate(ss,3,all_free=False,pad_value=False);lab,n=ndimage.label(safe&mask,np.ones((3,3),int));print(extra,'connected',lab[a]>0 and lab[a]==lab[b],'components',n,'cells',int(mask.sum()),flush=True)
