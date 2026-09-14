"""Independent dense footprint SAT audit, against original occupied/unknown cells."""
import math
import numpy as np


def audit_footprint(rows,free,config,half_length=.265,half_width=.225):
    if not rows:return {'valid':None,'status':'NO_PATH','poses_checked':0}
    res=float(config['resolution']);ox,oy,_=config['origin'];h,w=free.shape
    collisions=[]
    for i,p in enumerate(rows):
        x,y,yaw=p['x'],p['y'],p['yaw'];c=math.cos(yaw);s=math.sin(yaw)
        rx=half_length*abs(c)+half_width*abs(s)
        ry=half_length*abs(s)+half_width*abs(c)
        c0=math.floor((x-rx-ox)/res);c1=math.floor((x+rx-ox)/res)
        r0=h-1-math.floor((y+ry-oy)/res);r1=h-1-math.floor((y-ry-oy)/res)
        if min(c0,r0)<0 or c1>=w or r1>=h:
            collisions.append(i);continue
        rr,cc=np.nonzero(~free[r0:r1+1,c0:c1+1])
        if len(rr)==0:continue
        dx=ox+(cc+c0+.5)*res-x;dy=oy+(h-(rr+r0)-.5)*res-y
        # Four separating axes between oriented rectangle and occupied cell.
        projected_half=res*.5*(abs(c)+abs(s))
        hit=(np.abs(c*dx+s*dy)<=half_length+projected_half)&(np.abs(-s*dx+c*dy)<=half_width+projected_half)
        hit&=(np.abs(dx)<=rx+res*.5)&(np.abs(dy)<=ry+res*.5)
        if np.any(hit):collisions.append(i)
    xy=np.array([[p['x'],p['y']] for p in rows]);yaw=np.unwrap([p['yaw'] for p in rows])
    return {'valid':not collisions,'status':'PASS' if not collisions else 'FOOTPRINT_COLLISION',
            'poses_checked':len(rows),'collision_pose_indices':collisions,
            'max_translation_step_m':float(np.max(np.linalg.norm(np.diff(xy,axis=0),axis=1),initial=0)),
            'max_yaw_step_deg':float(np.max(np.abs(np.diff(yaw)),initial=0)*180/math.pi),
            'unknown_is_collision':True,'method':'oriented_rectangle_vs_occupied_pixel_SAT',
            'continuous_collision_argument':'separate conservative disk-inflated convex hull containment'}
