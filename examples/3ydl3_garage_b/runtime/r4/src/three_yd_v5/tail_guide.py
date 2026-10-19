"""Locally rounded XY guide initializations, checked in the original scope."""
import math
import numpy as np


def rounded_guide(p,s,world,hard,deadline):
    from .v10_storage import collision_checker
    from .curve_l3 import check_deadline
    checker=collision_checker(hard,world)
    output=[p[0]];accepted=[]
    for i in range(1,len(p)-1):
        check_deadline(deadline)
        incoming=p[i]-p[i-1];outgoing=p[i+1]-p[i]
        before=np.linalg.norm(incoming);after=np.linalg.norm(outgoing)
        if min(before,after)<1e-5:output.append(p[i]);continue
        u=incoming/before;v=outgoing/after
        angle=math.atan2(u[0]*v[1]-u[1]*v[0],np.dot(u,v))
        if not math.radians(12)<=abs(angle)<=math.radians(150):output.append(p[i]);continue
        turn=math.copysign(1.,angle);best=None
        for scale in (2.5,2.,1.5,1.25):
            radius=world.vehicle[2]*scale
            trim=radius*math.tan(abs(angle)/2)
            if trim>.45*min(before,after):continue
            entry=p[i]-trim*u;exit=p[i]+trim*v
            center=entry+turn*radius*np.array([-u[1],u[0]])
            a=math.atan2(*(entry-center)[::-1])
            theta=np.linspace(a,a+angle,max(2,int(math.ceil(radius*abs(angle)/.025))+1))
            xy=center+radius*np.c_[np.cos(theta),np.sin(theta)]
            xy[0]=entry;xy[-1]=exit
            poses=np.c_[xy,np.arctan2(np.sin(theta+turn*math.pi/2),np.cos(theta+turn*math.pi/2))]
            if checker.check(poses):best=(xy,radius);break
        if best is None:output.append(p[i]);continue
        output.extend(best[0]);accepted.append({'index':i,'radius_m':best[1],'turn_rad':angle})
    output.append(p[-1]);output=np.asarray(output)
    output=output[np.r_[True,np.linalg.norm(np.diff(output,axis=0),axis=1)>1e-7]]
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(output,axis=0),axis=1))]
    return output,arc,{'accepted_corners':accepted,'original_points':len(p),'seed_points':len(output),
                       'original_scope_and_footprint_checked':True,'initialization_only':True}
