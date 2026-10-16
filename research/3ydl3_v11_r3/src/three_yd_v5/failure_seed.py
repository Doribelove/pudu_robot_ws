"""Coupled short bends as untrusted initialization, original audits retained."""
import math
import numpy as np

def coupled_bends(p,s,world,hard,deadline):
    from .curve_l3 import check_deadline
    from .v10_storage import collision_checker
    from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
    from arena_3d_v1.semantic_world import densify
    checker=collision_checker(hard,world);output=[p[0]];records=[];i=1
    while i<len(p)-2:
        check_deadline(deadline)
        a=p[i]-p[i-1];bridge=p[i+1]-p[i];b=p[i+2]-p[i+1]
        la,lb,lc=map(np.linalg.norm,(a,bridge,b));chosen=None
        if min(la,lb,lc)>.01:
            u=a/la;v=b/lc;middle=bridge/lb
            turn1=math.atan2(np.cross(u,middle),u@middle)
            turn2=math.atan2(np.cross(middle,v),middle@v)
            eligible=(lb<2. and turn1*turn2<0 and min(abs(turn1),abs(turn2))>math.radians(35)
                      and abs(turn1+turn2)<math.radians(25))
            if eligible:
                for trim in (1.,1.5,2.,3.):
                    before=min(trim,la*.4);after=min(trim,lc*.4)
                    start=tuple(p[i]-before*u)+(math.atan2(u[1],u[0]),)
                    goal=tuple(p[i+1]+after*v)+(math.atan2(v[1],v[0]),)
                    for radius in (1.,.8,.6,.5):
                        for word,params in dubins_choices(start,goal,radius):
                            check_deadline(deadline)
                            length=radius*sum(params)
                            if length>before+lb+after+1. or (chosen and length>=chosen[0]):continue
                            edge=dubins_edge_from_parameters(start,goal,radius,word,params)
                            poses=densify(np.vstack((start,edge.samples)))
                            if checker.check(poses):chosen=(length,poses,word,radius,before,after)
                    if chosen:break
        if chosen:
            length,poses,word,radius,before,after=chosen
            output.extend(poses[:,:2]);records.append({'indices':[i,i+1],'word':word,'radius_m':radius,'trim_m':[before,after],'length_m':length})
            i+=2
        else:output.append(p[i]);i+=1
    output.extend(p[i:]);output=np.asarray(output)
    output=output[np.r_[True,np.linalg.norm(np.diff(output,axis=0),axis=1)>1e-7]]
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(output,axis=0),axis=1))]
    return output,arc,{'selected_pairs':records,'original_scope_and_footprint_checked':True,'initialization_only':True}
