"""Request-local clearance-aware guide seeds within the existing L3 mask."""
import math
import cv2
import numpy as np
from scipy.spatial import cKDTree


def clearance_guide(guide,start,goal,world,hard,deadline,max_shift=.3,smoothing_m=0.,stringpull=False):
    from .controller import inflate_circle
    from .curve_l3 import check_deadline,clean_guide,interp
    check_deadline(deadline)
    p,s=clean_guide(guide,start,goal,0.)
    u=np.unique(np.r_[np.arange(0,s[-1],.1),s,s[-1]])
    points=interp(p,s,u);res=world.map.resolution;ox,oy=world.map.origin[:2];h,w=hard.shape
    safe=~inflate_circle(hard,math.hypot(*world.vehicle[:2]),res)
    rr,cc=np.nonzero(safe)
    if not len(rr):return guide,{'used':False,'reason':'no_circle_clear_centers'}
    centers=np.c_[ox+(cc+.5)*res,oy+(h-rr-.5)*res]
    r=h-1-np.floor((points[:,1]-oy)/res).astype(int);c=np.floor((points[:,0]-ox)/res).astype(int)
    valid=(r>=0)&(c>=0)&(r<h)&(c<w);clear=np.zeros(len(points),bool);clear[valid]=safe[r[valid],c[valid]]
    selected=np.flatnonzero(~clear & (u>.5)&(u<s[-1]-.5))
    if not len(selected):return guide,{'used':False,'reason':'guide_already_clear'}
    distances,indices=cKDTree(centers).query(points[selected],distance_upper_bound=max_shift)
    take=np.isfinite(distances)
    points[selected[take]]=centers[indices[take]]
    if smoothing_m>0:
        from scipy.ndimage import gaussian_filter1d
        # Uniform arc samples avoid overweighting original grid vertices.
        uu=np.r_[np.arange(0,s[-1],.1),s[-1]]
        points=np.c_[np.interp(uu,u,points[:,0]),np.interp(uu,u,points[:,1])]
        tree=cKDTree(centers)
        for _ in range(3):
            check_deadline(deadline)
            smooth=gaussian_filter1d(points,smoothing_m/.1,axis=0,mode='nearest')
            blend=np.minimum(np.clip(uu/2.,0,1),np.clip((s[-1]-uu)/2.,0,1))
            smooth=points+(smooth-points)*blend[:,None]
            rr2=h-1-np.floor((smooth[:,1]-oy)/res).astype(int);cc2=np.floor((smooth[:,0]-ox)/res).astype(int)
            valid2=(rr2>=0)&(cc2>=0)&(rr2<h)&(cc2<w)
            ok=np.zeros(len(smooth),bool);ok[valid2]=safe[rr2[valid2],cc2[valid2]]
            ix=np.flatnonzero(~ok & (uu>.5)&(uu<s[-1]-.5))
            d,j=tree.query(smooth[ix],distance_upper_bound=max_shift)
            choose=np.isfinite(d);smooth[ix[choose]]=centers[j[choose]]
            points=smooth
        points[0]=start[:2];points[-1]=goal[:2]
    if stringpull:
        def simplify(left,right):
            if right<=left+1:return [left,right]
            check_deadline(deadline)
            a,b=points[left],points[right];delta=b-a;length=float(np.linalg.norm(delta))
            t=np.clip((points[left:right+1]-a)@delta/max(length*length,1e-12),0,1)
            deviation=np.linalg.norm(points[left:right+1]-(a+t[:,None]*delta),axis=1)
            if deviation.max()<=.22:
                line=a+np.linspace(0,1,max(2,int(np.ceil(length/(res*.25)))+1))[:,None]*delta
                cr=np.floor((line[:,0]-ox)/res).astype(int);rr=h-1-np.floor((line[:,1]-oy)/res).astype(int)
                if np.all((rr>=0)&(cr>=0)&(rr<h)&(cr<w)) and safe[rr,cr].all():return [left,right]
            split=left+int(deviation.argmax())
            if split in (left,right):split=(left+right)//2
            return simplify(left,split)[:-1]+simplify(split,right)
        points=points[simplify(0,len(points)-1)]
    else:points=cv2.approxPolyDP(points.astype(np.float32),.015,False).reshape(-1,2).astype(float)
    points[0]=start[:2];points[-1]=goal[:2]
    check_deadline(deadline)
    return points,{'used':bool(take.any()),'moved_samples':int(take.sum()),'unmoved_unsafe_samples':int((~take).sum()),
                   'max_shift_m':float(distances[take].max()) if take.any() else 0.,
                   'projection_bound_m':max_shift,'smoothing_m':smoothing_m,'same_search_scope':True,'initialization_only':True}
