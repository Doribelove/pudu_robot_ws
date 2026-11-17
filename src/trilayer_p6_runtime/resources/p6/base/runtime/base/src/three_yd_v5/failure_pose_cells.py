"""Diagnostic exact rectangle/cell separating-axis residual and Jacobian.

This supplies optimization constraints only. Original continuous certification
must still validate the output against the original map and vehicle.
"""
import numpy as np
from scipy import sparse
from scipy.spatial import cKDTree

class PoseCells:
    def __init__(self,world,hard,guide):
        res=world.map.resolution;ox,oy=world.map.origin[:2];h,w=hard.shape
        lo=np.min(guide,axis=0)-2;hi=np.max(guide,axis=0)+2
        c0=max(0,int((lo[0]-ox)/res));c1=min(w,int((hi[0]-ox)/res)+2)
        r0=max(0,h-1-int((hi[1]-oy)/res));r1=min(h,h-int((lo[1]-oy)/res)+1)
        rr,cc=np.nonzero(hard[r0:r1,c0:c1])
        self.obs=np.c_[ox+(cc+c0+.5)*res,oy+(h-rr-r0-.5)*res]
        self.tree=cKDTree(self.obs);self.cell=res/2;self.hl,self.hw=world.vehicle[:2]
    def gaps(self,p,v):
        near=self.tree.query_ball_point(p,np.hypot(self.hl,self.hw)+self.cell*1.5+.08)
        sizes=np.array([len(x) for x in near]);idx=np.repeat(np.arange(len(p)),sizes)
        if not len(idx):return idx,np.empty(0),np.empty((0,2)),np.empty(0)
        ob=self.obs[np.concatenate([np.asarray(x,int) for x in near if len(x)])]
        delta=p[idx]-ob;theta=np.arctan2(v[idx,1],v[idx,0]);co=np.cos(theta);si=np.sin(theta)
        ex=np.c_[co,si];ey=np.c_[-si,co];proj=(abs(co)+abs(si))*self.cell
        dproj=(-np.sign(co)*si+np.sign(si)*co)*self.cell
        along=np.sum(delta*ex,axis=1);across=np.sum(delta*ey,axis=1)
        hx=self.hl*abs(co)+self.hw*abs(si);hy=self.hl*abs(si)+self.hw*abs(co)
        dx=-self.hl*np.sign(co)*si+self.hw*np.sign(si)*co
        dy=self.hl*np.sign(si)*co-self.hw*np.sign(co)*si
        gap=np.c_[abs(delta[:,0])-hx-self.cell,abs(delta[:,1])-hy-self.cell,
                  abs(along)-self.hl-proj,abs(across)-self.hw-proj]
        axis=np.argmax(gap,axis=1);value=gap[np.arange(len(idx)),axis]
        gradient=np.zeros((len(idx),2));yaw=np.zeros(len(idx))
        s=axis==0;gradient[s,0]=np.sign(delta[s,0]);yaw[s]=-dx[s]
        s=axis==1;gradient[s,1]=np.sign(delta[s,1]);yaw[s]=-dy[s]
        s=axis==2;gradient[s]=np.sign(along[s,None])*ex[s];yaw[s]=np.sign(along[s])*across[s]-dproj[s]
        s=axis==3;gradient[s]=np.sign(across[s,None])*ey[s];yaw[s]=-np.sign(across[s])*along[s]-dproj[s]
        return idx,value,gradient,yaw
    def linearize(self,B,Y,p,v):
        idx,gap,grad,yaw=self.gaps(p,v);keep=gap<.04
        idx,gap,grad,yaw=idx[keep],gap[keep],grad[keep],yaw[keep]
        G=-sparse.hstack((sparse.diags(grad[:,0])@B[idx],sparse.diags(grad[:,1])@B[idx]),format='csc')-sparse.diags(yaw)@Y[idx]
        return G,1e-6-gap
    def violation(self,p,v):
        idx,gap,_,_=self.gaps(p,v);bad=np.maximum(1e-6-gap,0.)
        worst=np.zeros(len(p));np.maximum.at(worst,idx,bad)
        return float(worst.sum()),float(worst.max(initial=0.))
