"""Diagnostic only: test whether rejected L2 cells missed finer actual poses."""
import math
import numpy as np


def install():
    from three_yd_v5.rectangle_bridge import RectanglePoseCertifier
    from three_yd_v5.pose_batch import first_clear
    original=RectanglePoseCertifier.certify
    def certify(self,cell,preferred_yaw=None):
        old=original(self,cell,preferred_yaw)
        if old is not None:return old
        cell=tuple(map(int,cell));r,c=cell
        if not(0<=r<self.bits.shape[0] and 0<=c<self.bits.shape[1]):return None
        checked=getattr(self,'_r4_fine_checked',None)
        if checked is None:self._r4_fine_checked=checked=set()
        if cell in checked:return None
        checked.add(cell)
        center=self.view.world_points(np.array([r]),np.array([c]))[0]
        fine=self.world.map.resolution
        offsets=np.array([(a,b) for a in (0.,-.5,-1.,.5,1.) for b in (0.,-.5,-1.,.5,1.)])*fine
        offsets=offsets[np.argsort(np.linalg.norm(offsets,axis=1),kind='stable')]
        yaws=2*math.pi*np.arange(96)/96
        if preferred_yaw is not None:yaws=yaws[np.argsort(np.abs(np.arctan2(np.sin(yaws-preferred_yaw),np.cos(yaws-preferred_yaw))),kind='stable')]
        poses=np.empty((len(offsets)*len(yaws),3));poses[:,:2]=np.repeat(center+offsets,len(yaws),axis=0);poses[:,2]=np.tile(yaws,len(offsets))
        index,n=first_clear(self.checker,poses)
        self.batch_calls+=1;self.exact_pose_checks+=n
        if index<0:return None
        oi,yi=divmod(index,len(yaws));result={'cell':list(cell),'cell_center_xy':center.tolist(),
            'xy':poses[index,:2].tolist(),'yaw_rad':float(poses[index,2]),'yaw_bin':int(round(poses[index,2]*96/(2*math.pi)))%96,
            'yaw_bins':96,'subcell_offset_xy':offsets[oi].tolist(),'exact_rectangle_pose':True,'r4_fine_pose_diagnostic':True}
        self.cache[cell]=result
        return result
    RectanglePoseCertifier.certify=certify
