from types import SimpleNamespace
import numpy as np
from scipy.spatial import cKDTree
from three_yd_v5.geometry import variable_neighborhood,corners


def test_world_origin_and_metric_scope_cap():
    origin=np.array([-2.3,-5.1]);res=.05;height=400
    world=SimpleNamespace(hard=np.zeros((height,400),bool),map=SimpleNamespace(resolution=res))
    world.cells=lambda p:(height-1-np.floor((p[:,1]-origin[1])/res).astype(int),np.floor((p[:,0]-origin[0])/res).astype(int))
    world.world_points=lambda r,c:np.column_stack((origin[0]+(c+.5)*res,origin[1]+(height-r-.5)*res))
    p=np.array([[.023,.027],[8.023,.027],[8.023,8.027]])
    mask,info=variable_neighborhood(world,p,corners(p))
    rr,cc=np.nonzero(mask);xy=world.world_points(rr,cc)
    a=p[0];b=p[1];c=p[2]
    distances=[]
    for s,t in [(a,b),(b,c)]:
        d=t-s;u=np.clip((xy-s)@d/(d@d),0,1);distances.append(np.linalg.norm(xy-s-u[:,None]*d,axis=1))
    assert np.maximum(0,np.min(distances,axis=0)-1.2).max()<1e-8
    straight=(xy[:,0]>1)&(xy[:,0]<3)
    assert np.max(distances[0][straight])<=.8+1e-8


def test_l2_rectangle_bridge_marks_existing_1_2m_l3_scope():
    origin=np.array([-2.3,-5.1]);res=.05;height=400
    world=SimpleNamespace(hard=np.zeros((height,400),bool),map=SimpleNamespace(resolution=res))
    world.cells=lambda p:(height-1-np.floor((p[:,1]-origin[1])/res).astype(int),np.floor((p[:,0]-origin[0])/res).astype(int))
    world.world_points=lambda r,c:np.column_stack((origin[0]+(c+.5)*res,origin[1]+(height-r-.5)*res))
    p=np.array([[0.,0.],[6.,0.]])
    base,_=variable_neighborhood(world,p,[])
    forced,info=variable_neighborhood(world,p,[],forced_turning_points=[[3.,0.]])
    assert np.count_nonzero(forced)>np.count_nonzero(base)
    assert info['forced_bridge_points']==1
    assert info['max_sample_radius_m']==1.2
