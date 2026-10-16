from types import SimpleNamespace
import numpy as np

from three_yd_v5.r6_cache import BoundedLRU,RoiGrid
from three_yd_v5.recovery import V5Runner


def test_bounded_lru_evicts_oldest_and_promotes_hits():
    cache=BoundedLRU(2)
    cache['a']=1;cache['b']=2
    assert cache['a']==1
    cache['c']=3
    assert 'a' in cache and 'c' in cache and 'b' not in cache
    assert cache.evictions==1


def test_roi_grid_preserves_global_index_contract():
    local=np.arange(12,dtype=np.float32).reshape(3,4)
    field=RoiGrid(local,(2,5,7,11),(10,20),-1.)
    r=np.array([1,2,4,5]);c=np.array([7,7,10,10])
    assert field.shape==(10,20)
    assert np.array_equal(field[r,c],np.array([-1,0,11,-1],np.float32))


def test_candidate_cache_prefers_success_and_invalidates_on_dynamic_change(tmp_path):
    routes=[SimpleNamespace(signature='bad',region_ids=(1,)),
            SimpleNamespace(signature='good',region_ids=(2,))]
    topology=SimpleNamespace(routes=lambda query,limit:routes[:limit])
    world=SimpleNamespace(map=SimpleNamespace(sha256='map'),
                          semantic=SimpleNamespace(semantic_map_hash='semantic'),
                          vehicle=(.5,.3))
    dynamic=SimpleNamespace(version=0,mask=np.zeros((2,2),bool))
    created=[]
    class Controller:
        def __init__(self,world,topology,route,**kwargs):
            self.route=route;created.append(route.signature)
            self.lifecycle=SimpleNamespace(clear=lambda:None)
        def plan_l3(self,*args):
            if self.route.signature=='bad':
                return {'success':False,'failure_code':'RAW_STEERING_JUMP'}
            return {'success':True,'failure_code':''}
    query=SimpleNamespace(start=(0.,0.,0.),goal=(1.,1.,0.))
    views={};runner=V5Runner(world,topology,tmp_path,views,dynamic,controller_class=Controller)
    _,_,first=runner.plan(query,SimpleNamespace(),None,None)
    assert created==['bad','good'] and first['success']
    _,_,second=runner.plan(query,SimpleNamespace(),None,None)
    assert created==['bad','good','good'] and second['success']
    assert second['candidate_recovery']['preferred_route_hit']
    dynamic.version=1
    _,_,third=runner.plan(query,SimpleNamespace(),None,None)
    assert created[-2:]==['bad','good'] and third['success']
