"""Routing-filter contracts independent of Nav2 and elapsed-time measurements."""
import copy,json
from types import SimpleNamespace
from collections import defaultdict,OrderedDict
import numpy as np
from three_yd_v13.runtime_l1 import RuntimeSemanticTopology
class Base:
    def routes(self,q,limit=1):return []
class World:
    def __init__(self):
        self.map=SimpleNamespace(height=21,width=41,resolution=1.)
        self.hard=np.zeros((21,41),bool)
    def cells(self,p):return np.rint(p[:,1]).astype(int),np.rint(p[:,0]).astype(int)
def fixture(polygon):
 w=World();b=Base();b.world=w;b.static_hard=w.hard.copy();b.key='frozen';b.tasks={};b.contexts={};b._cache={};b._first_phase_cache=OrderedDict()
 b.parent_semantics={1:1};b.regions={1:{'name':'lane','kind':'lane'}};b.router=SimpleNamespace(features={'lane':{'coordinates':[[1,5],[39,5],[39,15],[1,15]]}},key='frozen',polygons={1:None},axes={1:np.array([1.,0.])})
 b.adj=defaultdict(set,{1:{2,3}});b.contacts={(1,2):[[2,5],[2,15]],(1,3):[[38,5],[38,15]]}
 b._static_transitions={(2,1):{3:((2.,10.),(38.,10.),(1.,0.),36.,None)},(3,1):{2:((38.,10.),(2.,10.),(-1.,0.),36.,'original_forbidden')}}
 import cv2
 mask=np.zeros(w.hard.shape,np.uint8);cv2.fillPoly(mask,[np.array(polygon,np.int32)],1);mask=mask.astype(bool);w.hard=w.hard|mask
 d=SimpleNamespace(version=1,content_hash='mask',mask=mask,obstacles={'x':np.array(polygon,float)})
 return w,b,d
w,b,d=fixture([[18,8],[22,8],[22,12],[18,12]]);a=RuntimeSemanticTopology(b,w,d);assert not a.blocked,'small obstacle must retain around-path'
w,b,d=fixture([[18,4],[22,4],[22,16],[18,16]]);before=copy.deepcopy(b._static_transitions);a=RuntimeSemanticTopology(b,w,d)
assert a.blocked==[[2,1,3]],a.blocked
assert a.top._static_transitions[3,1][2][-1]=='original_forbidden'
assert b._static_transitions==before and b.router.key=='frozen'
assert a.top._cache is not b._cache and a.top.tasks is not b.tasks
assert a.top.key!=b.key
assert len(a.reverse_nodes)==2;assert set(a.reverse_nodes.values())=={1}
assert len({a for a,b in a.graph.contacts})==2
assert a.data;a.release_workspaces();assert not a.data and not a.ports and not a.query_tops
print(json.dumps({'passed':7,'cases':['partial blocker preserves connectivity','full cut removed','original restriction preserved','frozen graph immutable','query cache isolated','runtime identity distinct','temporary arrays released']}))
