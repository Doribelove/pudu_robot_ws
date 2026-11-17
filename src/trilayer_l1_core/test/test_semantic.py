#!/usr/bin/env python3
import copy,json,math,os,tempfile,unittest
from pathlib import Path
from trilayer_l1_core.semantic import SemanticPlanner


def fixture():
    regions=[{'id':i,'name':name,'kind':kind} for i,name,kind in
             [(1,'west','lane'),(2,'junction','junction_area'),(3,'east','lane'),(4,'parking','parking_area')]]
    boxes=[(0,8,8,12),(8,8,12,12),(12,8,20,12),(0,4,4,8)]
    features=[]
    for region,(x0,y0,x1,y1) in zip(regions,boxes):
        features.append({'semantic_id':region['name'],'semantic_class':region['kind'],'geometry_type':'polygon',
                         'properties':{'source_name':region['name']},'coordinates':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]})
    graph={'regions':regions,'edges':[{'kind':'shared_outline','regions':[1,2],'geometry':[[8,8],[8,12]]},
                                    {'kind':'shared_outline','regions':[2,3],'geometry':[[12,8],[12,12]]}]}
    return graph,{'features':features,'traffic_rules':{}},{'bindings':[]}


class SemanticContract(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.old={k:v for k,v in os.environ.items() if k.startswith('THREEYD_')}
        for k in self.old:del os.environ[k]
    def tearDown(self):
        self.temp.cleanup();os.environ.update(self.old)
    def planner(self,graph=None,semantic=None,access=None,mode='astar'):
        g,s,a=fixture();g=g if graph is None else graph;s=s if semantic is None else semantic;a=a if access is None else access
        for name,value in [('graph.json',g),('semantic.json',s),('parking_access.json',a)]:
            (self.root/name).write_text(json.dumps(value))
        return SemanticPlanner(self.root/'graph.json',self.root/'cache',width=200,height=160,resolution=.1,mode=mode)
    def test_real_constructor_and_route_task(self):
        for mode in ('astar','legacy'):
            planner=self.planner(mode=mode)
            routes=planner.routes('q',(2.,10.,0.),(18.,10.,0.))
            self.assertTrue(routes)
            route,task,context=routes[0]
            self.assertEqual(route.region_ids,(1,2,3))
            self.assertEqual(task['region_sequence'],[1,2,3])
            self.assertGreaterEqual(len(task['right_reference_polyline']),2)
            self.assertEqual(context['endpoint_access']['start']['semantic_region_id'],1)
            self.assertTrue((self.root/'cache/v5_topology.json').is_file())
    def test_disconnected_graph(self):
        g,s,a=fixture();g['edges']=g['edges'][:1]
        self.assertEqual(self.planner(g,s,a).routes('q',(2.,10.,0.),(18.,10.,0.)),[])
    def test_one_way_and_forbidden_transition(self):
        g,s,a=fixture();s['traffic_rules']['one_way']=[{'semantic_id':'west','yaw':0.}]
        planner=self.planner(g,s,a)
        self.assertTrue(planner.routes('forward',(2.,10.,0.),(18.,10.,0.)))
        self.assertEqual(planner.routes('reverse',(18.,10.,math.pi),(2.,10.,math.pi)),[])
        s['traffic_rules']['forbidden_transitions']=[['west','junction','east']]
        self.assertEqual(self.planner(g,s,a).routes('blocked',(2.,10.,0.),(18.,10.,0.)),[])
    def test_parking_binding_and_anchor(self):
        g,s,a=fixture()
        with self.assertRaisesRegex(ValueError,'PARKING_OWNERSHIP_UNRESOLVED'):
            self.planner(g,s,a).routes('park',(2.,6.,0.),(18.,10.,0.))
        a['bindings']=[{'query_id':'park','endpoint':'start','parking_region_id':4,
                        'road_id':1,'anchor_xy':[2.,6.],'anchor_tolerance_m':.25,
                        'travel_sides':['bottom'],'allow_local_cross_road':True}]
        planner=self.planner(g,s,a);routes=planner.routes('park',(2.,6.,0.),(18.,10.,0.))
        self.assertTrue(routes);route,task,context=routes[0]
        self.assertEqual(route.region_ids[0],4)
        self.assertTrue(context['endpoint_access']['start']['allow_local_cross_road'])
        with self.assertRaisesRegex(ValueError,'PARKING_BINDING_POSITION_MISMATCH'):
            planner.routes('park',(3.,6.,0.),(18.,10.,0.))
        a['bindings'][0]['travel_sides']=['top']
        self.assertEqual(self.planner(g,s,a).routes('park',(2.,6.,0.),(18.,10.,0.)),[])
    def test_alternative_semantic_corridor(self):
        specs=[(1,'west','lane',(0,4,4,12)),(2,'nw','junction_area',(0,12,4,16)),
               (3,'north','lane',(4,12,16,16)),(4,'ne','junction_area',(16,12,20,16)),
               (5,'east','lane',(16,4,20,12)),(6,'se','junction_area',(16,0,20,4)),
               (7,'south','lane',(4,0,16,4)),(8,'sw','junction_area',(0,0,4,4))]
        g={'regions':[],'edges':[]};s={'features':[],'traffic_rules':{}}
        for i,name,kind,(x0,y0,x1,y1) in specs:
            g['regions'].append({'id':i,'name':name,'kind':kind})
            s['features'].append({'semantic_id':name,'properties':{'source_name':name},
                                  'coordinates':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]})
        for a,b,xy in [(1,2,[[0,12],[4,12]]),(2,3,[[4,12],[4,16]]),
                       (3,4,[[16,12],[16,16]]),(4,5,[[16,12],[20,12]]),
                       (5,6,[[16,4],[20,4]]),(6,7,[[16,0],[16,4]]),
                       (7,8,[[4,0],[4,4]]),(8,1,[[0,4],[4,4]])]:
            g['edges'].append({'kind':'shared_outline','regions':[a,b],'geometry':xy})
        routes=self.planner(g,s,{'bindings':[]}).routes('two',(2.,8.,0.),(18.,8.,0.),limit=3)
        sequences={r.region_ids for r,_,_ in routes}
        self.assertIn((1,2,3,4,5),sequences)
        self.assertIn((1,8,7,6,5),sequences)
        s['traffic_rules']['forbidden_transitions']=[['west','nw','north']]
        routes=self.planner(g,s,{'bindings':[]}).routes('one',(2.,8.,0.),(18.,8.,0.),limit=3)
        self.assertTrue(routes)
        self.assertTrue(all(3 not in r.region_ids for r,_,_ in routes))
        self.assertIn((1,8,7,6,5),{r.region_ids for r,_,_ in routes})

    def test_outside_semantic_endpoint(self):
        with self.assertRaisesRegex(ValueError,'ENDPOINT'):
            self.planner().routes('outside',(2.,1.,0.),(18.,10.,0.))


if __name__=='__main__':unittest.main(verbosity=2)
