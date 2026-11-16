import threading
import time
import unittest
from dataclasses import dataclass,replace
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from three_yd_v13 import schedule
from three_yd_v13.service import IncrementalService

@dataclass
class Query:
    start:tuple=(0.,0.,0.)
    goal:tuple=(5.,5.,0.)

class Scheduling(unittest.TestCase):
    def service(self):
        q=Query();s=NS(query=q,prefix=[],_p5_active=True,_p5_legacy=False,
            session=NS(_curve_request={'deadline':time.monotonic()+30,'config':{'new_l3_budget_s':15},'new_used_s':0}),
            record=None,remaining=None,retained_record=None,retained_path=None,controller=None)
        return s

    def test_short_tolerance_precedes_original_full_recovery(self):
        s=self.service();q=s.query;calls=[]
        plus=replace(q,goal=(5.,5.,.087));minus=replace(q,goal=(5.,5.,-.087))
        def solve(x):
            calls.append((x,s._p5_legacy));s.query=x
            return {'success':s._p5_legacy,'failure_code':'' if s._p5_legacy else 'CURVE_TEST_FAILURE'}
        s._plan=solve
        with patch.object(schedule.endpoints,'policy',return_value={'enabled':True}),patch.object(schedule.endpoints,'candidates',return_value=iter([plus,minus])),patch.object(schedule.endpoints,'attach',side_effect=lambda s,o,q,r,c,a:{**o,'attempts':a}):
            o=schedule.after_update(s,{'success':False,'failure_code':'CURVE_TEST_FAILURE'},q)
        self.assertTrue(o['success']);self.assertEqual(calls,[(plus,False),(minus,False),(q,True)])
        self.assertFalse(s._p5_legacy);self.assertEqual(q.goal,(5.,5.,0.))

    def test_shared_budget_stops_further_candidates(self):
        s=self.service();q=s.query;calls=[]
        def solve(x):
            calls.append(x);s.session._curve_request['new_used_s']=15
            return {'success':False,'failure_code':'CURVE_TEST_FAILURE'}
        s._plan=solve
        with patch.object(schedule.endpoints,'policy',return_value={'enabled':True}),patch.object(schedule.endpoints,'candidates',return_value=iter([replace(q,goal=(5.,5.,.087)),replace(q,goal=(5.,5.,-.087))])),patch.object(schedule.endpoints,'attach',side_effect=lambda s,o,*args:o):
            schedule.after_update(s,{'success':False,'failure_code':'CURVE_TEST_FAILURE'},q)
        self.assertEqual(len(calls),1);self.assertEqual(s.session._curve_request['new_used_s'],15)

    def test_request_cache_not_reused_after_exception(self):
        s=NS(lock=threading.RLock())
        def fail(*args,**kwargs):s._p5_cache['example']='data';raise ValueError('test')
        s._p4_guarded_update=fail
        with self.assertRaises(ValueError):IncrementalService._guarded_update(s,1)
        self.assertFalse(s._p5_active);self.assertEqual(s._p5_cache,{})

    def test_cache_identity_covers_obstacles_vehicle_and_policy(self):
        s=NS(world=NS(key='map',vehicle=(.265,.225,.4),hard=np.zeros((3,3),bool)),dynamic=NS(version=1,mask=np.zeros((3,3),bool)))
        with patch('three_yd_v5.v12_regional.current',return_value=NS(fingerprint='policy1')):
            original=schedule.cache_identity(s);s.dynamic.version=2
            self.assertNotEqual(original,schedule.cache_identity(s));s.dynamic.version=1
            s.world.hard[1,1]=True;self.assertNotEqual(original,schedule.cache_identity(s));s.world.hard[1,1]=False
            s.dynamic.mask[1,1]=True;self.assertNotEqual(original,schedule.cache_identity(s));s.dynamic.mask[1,1]=False
            s.world.vehicle=(.3,.3,.4);self.assertNotEqual(original,schedule.cache_identity(s));s.world.vehicle=(.265,.225,.4)
        with patch('three_yd_v5.v12_regional.current',return_value=NS(fingerprint='policy2')):
            self.assertNotEqual(original,schedule.cache_identity(s))

if __name__=='__main__':unittest.main()
