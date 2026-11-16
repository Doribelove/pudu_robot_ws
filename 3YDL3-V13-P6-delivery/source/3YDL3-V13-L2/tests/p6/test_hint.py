import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from three_yd_v5 import p6_hint as hint


class HintContract(unittest.TestCase):
    def runner(self, semantic=False):
        return NS(dynamic=NS(obstacles={'x':np.array([[1,1],[2,1],[2,2],[1,2]])}),
                  topology=NS(mode='semantic' if semantic else 'free_space'))

    def session(self, expired=False):
        return NS(_curve_request=dict(deadline=time.monotonic()+(-1 if expired else 30),
                  config={'new_l3_budget_s':15},new_used_s=0))

    def test_context_cleans_on_exception(self):
        original=hint.cache_key()
        with patch.dict('os.environ',V13_P6_HINT_MODE='local'):
            with self.assertRaises(ValueError):
                with hint.request_scope(self.runner(),self.session()):
                    self.assertNotEqual(original,hint.cache_key())
                    raise ValueError('failure')
        self.assertEqual(original,hint.cache_key())

    def test_semantic_and_clear_disabled(self):
        for runner in [self.runner(True),NS(dynamic=NS(obstacles={}),topology=NS(mode='free_space'))]:
            with hint.request_scope(runner,self.session()):self.assertIsNone(hint._context.get())

    def test_variant_cache_binding_and_thread_isolation(self):
        found=[]
        with hint.request_scope(self.runner(),self.session()):
            base=hint.cache_key();hint._context.get()['variant']='local'
            self.assertNotEqual(base,hint.cache_key())
            t=threading.Thread(target=lambda:found.append(hint.cache_key()));t.start();t.join()
            self.assertEqual(found,[('P5',)])

    def test_budget_and_L3_rejection_do_not_expand(self):
        runner=self.runner()
        for expired,stage in [(True,'L2'),(False,'L3'),(False,'infrastructure')]:
            session=self.session(expired)
            first=(None,None,dict(success=False,failure_stage=stage,failure_code='TEST'))
            with hint.request_scope(runner,session),patch('three_yd_v5.request_schedule.scheduled_plan') as solve:
                actual,_=hint.retry_short_hint(runner,None,session,None,None,first,{})
                self.assertIs(actual,first);solve.assert_not_called()

    def test_retry_precedes_original_scope_and_uses_same_budget(self):
        runner=self.runner();session=self.session();state=session._curve_request
        first=(None,None,dict(success=False,failure_stage='L2',failure_code='DISCONNECTED',candidate_recovery={'attempts':[{'first':True}],'wall_ms':2}))
        def solve(*args):
            self.assertIs(session._curve_request,state)
            self.assertEqual(hint._context.get()['variant'],'local')
            return (None,None,dict(success=True,candidate_recovery={'attempts':[{'expanded':True}],'wall_ms':3}))
        with patch.dict('os.environ',V13_P6_HINT_MODE='local'),hint.request_scope(runner,session),patch('three_yd_v5.request_schedule.scheduled_plan',side_effect=solve):
            out,detail=hint.retry_short_hint(runner,None,session,None,None,first,{})
            self.assertEqual(out[2]['candidate_recovery']['wall_ms'],5)
            self.assertEqual(len(out[2]['candidate_recovery']['attempts']),2)
            self.assertTrue(detail['p6_hint_recovery']['cold_domain_rebuild'])
            self.assertFalse(detail['p6_hint_recovery']['old_mutable_Dstar_state_reused'])
            self.assertEqual(hint._context.get()['variant'],'base')

if __name__=='__main__':unittest.main()
